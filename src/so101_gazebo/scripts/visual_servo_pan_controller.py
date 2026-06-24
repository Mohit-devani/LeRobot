#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class VisualServoPanController(Node):

    def __init__(self):
        super().__init__("visual_servo_pan_controller")

        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory"
        )

        self.error_sub = self.create_subscription(
            Point,
            "/visual_servo_error",
            self.error_callback,
            10
        )

        self.joint_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            10
        )

        self.latest_error = None
        self.joints = {}

        self.arm_joints = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll"
        ]

        # Keep these fixed from our camera-friendly pose
        self.shoulder_lift = -0.5
        self.elbow_flex = 0.5
        self.wrist_flex = -0.3
        self.wrist_roll = 0.0

        self.x_tolerance = 35.0
        self.max_pan_step = 0.07
        self.min_pan_step = 0.02
        self.pan_limit = 0.9

        # This will be calibrated automatically
        self.gain_sign = None

        self.get_logger().info("Visual servo PAN controller started")
        self.get_logger().info("Goal: reduce /visual_servo_error.x toward 0")

    def error_callback(self, msg):
        self.latest_error = msg

    def joint_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.joints[name] = pos

    def wait_for_data(self):
        self.get_logger().info("Waiting for /visual_servo_error and /joint_states...")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)

            if self.latest_error is not None and "shoulder_pan" in self.joints:
                return True

        return False

    def current_pan(self):
        return float(self.joints.get("shoulder_pan", 0.0))

    def clamp_pan(self, value):
        return max(-self.pan_limit, min(self.pan_limit, value))

    def send_pan_goal(self, pan, duration=2):
        pan = self.clamp_pan(pan)

        positions = [
            pan,
            self.shoulder_lift,
            self.elbow_flex,
            self.wrist_flex,
            self.wrist_roll
        ]

        self.get_logger().info(f"Sending pan goal: {pan:.3f}")

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.arm_joints

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = duration
        point.time_from_start.nanosec = 0

        goal_msg.trajectory.points.append(point)

        self.arm_client.wait_for_server()

        future = self.arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Arm goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result

        if result.error_code == 0:
            self.get_logger().info("Pan goal done")
            return True

        self.get_logger().error(f"Pan goal failed: {result.error_string}")
        return False

    def wait_and_update(self, seconds=2.0):
        end_time = time.time() + seconds

        while time.time() < end_time and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def calibrate_direction(self):
        err0 = self.latest_error.x
        pan0 = self.current_pan()

        if abs(err0) < self.x_tolerance:
            self.get_logger().info("Already horizontally centered")
            return True

        self.get_logger().info(f"Calibration start: error_x={err0:.1f}, pan={pan0:.3f}")

        probe_step = 0.08
        self.send_pan_goal(pan0 + probe_step, duration=2)
        self.wait_and_update(2.0)

        err1 = self.latest_error.x
        self.get_logger().info(f"Calibration after +pan: error_x={err1:.1f}")

        sign_error0 = 1.0 if err0 >= 0 else -1.0

        if abs(err1) < abs(err0):
            # +pan helped for this initial error
            self.gain_sign = 1.0 / sign_error0
            self.get_logger().info("Calibration: +pan improved error")
        else:
            # +pan made it worse, reverse direction
            self.gain_sign = -1.0 / sign_error0
            self.get_logger().info("Calibration: +pan worsened error, reversing direction")

        self.get_logger().info(f"Calibrated gain_sign={self.gain_sign:.1f}")
        return True

    def run(self):
        self.wait_for_data()
        self.calibrate_direction()

        last_abs_error = abs(self.latest_error.x)

        for i in range(20):
            self.wait_and_update(0.5)

            error_x = self.latest_error.x
            area = self.latest_error.z
            pan = self.current_pan()

            self.get_logger().info(
                f"Loop {i}: error_x={error_x:.1f}, area={area:.1f}, current_pan={pan:.3f}"
            )

            if abs(error_x) < self.x_tolerance:
                self.get_logger().info("Horizontal centering complete")
                return

            step_mag = abs(error_x) * 0.0004
            step_mag = max(self.min_pan_step, min(self.max_pan_step, step_mag))

            error_sign = 1.0 if error_x >= 0 else -1.0
            pan_step = self.gain_sign * error_sign * step_mag

            new_pan = self.clamp_pan(pan + pan_step)

            self.get_logger().info(
                f"Applying pan_step={pan_step:.3f}, new_pan={new_pan:.3f}"
            )

            self.send_pan_goal(new_pan, duration=2)
            self.wait_and_update(2.0)

            new_abs_error = abs(self.latest_error.x)

            # Safety: if error gets worse, reverse once
            if new_abs_error > last_abs_error + 20:
                self.gain_sign *= -1.0
                self.get_logger().warn(
                    f"Error increased from {last_abs_error:.1f} to {new_abs_error:.1f}. Reversing gain_sign."
                )

            last_abs_error = new_abs_error

        self.get_logger().warn("Stopped after 20 loops. Check final /visual_servo_error.")


def main(args=None):
    rclpy.init(args=args)

    node = VisualServoPanController()

    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
