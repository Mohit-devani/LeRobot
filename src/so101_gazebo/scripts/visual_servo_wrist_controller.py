#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class VisualServoWristController(Node):

    def __init__(self):
        super().__init__("visual_servo_wrist_controller")

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

        self.y_tolerance = 35.0
        self.max_wrist_step = 0.06
        self.min_wrist_step = 0.015

        self.wrist_min = -0.9
        self.wrist_max = 0.4

        self.gain_sign = None

        self.get_logger().info("Visual servo WRIST controller started")
        self.get_logger().info("Goal: reduce /visual_servo_error.y toward 0")

    def error_callback(self, msg):
        self.latest_error = msg

    def joint_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.joints[name] = pos

    def wait_for_data(self):
        self.get_logger().info("Waiting for /visual_servo_error and /joint_states...")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)

            needed = [
                "shoulder_pan",
                "shoulder_lift",
                "elbow_flex",
                "wrist_flex",
                "wrist_roll"
            ]

            if self.latest_error is not None and all(j in self.joints for j in needed):
                return True

        return False

    def clamp_wrist(self, value):
        return max(self.wrist_min, min(self.wrist_max, value))

    def get_current_arm_pose(self):
        return [
            float(self.joints.get("shoulder_pan", 0.0)),
            float(self.joints.get("shoulder_lift", -0.5)),
            float(self.joints.get("elbow_flex", 0.5)),
            float(self.joints.get("wrist_flex", -0.3)),
            float(self.joints.get("wrist_roll", 0.0)),
        ]

    def send_arm_goal(self, positions, duration=2):
        positions[3] = self.clamp_wrist(positions[3])

        self.get_logger().info(
            f"Sending wrist goal: wrist_flex={positions[3]:.3f}, full pose={positions}"
        )

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
            self.get_logger().info("Wrist goal done")
            return True

        self.get_logger().error(f"Wrist goal failed: {result.error_string}")
        return False

    def wait_and_update(self, seconds=2.0):
        end_time = time.time() + seconds

        while time.time() < end_time and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def calibrate_direction(self):
        err0 = self.latest_error.y
        pose0 = self.get_current_arm_pose()
        wrist0 = pose0[3]

        if abs(err0) < self.y_tolerance:
            self.get_logger().info("Already vertically centered")
            return True

        self.get_logger().info(
            f"Calibration start: error_y={err0:.1f}, wrist_flex={wrist0:.3f}"
        )

        probe_step = 0.06
        pose_probe = pose0.copy()
        pose_probe[3] = wrist0 + probe_step

        self.send_arm_goal(pose_probe, duration=2)
        self.wait_and_update(2.0)

        err1 = self.latest_error.y
        self.get_logger().info(f"Calibration after +wrist: error_y={err1:.1f}")

        sign_error0 = 1.0 if err0 >= 0 else -1.0

        if abs(err1) < abs(err0):
            self.gain_sign = 1.0 / sign_error0
            self.get_logger().info("Calibration: +wrist improved error")
        else:
            self.gain_sign = -1.0 / sign_error0
            self.get_logger().info("Calibration: +wrist worsened error, reversing direction")

        self.get_logger().info(f"Calibrated gain_sign={self.gain_sign:.1f}")
        return True

    def run(self):
        self.wait_for_data()
        self.calibrate_direction()

        last_abs_error = abs(self.latest_error.y)

        for i in range(20):
            self.wait_and_update(0.5)

            error_y = self.latest_error.y
            area = self.latest_error.z
            pose = self.get_current_arm_pose()
            wrist = pose[3]

            self.get_logger().info(
                f"Loop {i}: error_y={error_y:.1f}, area={area:.1f}, wrist_flex={wrist:.3f}"
            )

            if abs(error_y) < self.y_tolerance:
                self.get_logger().info("Vertical centering complete")
                return

            step_mag = abs(error_y) * 0.00035
            step_mag = max(self.min_wrist_step, min(self.max_wrist_step, step_mag))

            error_sign = 1.0 if error_y >= 0 else -1.0
            wrist_step = self.gain_sign * error_sign * step_mag

            new_pose = pose.copy()
            new_pose[3] = self.clamp_wrist(wrist + wrist_step)

            self.get_logger().info(
                f"Applying wrist_step={wrist_step:.3f}, new_wrist={new_pose[3]:.3f}"
            )

            self.send_arm_goal(new_pose, duration=2)
            self.wait_and_update(2.0)

            new_abs_error = abs(self.latest_error.y)

            if new_abs_error > last_abs_error + 20:
                self.gain_sign *= -1.0
                self.get_logger().warn(
                    f"Error increased from {last_abs_error:.1f} to {new_abs_error:.1f}. Reversing gain_sign."
                )

            last_abs_error = new_abs_error

        self.get_logger().warn("Stopped after 20 loops. Check final /visual_servo_error.")


def main(args=None):
    rclpy.init(args=args)

    node = VisualServoWristController()

    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
