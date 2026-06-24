#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class VisualServoCenteringSequence(Node):

    def __init__(self):
        super().__init__("visual_servo_centering_sequence")

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

        self.detected_sub = self.create_subscription(
            Bool,
            "/object_detected",
            self.detected_callback,
            10
        )

        self.joint_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            10
        )

        self.center_done_pub = self.create_publisher(
            Bool,
            "/visual_servo_sequence_done",
            10
        )

        self.latest_error = None
        self.object_detected = False
        self.joints = {}

        self.arm_joints = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll"
        ]

        self.x_tolerance = 35.0
        self.y_tolerance = 35.0

        self.pan_limit = 0.9
        self.wrist_min = -0.9
        self.wrist_max = 0.4

        self.get_logger().info("Visual servo centering sequence started")

    def error_callback(self, msg):
        self.latest_error = msg

    def detected_callback(self, msg):
        self.object_detected = msg.data

    def joint_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.joints[name] = pos

    def wait_for_data(self):
        self.get_logger().info("Waiting for object detection, visual error, and joint states...")

        needed = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll"
        ]

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)

            if (
                self.object_detected and
                self.latest_error is not None and
                all(j in self.joints for j in needed)
            ):
                self.get_logger().info("All required data received")
                return True

        return False

    def current_pose(self):
        return [
            float(self.joints.get("shoulder_pan", 0.0)),
            float(self.joints.get("shoulder_lift", -0.5)),
            float(self.joints.get("elbow_flex", 0.5)),
            float(self.joints.get("wrist_flex", -0.3)),
            float(self.joints.get("wrist_roll", 0.0)),
        ]

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def send_arm_goal(self, pose, duration=1):
        pose[0] = self.clamp(pose[0], -self.pan_limit, self.pan_limit)
        pose[3] = self.clamp(pose[3], self.wrist_min, self.wrist_max)

        self.get_logger().info(
            f"ARM GOAL pan={pose[0]:.3f}, wrist={pose[3]:.3f}, pose={pose}"
        )

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.arm_joints

        point = JointTrajectoryPoint()
        point.positions = pose
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
            return True

        self.get_logger().error(f"Arm goal failed: {result.error_string}")
        return False

    def wait_update(self, seconds=2.0):
        end_time = time.time() + seconds

        while time.time() < end_time and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def calibrate_axis(self, axis):
        pose0 = self.current_pose()

        if axis == "x":
            err0 = self.latest_error.x
            joint_index = 0
            probe_step = 0.08
            label = "pan"
        else:
            err0 = self.latest_error.y
            joint_index = 3
            probe_step = 0.06
            label = "wrist"

        self.get_logger().info(f"Calibrating {axis}: start error={err0:.1f}")

        pose_probe = pose0.copy()
        pose_probe[joint_index] += probe_step

        self.send_arm_goal(pose_probe, duration=1)
        self.wait_update(0.25)

        err1 = self.latest_error.x if axis == "x" else self.latest_error.y

        self.get_logger().info(
            f"Calibration {label}: after positive step error={err1:.1f}"
        )

        sign_error0 = 1.0 if err0 >= 0 else -1.0

        if abs(err1) < abs(err0):
            gain_sign = 1.0 / sign_error0
            self.get_logger().info(f"{label}: positive step improved error")
        else:
            gain_sign = -1.0 / sign_error0
            self.get_logger().info(f"{label}: positive step worsened error, reversing")

        self.get_logger().info(f"{label}: calibrated gain_sign={gain_sign:.1f}")

        return gain_sign

    def center_x(self):
        self.get_logger().info("Starting horizontal centering")

        if abs(self.latest_error.x) < self.x_tolerance:
            self.get_logger().info("Already horizontally centered")
            return True

        gain_sign = self.calibrate_axis("x")
        last_abs_error = abs(self.latest_error.x)

        for i in range(20):
            self.wait_update(0.25)

            error_x = self.latest_error.x
            pose = self.current_pose()

            self.get_logger().info(
                f"PAN loop {i}: error_x={error_x:.1f}, current_pan={pose[0]:.3f}"
            )

            if abs(error_x) < self.x_tolerance:
                self.get_logger().info("Horizontal centering complete")
                return True

            step_mag = abs(error_x) * 0.0004
            step_mag = max(0.02, min(0.07, step_mag))

            error_sign = 1.0 if error_x >= 0 else -1.0
            pan_step = gain_sign * error_sign * step_mag

            new_pose = pose.copy()
            new_pose[0] += pan_step

            self.send_arm_goal(new_pose, duration=1)
            self.wait_update(0.25)

            new_abs_error = abs(self.latest_error.x)

            if new_abs_error > last_abs_error + 20:
                gain_sign *= -1.0
                self.get_logger().warn("PAN error increased. Reversing direction.")

            last_abs_error = new_abs_error

        self.get_logger().warn("Horizontal centering stopped after loop limit")
        return False

    def center_y(self):
        self.get_logger().info("Starting vertical centering")

        if abs(self.latest_error.y) < self.y_tolerance:
            self.get_logger().info("Already vertically centered")
            return True

        gain_sign = self.calibrate_axis("y")
        last_abs_error = abs(self.latest_error.y)

        for i in range(20):
            self.wait_update(0.25)

            error_y = self.latest_error.y
            pose = self.current_pose()

            self.get_logger().info(
                f"WRIST loop {i}: error_y={error_y:.1f}, current_wrist={pose[3]:.3f}"
            )

            if abs(error_y) < self.y_tolerance:
                self.get_logger().info("Vertical centering complete")
                return True

            step_mag = abs(error_y) * 0.00035
            step_mag = max(0.015, min(0.06, step_mag))

            error_sign = 1.0 if error_y >= 0 else -1.0
            wrist_step = gain_sign * error_sign * step_mag

            new_pose = pose.copy()
            new_pose[3] += wrist_step

            self.send_arm_goal(new_pose, duration=1)
            self.wait_update(0.25)

            new_abs_error = abs(self.latest_error.y)

            if new_abs_error > last_abs_error + 20:
                gain_sign *= -1.0
                self.get_logger().warn("WRIST error increased. Reversing direction.")

            last_abs_error = new_abs_error

        self.get_logger().warn("Vertical centering stopped after loop limit")
        return False

    def publish_done(self, value):
        msg = Bool()
        msg.data = value

        for _ in range(5):
            self.center_done_pub.publish(msg)
            time.sleep(0.1)

    def run(self):
        self.wait_for_data()

        x_ok = self.center_x()
        self.wait_update(0.25)

        y_ok = self.center_y()
        self.wait_update(0.25)

        final_x = self.latest_error.x
        final_y = self.latest_error.y

        centered = (
            abs(final_x) < self.x_tolerance and
            abs(final_y) < self.y_tolerance
        )

        self.get_logger().info(
            f"FINAL visual error: x={final_x:.1f}, y={final_y:.1f}, centered={centered}"
        )

        self.publish_done(centered)

        if centered:
            self.get_logger().info("VISUAL CENTERING SEQUENCE COMPLETE")
        else:
            self.get_logger().warn("VISUAL CENTERING SEQUENCE FINISHED BUT NOT CENTERED")

        # Keep publishing final state briefly so other nodes / ros2 topic echo can catch it.
        self.get_logger().info("Holding /visual_servo_sequence_done for 10 seconds")
        for _ in range(20):
            self.publish_done(centered)
            time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)

    node = VisualServoCenteringSequence()

    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
