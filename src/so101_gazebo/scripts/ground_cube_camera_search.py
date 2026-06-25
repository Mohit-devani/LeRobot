#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class GroundCubeCameraSearch(Node):

    def __init__(self):
        super().__init__("ground_cube_camera_search")

        self.object_detected = False
        self.center = None

        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory"
        )

        self.object_sub = self.create_subscription(
            Bool,
            "/object_detected",
            self.object_callback,
            10
        )

        self.center_sub = self.create_subscription(
            Point,
            "/detected_object_center",
            self.center_callback,
            10
        )

        self.arm_joints = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll"
        ]

        # Candidate poses for looking downward / forward toward ground cube.
        # We are testing vision only here, not full pick.
        # Fixed stable pose for ground-cube camera detection.
        # Do not keep scanning after this. First goal: stable detection.
        self.search_poses = [
            [0.0, -0.35, 0.70, 0.60, 0.0],
        ]

    def object_callback(self, msg):
        self.object_detected = msg.data

    def center_callback(self, msg):
        self.center = msg

    def wait_for_future(self, future, timeout_sec=20.0):
        start = time.time()

        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.02)

            if timeout_sec is not None and time.time() - start > timeout_sec:
                self.get_logger().error("Future timeout")
                return False

        return True

    def move_arm(self, positions, duration=3):
        self.get_logger().info(f"SEARCH ARM POSE: {positions}")

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.arm_joints

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = duration

        goal_msg.trajectory.points.append(point)

        self.arm_client.wait_for_server()

        future = self.arm_client.send_goal_async(goal_msg)

        if not self.wait_for_future(future, timeout_sec=10.0):
            return False

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Arm goal rejected")
            return False

        result_future = goal_handle.get_result_async()

        if not self.wait_for_future(result_future, timeout_sec=duration + 10.0):
            return False

        result = result_future.result().result

        if result.error_code == 0:
            self.get_logger().info("Search pose reached")
            return True

        self.get_logger().error(f"Arm goal failed: {result.error_string}")
        return False

    def wait_for_detection(self, seconds=4.0):
        start = time.time()

        while rclpy.ok() and time.time() - start < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)

            if self.object_detected and self.center is not None:
                self.get_logger().info(
                    f"GROUND CUBE DETECTED: cx={self.center.x:.1f}, cy={self.center.y:.1f}, area={self.center.z:.1f}"
                )
                return True

        return False

    def run(self):
        self.get_logger().info("Ground cube camera search started")

        time.sleep(2.0)

        for i, pose in enumerate(self.search_poses):
            self.get_logger().info(f"Trying search pose {i + 1}/{len(self.search_poses)}")

            self.object_detected = False
            self.center = None

            self.move_arm(pose, duration=3)

            if self.wait_for_detection(seconds=4.0):
                self.get_logger().info(f"FOUND WORKING GROUND-CUBE CAMERA POSE: {pose}")
                self.get_logger().info("Ground cube vision test PASSED")
                return

        self.get_logger().error("Ground cube vision test FAILED: no pose detected cube")


def main(args=None):
    rclpy.init(args=args)

    node = GroundCubeCameraSearch()

    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
