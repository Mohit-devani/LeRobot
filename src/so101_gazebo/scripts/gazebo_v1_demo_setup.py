#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class GazeboV1DemoSetup(Node):

    def __init__(self):
        super().__init__("gazebo_v1_demo_setup")

        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory"
        )

        self.arm_joints = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll"
        ]

        # Camera-ready pose for wrist-camera cube detection.
        self.camera_pose = [0.0, -0.5, 0.5, -0.3, 0.0]

    def move_arm(self, positions, duration=4):
        self.get_logger().info(f"Moving arm to camera pose: {positions}")

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
            self.get_logger().info("Arm camera pose reached")
            return True

        self.get_logger().error(f"Arm goal failed: {result.error_string}")
        return False

    def run(self):
        self.get_logger().info("Gazebo V1 demo setup started")

        time.sleep(2.0)

        self.move_arm(self.camera_pose, duration=4)

        self.get_logger().info("Gazebo V1 demo setup complete")


def main(args=None):
    rclpy.init(args=args)

    node = GazeboV1DemoSetup()

    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
