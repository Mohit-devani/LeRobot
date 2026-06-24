#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from ros_gz_interfaces.srv import SetEntityPose


class WristVisionTestSetup(Node):

    def __init__(self):
        super().__init__("wrist_vision_test_setup")

        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory"
        )

        self.set_pose_client = self.create_client(
            SetEntityPose,
            "/world/so101_world/set_pose"
        )

        self.arm_joints = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll"
        ]

        self.camera_pose = [0.0, -0.5, 0.5, -0.3, 0.0]

        # This is the pose you manually found working.
        self.visible_cube_x = 0.28
        self.visible_cube_y = 0.03
        self.visible_cube_z = 0.350

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

    def move_cube_to_camera_view(self):
        self.get_logger().info("Waiting for /world/so101_world/set_pose service")

        while not self.set_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Still waiting for set_pose service...")

        req = SetEntityPose.Request()

        req.entity.name = "pick_cube"
        req.entity.type = 2

        req.pose.position.x = self.visible_cube_x
        req.pose.position.y = self.visible_cube_y
        req.pose.position.z = self.visible_cube_z

        req.pose.orientation.x = 0.0
        req.pose.orientation.y = 0.0
        req.pose.orientation.z = 0.0
        req.pose.orientation.w = 1.0

        self.get_logger().info(
            f"Moving pick_cube to x={self.visible_cube_x}, y={self.visible_cube_y}, z={self.visible_cube_z}"
        )

        future = self.set_pose_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()

        if response is not None and response.success:
            self.get_logger().info("Cube moved to camera-visible pose")
            return True

        self.get_logger().error("Failed to move cube")
        return False

    def run(self):
        self.get_logger().info("Wrist vision test setup started")

        time.sleep(2.0)

        self.move_arm(self.camera_pose, duration=4)

        time.sleep(1.0)

        self.move_cube_to_camera_view()

        self.get_logger().info("Wrist vision test setup complete")


def main(args=None):
    rclpy.init(args=args)

    node = WristVisionTestSetup()

    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
