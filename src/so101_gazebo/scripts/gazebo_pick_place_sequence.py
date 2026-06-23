#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class GazeboPickPlaceSequence(Node):

    def __init__(self):
        super().__init__("gazebo_pick_place_sequence")

        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory"
        )

        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory"
        )

        self.state_pub = self.create_publisher(
            String,
            "/pick_place_state",
            10
        )

        self.arm_joints = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll"
        ]

        # Safe poses confirmed manually
        self.home = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.pick_pose = [0.0, -0.5, 0.5, -0.3, 0.0]
        self.place_pose = [0.6, -0.5, 0.5, -0.3, 0.0]
        self.after_release_pose = [-0.3, -0.5, 0.5, -0.3, 0.0]

        self.gripper_open = [1.2]
        self.gripper_close = [-0.15]

    def send_arm_goal(self, positions, duration=4):
        self.get_logger().info(f"ARM GOAL: {positions}")

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
            self.get_logger().info("ARM GOAL DONE")
            return True

        self.get_logger().error(f"Arm goal failed: {result.error_string}")
        return False

    def send_gripper_goal(self, positions, duration=2):
        self.get_logger().info(f"GRIPPER GOAL: {positions}")

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ["gripper"]

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = duration
        point.time_from_start.nanosec = 0

        goal_msg.trajectory.points.append(point)

        self.gripper_client.wait_for_server()

        future = self.gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result

        if result.error_code == 0:
            self.get_logger().info("GRIPPER GOAL DONE")
            return True

        self.get_logger().error(f"Gripper goal failed: {result.error_string}")
        return False

    def publish_state(self, state):
        msg = String()
        msg.data = state

        self.get_logger().info(f"PUBLISH STATE: {state}")

        # Publish multiple times so ROS discovery/timing does not miss it
        for _ in range(5):
            self.state_pub.publish(msg)
            time.sleep(0.2)

    def run_sequence(self):
        self.get_logger().info("AUTO PICK-PLACE SEQUENCE STARTED")

        self.send_gripper_goal(self.gripper_open)
        time.sleep(1.0)

        self.send_arm_goal(self.pick_pose, duration=4)
        time.sleep(1.0)

        self.send_gripper_goal(self.gripper_close)
        time.sleep(1.0)

        self.publish_state("GRIP_CLOSE")
        time.sleep(2.0)

        self.send_arm_goal(self.place_pose, duration=6)
        time.sleep(1.0)

        self.send_gripper_goal(self.gripper_open)
        time.sleep(1.0)

        self.publish_state("GRIP_OPEN")
        time.sleep(1.0)

        self.send_arm_goal(self.after_release_pose, duration=4)
        time.sleep(1.0)

        self.send_arm_goal(self.home, duration=5)

        self.get_logger().info("AUTO PICK-PLACE SEQUENCE COMPLETE")


def main(args=None):
    rclpy.init(args=args)

    node = GazeboPickPlaceSequence()

    try:
        node.run_sequence()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
