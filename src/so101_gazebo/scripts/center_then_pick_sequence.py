#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Bool, String
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class CenterThenPickSequence(Node):

    def __init__(self):
        super().__init__("center_then_pick_sequence")

        self.centered = False
        self.started_pick = False

        self.done_sub = self.create_subscription(
            Bool,
            "/visual_servo_sequence_done",
            self.center_done_callback,
            10
        )

        self.state_pub = self.create_publisher(
            String,
            "/pick_place_state",
            10
        )

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

        self.arm_joints = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll"
        ]

        self.place_pose = [0.85, -0.45, 0.45, -0.45, 0.0]
        self.after_release_pose = [-0.85, -0.45, 0.45, -0.45, 0.0]
        self.home = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.gripper_open = [1.2]
        self.gripper_close = [-0.15]

        self.get_logger().info("Center-then-pick sequence started")
        self.get_logger().info("Waiting for /visual_servo_sequence_done == true")

    def center_done_callback(self, msg):
        if msg.data and not self.started_pick:
            self.centered = True
            self.started_pick = True
            self.get_logger().info("CENTERING DONE received. Starting pick-place.")
            self.run_pick_place()

    def send_arm_goal(self, positions, duration=1):
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

    def send_gripper_goal(self, positions, duration=1):
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

        for _ in range(3):
            self.state_pub.publish(msg)
            time.sleep(0.1)

    def run_pick_place(self):
        self.get_logger().info("CAMERA-CENTERED PICK-PLACE STARTED")

        # Give visual servo and TF a moment to settle.
        time.sleep(0.3)

        # Open gripper before grasp.
        self.send_gripper_goal(self.gripper_open, duration=1)
        time.sleep(0.2)

        # Close gripper.
        self.send_gripper_goal(self.gripper_close, duration=1)
        time.sleep(0.2)

        # Attach cube to gripper virtually.
        self.publish_state("GRIP_CLOSE")
        time.sleep(0.8)

        # Move attached cube to place side.
        self.send_arm_goal(self.place_pose, duration=3)
        time.sleep(0.3)

        # IMPORTANT:
        # Release virtual attach BEFORE opening the gripper.
        # If we open while the cube is still attached near the fingers,
        # the gripper can collide with the cube and get stuck.
        self.publish_state("GRIP_OPEN")
        time.sleep(0.3)

        # Move home while gripper is still closed.
        # This avoids opening the fingers near the released cube.
        self.send_arm_goal(self.home, duration=3)
        time.sleep(0.3)

        # Now open gripper safely at home, far from the cube.
        self.send_gripper_goal(self.gripper_open, duration=1)
        time.sleep(0.2)

        self.get_logger().info("CAMERA-CENTERED PICK-PLACE COMPLETE")


def main(args=None):
    rclpy.init(args=args)

    node = CenterThenPickSequence()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
