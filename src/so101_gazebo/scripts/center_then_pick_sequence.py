#!/usr/bin/env python3

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Point
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class CenterThenPickSequence(Node):

    def __init__(self):
        super().__init__("center_then_pick_sequence")

        self.centered = False
        self.started_pick = False
        self.pick_thread = None

        self.object_detected = False
        self.latest_error = None

        # Safety gate tolerance after visual centering.
        self.grasp_error_tolerance_x = 60.0
        self.grasp_error_tolerance_y = 60.0

        self.done_sub = self.create_subscription(
            Bool,
            "/visual_servo_sequence_done",
            self.center_done_callback,
            10
        )

        self.object_sub = self.create_subscription(
            Bool,
            "/object_detected",
            self.object_detected_callback,
            10
        )

        self.error_sub = self.create_subscription(
            Point,
            "/visual_servo_error",
            self.visual_error_callback,
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
        self.home = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.gripper_open = [1.2]
        self.gripper_close = [-0.15]

        self.get_logger().info("Center-then-pick sequence started")
        self.get_logger().info("Waiting for /visual_servo_sequence_done == true")

    def object_detected_callback(self, msg):
        self.object_detected = msg.data

    def visual_error_callback(self, msg):
        self.latest_error = msg

    def center_done_callback(self, msg):
        if msg.data and not self.started_pick:
            self.centered = True
            self.started_pick = True

            self.get_logger().info("CENTERING DONE received. Starting pick-place.")

            # Important:
            # Run long pick-place sequence outside subscriber callback.
            self.pick_thread = threading.Thread(
                target=self.run_pick_place,
                daemon=True
            )
            self.pick_thread.start()

    def wait_for_future(self, future, timeout_sec=20.0):
        start_time = time.time()

        while rclpy.ok() and not future.done():
            if timeout_sec is not None:
                if time.time() - start_time > timeout_sec:
                    self.get_logger().error("Future wait timed out")
                    return False

            time.sleep(0.02)

        return True

    def is_grasp_safe(self):
        if not self.object_detected:
            self.get_logger().warn("GRASP BLOCKED: object_detected is false")
            return False

        if self.latest_error is None:
            self.get_logger().warn("GRASP BLOCKED: no visual_servo_error received")
            return False

        error_x = abs(self.latest_error.x)
        error_y = abs(self.latest_error.y)

        self.get_logger().info(
            f"GRASP CHECK: error_x={error_x:.1f}, error_y={error_y:.1f}"
        )

        if error_x > self.grasp_error_tolerance_x:
            self.get_logger().warn("GRASP BLOCKED: x error too large")
            return False

        if error_y > self.grasp_error_tolerance_y:
            self.get_logger().warn("GRASP BLOCKED: y error too large")
            return False

        self.get_logger().info("GRASP CHECK PASSED")
        return True

    def send_arm_goal(self, positions, duration=3):
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

        if not self.wait_for_future(future, timeout_sec=10.0):
            return False

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            return False

        result_future = goal_handle.get_result_async()

        if not self.wait_for_future(result_future, timeout_sec=duration + 10.0):
            return False

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

        # Let final visual error settle.
        time.sleep(0.3)

        # Safety gate before grasp.
        if not self.is_grasp_safe():
            self.get_logger().error("CAMERA-CENTERED PICK-PLACE ABORTED: unsafe grasp")
            return

        # Open gripper before grasp.
        if not self.send_gripper_goal(self.gripper_open, duration=1):
            return
        time.sleep(0.2)

        # Close gripper.
        if not self.send_gripper_goal(self.gripper_close, duration=1):
            return
        time.sleep(0.2)

        # Attach cube virtually.
        self.publish_state("GRIP_CLOSE")
        time.sleep(0.8)

        # Move attached cube to place side.
        if not self.send_arm_goal(self.place_pose, duration=3):
            return
        time.sleep(0.3)

        # Release virtual attach.
        self.publish_state("GRIP_OPEN")
        time.sleep(0.3)

        # Move home while gripper is still closed.
        if not self.send_arm_goal(self.home, duration=3):
            return
        time.sleep(0.3)

        # Open gripper safely at home.
        if not self.send_gripper_goal(self.gripper_open, duration=1):
            return
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
