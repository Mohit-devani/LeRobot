#!/usr/bin/env python3

import time
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import String, Bool

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from tf2_ros import Buffer, TransformListener


class GroundPosePickSequence(Node):

    def __init__(self):
        super().__init__("ground_pose_pick_sequence")

        self.started = False
        self.pick_thread = None

        self.cube_pose = None
        self.object_visible = False
        self.latest_camera_error = None
        self.waiting_camera_logged = False
        self.waiting_pose_logged = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/pick_cube_pose",
            self.pose_callback,
            10
        )

        self.object_sub = self.create_subscription(
            Bool,
            "/object_detected",
            self.object_callback,
            10
        )

        self.camera_error_sub = self.create_subscription(
            PointStamped,
            "/camera_cube_error",
            self.camera_error_callback,
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

        self.gripper_open = [1.2]
        self.gripper_close = [-0.15]

        self.get_logger().info("Ground pose pick sequence started")
        self.get_logger().info("Waiting for BOTH /pick_cube_pose and /object_detected=True")

    def pose_callback(self, msg):
        self.cube_pose = msg

        if not self.started:
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            self.get_logger().info(
                f"Received cube pose: x={x:.3f}, y={y:.3f}, z={z:.3f}"
            )

        self.try_start_sequence()

    def object_callback(self, msg):
        self.object_visible = bool(msg.data)

        if self.object_visible and not self.started:
            self.get_logger().info("CAMERA GATE: red cube visible in wrist camera")

        self.try_start_sequence()

    def camera_error_callback(self, msg):
        self.latest_camera_error = msg

    def try_start_sequence(self):
        if self.started:
            return

        if self.cube_pose is None:
            if not self.waiting_pose_logged:
                self.waiting_pose_logged = True
                self.get_logger().info("Waiting for /pick_cube_pose...")
            return

        if not self.object_visible:
            if not self.waiting_camera_logged:
                self.waiting_camera_logged = True
                self.get_logger().info("Waiting for camera to detect red cube...")
            return

        self.started = True

        self.get_logger().info("CAMERA + POSE GATE PASSED")
        self.get_logger().info("Starting ground pose pick-place")

        self.pick_thread = threading.Thread(
            target=self.run_pick_place,
            daemon=True
        )
        self.pick_thread.start()

    def wait_for_future(self, future, timeout_sec=180.0):
        start = time.time()

        while rclpy.ok() and not future.done():
            if timeout_sec is not None and time.time() - start > timeout_sec:
                self.get_logger().error("Future wait timed out")
                return False

            time.sleep(0.05)

        return True

    def send_arm_goal(self, positions, duration=3):
        self.get_logger().info(f"ARM GOAL: {positions}")

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.arm_joints

        point = JointTrajectoryPoint()
        point.positions = positions
        duration_sec = float(duration)
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1_000_000_000)

        goal_msg.trajectory.points.append(point)

        self.arm_client.wait_for_server()

        future = self.arm_client.send_goal_async(goal_msg)

        if not self.wait_for_future(future, timeout_sec=30.0):
            return False

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Arm goal rejected")
            return False

        result_future = goal_handle.get_result_async()

        if not self.wait_for_future(result_future, timeout_sec=180.0):
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
        duration_sec = float(duration)
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1_000_000_000)

        goal_msg.trajectory.points.append(point)

        self.gripper_client.wait_for_server()

        future = self.gripper_client.send_goal_async(goal_msg)

        if not self.wait_for_future(future, timeout_sec=30.0):
            return False

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            return False

        result_future = goal_handle.get_result_async()

        if not self.wait_for_future(result_future, timeout_sec=120.0):
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

        for _ in range(5):
            self.state_pub.publish(msg)
            time.sleep(0.2)

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def gripper_xy_close_to_cube(self, max_xy_distance=0.06):
        try:
            tf = self.tf_buffer.lookup_transform(
                "world",
                "gripper_link",
                Time()
            )
        except Exception as e:
            self.get_logger().error(f"Cannot check gripper-cube distance. TF missing: {e}")
            return False

        cube_x = self.cube_pose.pose.position.x
        cube_y = self.cube_pose.pose.position.y

        grip_x = tf.transform.translation.x
        grip_y = tf.transform.translation.y

        dx = grip_x - cube_x
        dy = grip_y - cube_y

        xy_distance = math.sqrt(dx * dx + dy * dy)

        self.get_logger().info(
            f"GRASP XY CHECK: xy_distance={xy_distance:.3f} m | "
            f"dx={dx:.3f}, dy={dy:.3f} | "
            f"limit={max_xy_distance:.3f} m"
        )

        if xy_distance <= max_xy_distance:
            self.get_logger().info("GRASP XY CHECK PASSED")
            return True

        self.get_logger().error("GRASP XY CHECK FAILED - cube will NOT attach")
        return False

    def run_camera_pan_alignment(self):
        self.get_logger().info("CAMERA PRE-ALIGNMENT STARTED")

        pan = 0.0

        for step_index in range(6):
            if self.latest_camera_error is None:
                self.get_logger().warn("CAMERA ALIGN: waiting for /camera_cube_error")
                time.sleep(0.5)
                continue

            error_x = self.latest_camera_error.point.x
            error_y = self.latest_camera_error.point.y
            area = self.latest_camera_error.point.z

            self.get_logger().info(
                f"CAMERA ALIGN STEP {step_index + 1}: "
                f"error_x={error_x:.1f}, error_y={error_y:.1f}, "
                f"area={area:.1f}, pan={pan:.3f}"
            )

            if area > 100.0 and abs(error_x) < 60.0:
                self.get_logger().info("CAMERA PRE-ALIGNMENT PASSED")
                return True

            # Sign tuned for this SO101 Gazebo setup:
            # negative error_x means cube is left in image, so pan moves negative.
            pan_step = self.clamp(0.0010 * error_x, -0.08, 0.08)
            pan = self.clamp(pan + pan_step, -0.40, 0.40)

            align_pose = [pan, -0.35, 0.70, 0.60, 0.0]

            if not self.send_arm_goal(align_pose, duration=0.25):
                self.get_logger().error("CAMERA PRE-ALIGNMENT ARM GOAL FAILED")
                return False

            time.sleep(0.8)

        self.get_logger().warn("CAMERA PRE-ALIGNMENT FINISHED BEST-EFFORT")
        return True

    def run_pick_place(self):
        self.get_logger().info("GROUND POSE PICK-PLACE STARTED")

        # First real use of camera error for motion.
        # This does a small pan alignment before the pose-driven pick.
        self.run_camera_pan_alignment()

        cube_x = self.cube_pose.pose.position.x
        cube_y = self.cube_pose.pose.position.y

        # Negative sign is required for this SO101/Gazebo joint direction.
        pan = -math.atan2(cube_y, cube_x)
        pan = self.clamp(pan, -0.8, 0.8)

        self.get_logger().info(f"Computed shoulder_pan from cube pose: {pan:.3f}")

        # Tuned from ground_grasp_pose_scan.py
        # J_FORWARD_5 gave best XY alignment:
        # xy_distance≈0.018 m, dx≈-0.017 m, dy≈-0.005 m
        pre_grasp_pose = [pan, -0.30, 0.65, 0.55, 0.0]
        grasp_pose     = [pan, -0.12, 0.42, 0.35, 0.0]
        lift_pose      = [pan, -0.25, 0.60, 0.45, 0.0]
        place_pose     = [0.75, -0.45, 0.45, -0.35, 0.0]
        retreat_pose   = [0.0, 0.0, 0.0, 0.0, 0.0]

        # 1. Open gripper.
        if not self.send_gripper_goal(self.gripper_open, duration=0.3):
            return

        time.sleep(0.5)

        # 2. Move near cube.
        if not self.send_arm_goal(pre_grasp_pose, duration=0.8):
            return

        time.sleep(0.5)

        # 3. Move to tuned grasp pose.
        if not self.send_arm_goal(grasp_pose, duration=0.8):
            return

        time.sleep(0.5)

        # 4. Close gripper.
        if not self.send_gripper_goal(self.gripper_close, duration=0.3):
            return

        time.sleep(0.5)

        # 5. Safety gate: only attach if gripper is close in X/Y.
        if not self.gripper_xy_close_to_cube(max_xy_distance=0.06):
            self.get_logger().error("ABORTING PICK: gripper is not close enough to cube")
            return

        # 6. Attach cube virtually after gripper closes.
        self.publish_state("GRIP_CLOSE")
        time.sleep(1.0)

        # 7. Lift.
        if not self.send_arm_goal(lift_pose, duration=0.8):
            return

        time.sleep(0.5)

        # 8. Move to place.
        if not self.send_arm_goal(place_pose, duration=1.0):
            return

        time.sleep(0.5)

        # 9. Release.
        self.publish_state("GRIP_OPEN")
        time.sleep(0.5)

        # 10. Retreat.
        if not self.send_arm_goal(retreat_pose, duration=1.0):
            return

        time.sleep(0.5)

        # 11. Open gripper safely after retreat.
        if not self.send_gripper_goal(self.gripper_open, duration=0.3):
            return

        self.get_logger().info("GROUND POSE PICK-PLACE COMPLETE")


def main(args=None):
    rclpy.init(args=args)

    node = GroundPosePickSequence()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
