#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import String
from geometry_msgs.msg import Pose

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

from ros_gz_interfaces.srv import SetEntityPose


class GazeboCubeAttach(Node):

    def __init__(self):
        super().__init__("gazebo_cube_attach")

        self.attached = False
        self.pending_future = None
        self.last_log_time = self.get_clock().now()

        # Calibration offsets.
        # These move the cube from gripper_link to the visual jaw area.
        self.grasp_offset_x = 0.025
        self.grasp_offset_y = 0.000
        self.grasp_offset_z = 0.145

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.state_sub = self.create_subscription(
            String,
            "/pick_place_state",
            self.state_callback,
            10
        )

        self.client = self.create_client(
            SetEntityPose,
            "/world/so101_world/set_pose"
        )

        self.timer = self.create_timer(0.10, self.timer_callback)

        self.get_logger().info("Gazebo cube attach node started")
        self.get_logger().info("Listening to /pick_place_state")
        self.get_logger().info("Using /world/so101_world/set_pose")
        self.get_logger().info(
            f"Grasp offset: x={self.grasp_offset_x}, "
            f"y={self.grasp_offset_y}, z={self.grasp_offset_z}"
        )

    def state_callback(self, msg):
        self.get_logger().info(f"Received state: {msg.data}")

        if msg.data == "GRIP_CLOSE":
            self.attached = True
            self.get_logger().info("ATTACH MODE ON")

        elif msg.data == "GRIP_OPEN":
            self.attached = False
            self.get_logger().info("ATTACH MODE OFF")

    def lookup_gripper_pose(self):
        candidates = [
            ("world", "gripper_link"),
            ("base_link", "gripper_link"),
        ]

        last_error = None

        for parent, child in candidates:
            try:
                tf = self.tf_buffer.lookup_transform(
                    parent,
                    child,
                    Time()
                )
                return parent, child, tf
            except TransformException as ex:
                last_error = ex

        raise last_error

    def timer_callback(self):
        if not self.attached:
            return

        if not self.client.service_is_ready():
            self.get_logger().warn("set_pose service not ready")
            return

        try:
            parent, child, tf = self.lookup_gripper_pose()
        except TransformException as ex:
            self.get_logger().warn(f"TF not available: {ex}")
            return

        req = SetEntityPose.Request()

        req.entity.name = "pick_cube"
        req.entity.type = 2  # MODEL

        pose = Pose()

        pose.position.x = tf.transform.translation.x + self.grasp_offset_x
        pose.position.y = tf.transform.translation.y + self.grasp_offset_y
        pose.position.z = tf.transform.translation.z + self.grasp_offset_z

        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        req.pose = pose

        self.pending_future = self.client.call_async(req)

        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds > 500_000_000:
            self.get_logger().info(
                f"CALL set_pose: TF {parent}->{child}, "
                f"cube x={pose.position.x:.3f}, "
                f"y={pose.position.y:.3f}, "
                f"z={pose.position.z:.3f}"
            )
            self.last_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = GazeboCubeAttach()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
