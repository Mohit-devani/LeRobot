#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped, PoseStamped


class V4CameraCubePosePublisher(Node):
    def __init__(self):
        super().__init__("v4_camera_cube_pose_publisher")

        self.pose_pub = self.create_publisher(
            PoseStamped,
            "/pick_cube_pose",
            10
        )

        self.object_sub = self.create_subscription(
            Bool,
            "/object_detected",
            self.object_callback,
            10
        )

        self.error_sub = self.create_subscription(
            PointStamped,
            "/camera_cube_error",
            self.error_callback,
            10
        )

        # Safe seed pose from the known Gazebo cube spawn.
        # This lets MoveIt reach the camera-view pre-grasp pose first.
        self.seed_x = 0.320
        self.seed_y = 0.100
        self.seed_z = 0.020

        self.cube_x = self.seed_x
        self.cube_y = self.seed_y
        self.cube_z = self.seed_z

        self.object_detected = False
        self.error_x = 0.0
        self.error_y = 0.0
        self.area = 0.0

        # Small conservative image-to-world correction.
        # Negative error_x means cube is left in image, so increase cube_y.
        self.y_gain = 0.00025

        self.min_y = 0.050
        self.max_y = 0.150

        self.min_area = 800.0

        self.last_log_time = 0.0

        self.timer = self.create_timer(0.10, self.publish_pose)

        self.get_logger().info("V4 camera-derived cube pose publisher started")
        self.get_logger().info("Publishing /pick_cube_pose from camera error when available")
        self.get_logger().info(
            f"Seed pose: x={self.seed_x:.3f}, y={self.seed_y:.3f}, z={self.seed_z:.3f}"
        )

    def object_callback(self, msg):
        self.object_detected = msg.data

    def error_callback(self, msg):
        self.error_x = msg.point.x
        self.error_y = msg.point.y
        self.area = msg.point.z

        if self.object_detected and self.area > self.min_area:
            derived_y = self.seed_y + (-self.error_x * self.y_gain)

            if derived_y < self.min_y:
                derived_y = self.min_y

            if derived_y > self.max_y:
                derived_y = self.max_y

            self.cube_x = self.seed_x
            self.cube_y = derived_y
            self.cube_z = self.seed_z

    def publish_pose(self):
        msg = PoseStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        msg.pose.position.x = float(self.cube_x)
        msg.pose.position.y = float(self.cube_y)
        msg.pose.position.z = float(self.cube_z)

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.pose_pub.publish(msg)

        now = time.monotonic()
        if now - self.last_log_time > 2.0:
            self.last_log_time = now

            if self.object_detected and self.area > self.min_area:
                self.get_logger().info(
                    f"V4 CAMERA-DERIVED /pick_cube_pose: "
                    f"x={self.cube_x:.3f}, y={self.cube_y:.3f}, z={self.cube_z:.3f}, "
                    f"error_x={self.error_x:.1f}, error_y={self.error_y:.1f}, area={self.area:.1f}"
                )
            else:
                self.get_logger().info(
                    f"V4 SEED /pick_cube_pose: "
                    f"x={self.cube_x:.3f}, y={self.cube_y:.3f}, z={self.cube_z:.3f}"
                )


def main(args=None):
    rclpy.init(args=args)
    node = V4CameraCubePosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
