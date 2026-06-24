#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from std_msgs.msg import Bool


class VisualServoError(Node):

    def __init__(self):
        super().__init__("visual_servo_error")

        self.image_center_x = 320.0
        self.image_center_y = 240.0
        self.center_tolerance = 25.0

        self.object_detected = False

        self.detected_sub = self.create_subscription(
            Bool,
            "/object_detected",
            self.detected_callback,
            10
        )

        self.center_sub = self.create_subscription(
            Point,
            "/detected_object_center",
            self.center_callback,
            10
        )

        self.error_pub = self.create_publisher(
            Point,
            "/visual_servo_error",
            10
        )

        self.centered_pub = self.create_publisher(
            Bool,
            "/visual_servo_centered",
            10
        )

        self.get_logger().info("Visual servo error node started")
        self.get_logger().info("Subscribing to /object_detected")
        self.get_logger().info("Subscribing to /detected_object_center")
        self.get_logger().info("Publishing /visual_servo_error")
        self.get_logger().info("Publishing /visual_servo_centered")

    def detected_callback(self, msg):
        self.object_detected = msg.data

        if not self.object_detected:
            centered_msg = Bool()
            centered_msg.data = False
            self.centered_pub.publish(centered_msg)

    def center_callback(self, msg):
        if not self.object_detected:
            return

        detected_x = msg.x
        detected_y = msg.y
        area = msg.z

        error_x = detected_x - self.image_center_x
        error_y = detected_y - self.image_center_y

        error_msg = Point()
        error_msg.x = float(error_x)
        error_msg.y = float(error_y)
        error_msg.z = float(area)

        self.error_pub.publish(error_msg)

        centered = (
            abs(error_x) < self.center_tolerance and
            abs(error_y) < self.center_tolerance
        )

        centered_msg = Bool()
        centered_msg.data = centered
        self.centered_pub.publish(centered_msg)

        self.get_logger().info(
            f"visual error: x={error_x:.1f}, y={error_y:.1f}, area={area:.1f}, centered={centered}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = VisualServoError()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
