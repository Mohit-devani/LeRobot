#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import Point


class GroundCubeAlignmentCheck(Node):

    def __init__(self):
        super().__init__("ground_cube_alignment_check")

        self.object_detected = False
        self.center = None

        self.target_x = 320.0
        self.target_y = 370.0

        self.tolerance_x = 70.0
        self.tolerance_y = 90.0

        self.min_area = 100.0

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

    def object_callback(self, msg):
        self.object_detected = msg.data

    def center_callback(self, msg):
        self.center = msg

    def is_aligned(self):
        if not self.object_detected:
            return False

        if self.center is None:
            return False

        error_x = abs(self.center.x - self.target_x)
        error_y = abs(self.center.y - self.target_y)

        area = self.center.z

        self.get_logger().info(
            f"GROUND ALIGN CHECK: cx={self.center.x:.1f}, cy={self.center.y:.1f}, "
            f"area={area:.1f}, error_x={error_x:.1f}, error_y={error_y:.1f}"
        )

        if area < self.min_area:
            self.get_logger().warn("GROUND ALIGN BLOCKED: cube area too small")
            return False

        if error_x > self.tolerance_x:
            self.get_logger().warn("GROUND ALIGN BLOCKED: x too far")
            return False

        if error_y > self.tolerance_y:
            self.get_logger().warn("GROUND ALIGN BLOCKED: y too far")
            return False

        return True

    def run(self):
        self.get_logger().info("Ground cube alignment check started")
        self.get_logger().info("Target window: cx≈320, cy≈360")

        start = time.time()

        while rclpy.ok() and time.time() - start < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.is_aligned():
                self.get_logger().info("GROUND CUBE ALIGNMENT PASSED")
                return

        self.get_logger().error("GROUND CUBE ALIGNMENT FAILED")


def main(args=None):
    rclpy.init(args=args)

    node = GroundCubeAlignmentCheck()

    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
