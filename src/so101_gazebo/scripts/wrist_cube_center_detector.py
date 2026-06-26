#!/usr/bin/env python3

import time

import cv2
import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped


class WristCubeCenterDetector(Node):

    def __init__(self):
        super().__init__("wrist_cube_center_detector")

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            "/wrist_camera/image",
            self.image_callback,
            10
        )

        self.detected_pub = self.create_publisher(
            Bool,
            "/object_detected",
            10
        )

        self.center_pub = self.create_publisher(
            PointStamped,
            "/camera_cube_center",
            10
        )

        self.error_pub = self.create_publisher(
            PointStamped,
            "/camera_cube_error",
            10
        )

        self.target_x = 320.0
        self.target_y = 370.0

        self.min_area = 100.0
        self.last_log_time = 0.0

        self.get_logger().info("Wrist cube center detector started")
        self.get_logger().info("Publishing /object_detected, /camera_cube_center, /camera_cube_error")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        lower_red_1 = (0, 80, 50)
        upper_red_1 = (10, 255, 255)

        lower_red_2 = (170, 80, 50)
        upper_red_2 = (180, 255, 255)

        mask_1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        mask_2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
        mask = mask_1 | mask_2

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        detected_msg = Bool()
        detected_msg.data = False

        if not contours:
            self.detected_pub.publish(detected_msg)
            cv2.putText(
                bgr,
                "NO OBJECT",
                (30, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.3,
                (0, 0, 255),
                3
            )
            cv2.imshow("Wrist Cube Center Detector", bgr)
            cv2.waitKey(1)
            return

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area < self.min_area:
            self.detected_pub.publish(detected_msg)
            cv2.putText(
                bgr,
                "NO OBJECT",
                (30, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.3,
                (0, 0, 255),
                3
            )
            cv2.imshow("Wrist Cube Center Detector", bgr)
            cv2.waitKey(1)
            return

        x, y, w, h = cv2.boundingRect(largest)
        cx = float(x + w / 2.0)
        cy = float(y + h / 2.0)

        detected_msg.data = True
        self.detected_pub.publish(detected_msg)

        center_msg = PointStamped()
        center_msg.header.stamp = self.get_clock().now().to_msg()
        center_msg.header.frame_id = "wrist_camera_image"
        center_msg.point.x = cx
        center_msg.point.y = cy
        center_msg.point.z = area
        self.center_pub.publish(center_msg)

        error_msg = PointStamped()
        error_msg.header.stamp = self.get_clock().now().to_msg()
        error_msg.header.frame_id = "wrist_camera_image"
        error_msg.point.x = cx - self.target_x
        error_msg.point.y = cy - self.target_y
        error_msg.point.z = area
        self.error_pub.publish(error_msg)

        now = time.monotonic()
        if now - self.last_log_time > 2.0:
            self.last_log_time = now
            self.get_logger().info(
                f"CAMERA CUBE CENTER: cx={cx:.1f}, cy={cy:.1f}, "
                f"error_x={error_msg.point.x:.1f}, error_y={error_msg.point.y:.1f}, "
                f"area={area:.1f}"
            )

        cv2.rectangle(bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(bgr, (int(cx), int(cy)), 5, (255, 0, 0), -1)

        cv2.putText(
            bgr,
            f"RED CUBE cx={int(cx)} cy={int(cy)}",
            (30, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2
        )

        cv2.putText(
            bgr,
            f"err_x={error_msg.point.x:.0f} err_y={error_msg.point.y:.0f}",
            (30, 105),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 0),
            2
        )

        cv2.imshow("Wrist Cube Center Detector", bgr)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    node = WristCubeCenterDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
