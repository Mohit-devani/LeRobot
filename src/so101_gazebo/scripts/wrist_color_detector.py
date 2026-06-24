#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge


class WristColorDetector(Node):

    def __init__(self):
        super().__init__("wrist_color_detector")

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            "/wrist_camera/image",
            self.image_callback,
            10
        )

        self.center_pub = self.create_publisher(
            Point,
            "/detected_object_center",
            10
        )

        self.detected_pub = self.create_publisher(
            Bool,
            "/object_detected",
            10
        )

        self.get_logger().info("Wrist color detector started")
        self.get_logger().info("Subscribing to /wrist_camera/image")
        self.get_logger().info("Publishing /detected_object_center and /object_detected")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

        # Red object detection.
        # Red wraps around HSV, so we use two ranges.
        lower_red_1 = np.array([0, 80, 50])
        upper_red_1 = np.array([10, 255, 255])

        lower_red_2 = np.array([170, 80, 50])
        upper_red_2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)

        mask = mask1 | mask2

        # Clean noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        detected_msg = Bool()
        center_msg = Point()

        if len(contours) == 0:
            detected_msg.data = False
            self.detected_pub.publish(detected_msg)

            cv2.putText(
                frame_bgr,
                "NO OBJECT",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                2
            )

        else:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > 100:
                x, y, w, h = cv2.boundingRect(largest)

                cx = x + w // 2
                cy = y + h // 2

                detected_msg.data = True

                center_msg.x = float(cx)
                center_msg.y = float(cy)
                center_msg.z = float(area)

                self.detected_pub.publish(detected_msg)
                self.center_pub.publish(center_msg)

                cv2.rectangle(
                    frame_bgr,
                    (x, y),
                    (x + w, y + h),
                    (0, 255, 0),
                    2
                )

                cv2.circle(
                    frame_bgr,
                    (cx, cy),
                    5,
                    (255, 0, 0),
                    -1
                )

                cv2.putText(
                    frame_bgr,
                    f"RED CUBE cx={cx} cy={cy}",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2
                )

                self.get_logger().info(
                    f"Detected object center: x={cx}, y={cy}, area={area:.1f}"
                )

            else:
                detected_msg.data = False
                self.detected_pub.publish(detected_msg)

        cv2.imshow("Wrist Camera Color Detector", frame_bgr)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    node = WristColorDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
