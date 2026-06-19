#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO


class CameraViewer(Node):

    def __init__(self):
        super().__init__('camera_viewer')

        self.get_logger().info('Starting Camera Viewer...')

        self.bridge = CvBridge()

        self.get_logger().info('Loading YOLO model...')
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info('YOLO model loaded successfully.')

        self.frame_count = 0

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info('Subscribed to /image_raw')

    def image_callback(self, msg):
        try:
            self.frame_count += 1

            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f'Received {self.frame_count} frames'
                )

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            results = self.model(frame, verbose=False)

            annotated_frame = results[0].plot()

            cv2.imshow('YOLO Camera', annotated_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(
                f'Callback error: {str(e)}'
            )


def main(args=None):
    rclpy.init(args=args)

    node = CameraViewer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')

    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
