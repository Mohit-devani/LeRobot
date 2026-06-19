#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster

class CameraFrame(Node):

    def __init__(self):

        super().__init__('camera_frame')

        self.br = TransformBroadcaster(self)

        self.timer = self.create_timer(
            0.1,
            self.publish_tf
        )

    def publish_tf(self):

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()

        t.header.frame_id = 'base_link'

        t.child_frame_id = 'camera_link'

        t.transform.translation.x = 0.30
        t.transform.translation.y = 0.00
        t.transform.translation.z = 0.40

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.br.sendTransform(t)

def main(args=None):

    rclpy.init(args=args)

    node = CameraFrame()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
