#!/usr/bin/env python3

import random
import rclpy

from rclpy.node import Node
from rclpy.time import Time

from visualization_msgs.msg import Marker
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped

from tf2_ros import Buffer
from tf2_ros import TransformListener
from tf2_ros import TransformBroadcaster


class CubeMarker(Node):

    def __init__(self):
        super().__init__('cube_marker')

        self.marker_publisher = self.create_publisher(
            Marker,
            '/visualization_marker',
            10
        )

        self.random_subscriber = self.create_subscription(
            String,
            '/randomize_cube',
            self.random_callback,
            10
        )

        self.state_subscriber = self.create_subscription(
            String,
            '/pick_place_state',
            self.state_callback,
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.attach_frame = 'gripper_frame_link'

        self.cube_x = 0.25
        self.cube_y = 0.00
        self.cube_z = 0.05

        self.attached_to_gripper = False
        self.placed = False

        self.timer = self.create_timer(
            0.1,
            self.update
        )

        self.get_logger().info('Random cube marker started')
        self.get_logger().info('Publishing cube_link TF')

    def random_callback(self, msg):
        self.cube_x = random.uniform(0.22, 0.32)
        self.cube_y = random.uniform(-0.18, 0.18)
        self.cube_z = 0.05

        self.attached_to_gripper = False
        self.placed = False

        self.get_logger().info(
            f'New cube position: x={self.cube_x:.2f}, '
            f'y={self.cube_y:.2f}, z={self.cube_z:.2f}'
        )

    def state_callback(self, msg):
        state = msg.data.strip()

        if state == 'START':
            self.attached_to_gripper = False
            self.placed = False

        elif state == 'GRIP_CLOSE':
            self.attached_to_gripper = True
            self.placed = False
            self.get_logger().info('Cube attached to gripper')

        elif state == 'GRIP_OPEN':
            self.save_release_position()
            self.attached_to_gripper = False
            self.placed = True

            self.get_logger().info(
                f'Cube released at x={self.cube_x:.2f}, '
                f'y={self.cube_y:.2f}, z={self.cube_z:.2f}'
            )

    def save_release_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                self.attach_frame,
                Time()
            )

            self.cube_x = transform.transform.translation.x
            self.cube_y = transform.transform.translation.y
            self.cube_z = transform.transform.translation.z

            if self.cube_z < 0.05:
                self.cube_z = 0.05

        except Exception as e:
            self.get_logger().warn(
                f'Could not read gripper TF. Keeping old cube position. Error: {e}'
            )

    def get_cube_parent_and_position(self):
        if self.attached_to_gripper:
            return self.attach_frame, 0.0, 0.0, 0.0

        return 'base_link', self.cube_x, self.cube_y, self.cube_z

    def publish_cube_tf(self):
        parent_frame, x, y, z = self.get_cube_parent_and_position()

        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = 'cube_link'

        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(transform)

    def publish_cube_marker(self):
        marker = Marker()

        marker.header.frame_id = 'cube_link'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'objects'
        marker.id = 1

        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.06
        marker.scale.y = 0.06
        marker.scale.z = 0.06

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_publisher.publish(marker)

    def update(self):
        self.publish_cube_tf()
        self.publish_cube_marker()


def main(args=None):
    rclpy.init(args=args)

    node = CubeMarker()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
