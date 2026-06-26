#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped


class DynamicGazeboCubePosePublisher(Node):
    def __init__(self):
        super().__init__("dynamic_gazebo_cube_pose_publisher")

        self.publisher = self.create_publisher(
            PoseStamped,
            "/pick_cube_pose",
            10
        )

        # Ground cube pose used by so101_ground_cube.sdf
        self.cube_x = 0.320
        self.cube_y = 0.100
        self.cube_z = 0.020

        self.timer = self.create_timer(0.10, self.publish_cube_pose)

        self.get_logger().info(
            f"Publishing /pick_cube_pose: "
            f"x={self.cube_x:.3f}, y={self.cube_y:.3f}, z={self.cube_z:.3f}"
        )

    def publish_cube_pose(self):
        msg = PoseStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        msg.pose.position.x = self.cube_x
        msg.pose.position.y = self.cube_y
        msg.pose.position.z = self.cube_z

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = DynamicGazeboCubePosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
