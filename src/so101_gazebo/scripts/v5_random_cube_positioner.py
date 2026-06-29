#!/usr/bin/env python3

import random
import rclpy
from rclpy.node import Node

from ros_gz_interfaces.srv import SetEntityPose


class V5RandomCubePositioner(Node):
    def __init__(self):
        super().__init__("v5_random_cube_positioner")

        self.declare_parameter("cube_position", "RANDOM")
        self.cube_position = self.get_parameter("cube_position").value.upper()

        self.set_pose_client = self.create_client(
            SetEntityPose,
            "/world/so101_world/set_pose"
        )

        self.positions = {
            "LEFT":   (0.320, 0.060, 0.020),
            "CENTER": (0.320, 0.100, 0.020),
            "RIGHT":  (0.320, 0.140, 0.020),
        }

        self.done = False
        self.timer = self.create_timer(1.0, self.move_cube_once)

        self.get_logger().info("V9 red target positioner started")
        self.get_logger().info("Allowed cube_position: RANDOM / LEFT / CENTER / RIGHT")
        self.get_logger().info(f"Requested cube_position: {self.cube_position}")

    def choose_position(self):
        if self.cube_position == "RANDOM":
            label = random.choice(list(self.positions.keys()))
            x, y, z = self.positions[label]
            return label, x, y, z

        if self.cube_position not in self.positions:
            self.get_logger().warn(
                f"Unknown cube_position '{self.cube_position}', using CENTER"
            )
            label = "CENTER"
            x, y, z = self.positions[label]
            return label, x, y, z

        label = self.cube_position
        x, y, z = self.positions[label]
        return label, x, y, z

    def move_cube_once(self):
        if self.done:
            return

        if not self.set_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /world/so101_world/set_pose service...")
            return

        label, x, y, z = self.choose_position()

        req = SetEntityPose.Request()
        req.entity.name = "pick_cube"
        req.entity.type = 2

        req.pose.position.x = x
        req.pose.position.y = y
        req.pose.position.z = z

        req.pose.orientation.x = 0.0
        req.pose.orientation.y = 0.0
        req.pose.orientation.z = 0.0
        req.pose.orientation.w = 1.0

        self.get_logger().info(
            f"V9 RED TARGET POSITION SELECTED: {label} x={x:.3f} y={y:.3f} z={z:.3f}"
        )

        future = self.set_pose_client.call_async(req)
        future.add_done_callback(self.done_callback)

        self.done = True

    def done_callback(self, future):
        try:
            result = future.result()
            self.get_logger().info(f"V9 RED TARGET MOVE SERVICE RESULT: {result}")
            self.get_logger().info("V9 RED TARGET POSITIONING COMPLETE")
        except Exception as e:
            self.get_logger().error(f"V9 red target positioning failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = V5RandomCubePositioner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
