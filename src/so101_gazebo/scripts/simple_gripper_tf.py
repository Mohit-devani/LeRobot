#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class SimpleGripperTF(Node):

    def __init__(self):
        super().__init__("simple_gripper_tf")

        self.br = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            10
        )

        self.get_logger().info("Simple gripper TF broadcaster started")
        self.get_logger().info("Reading /joint_states")
        self.get_logger().info("Publishing base_link -> gripper_link")

    def get_joint(self, msg, joint_name, default=0.0):
        if joint_name not in msg.name:
            return default

        index = msg.name.index(joint_name)

        if index >= len(msg.position):
            return default

        return msg.position[index]

    def joint_callback(self, msg):
        # Gazebo visual shoulder_pan direction is opposite to our simple FK convention
        shoulder_pan = -self.get_joint(msg, "shoulder_pan")
        shoulder_lift = self.get_joint(msg, "shoulder_lift")
        elbow_flex = self.get_joint(msg, "elbow_flex")
        wrist_flex = self.get_joint(msg, "wrist_flex")

        # Approximate SO101 geometry.
        # This is not final kinematics.
        # This is only to give attach node a live gripper position.
        base_height = 0.09
        l1 = 0.13
        l2 = 0.13
        l3 = 0.08

        a1 = shoulder_lift
        a2 = shoulder_lift + elbow_flex
        a3 = shoulder_lift + elbow_flex + wrist_flex

        reach = (
            l1 * math.cos(a1) +
            l2 * math.cos(a2) +
            l3 * math.cos(a3)
        )

        z = (
            base_height +
            l1 * math.sin(a1) +
            l2 * math.sin(a2) +
            l3 * math.sin(a3)
        )

        x = reach * math.cos(shoulder_pan)
        y = reach * math.sin(shoulder_pan)

        t = TransformStamped()

        if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0:
            t.header.stamp = msg.header.stamp
        else:
            t.header.stamp = self.get_clock().now().to_msg()

        t.header.frame_id = "base_link"
        t.child_frame_id = "gripper_link"

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = max(z, 0.04)

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    node = SimpleGripperTF()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
