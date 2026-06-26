#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import String
from geometry_msgs.msg import Pose
from ros_gz_interfaces.srv import SetEntityPose

import tf2_ros


class GazeboCubeDynamicAttach(Node):
    def __init__(self):
        super().__init__("gazebo_cube_dynamic_attach")

        self.world_frame = "world"
        self.gripper_frame = "gripper_link"

        self.cube_name = "pick_cube"
        self.set_pose_service = "/world/so101_world/set_pose"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.client = self.create_client(SetEntityPose, self.set_pose_service)

        self.state_sub = self.create_subscription(
            String,
            "/pick_place_state",
            self.state_callback,
            10
        )

        # Ground cube center height:
        # cube size is 0.04 m, so center on ground = 0.02 m
        self.ground_z = 0.020

        # Visible carry height after grasp
        self.carry_z = 0.120

        # Delay before lifting after GRIP_CLOSE
        self.lift_delay_sec = 1.0
        self.lift_ramp_sec = 2.0

        self.attach_mode = False
        self.attach_start_time = None

        # Cube starts here in ground cube world
        self.cube_x = 0.320
        self.cube_y = 0.100
        self.cube_z = self.ground_z

        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_z = 0.0

        self.lift_started_logged = False
        self.lift_complete_logged = False

        self.timer = self.create_timer(0.15, self.update_attached_cube)

        self.get_logger().info("Gazebo dynamic cube attach started")
        self.get_logger().info("Mode: ground safe attach + lift after close + release to ground")

    def state_callback(self, msg):
        state = msg.data.strip()

        if state == "GRIP_CLOSE":
            # IMPORTANT:
            # Only lock once. Do not reset timer every repeated GRIP_CLOSE.
            if not self.attach_mode:
                self.lock_attach_offset()
                self.attach_mode = True
                self.attach_start_time = time.monotonic()
                self.lift_started_logged = False
                self.lift_complete_logged = False
                self.get_logger().info("CUBE ATTACHED")

        elif state == "GRIP_OPEN":
            if self.attach_mode:
                self.release_to_ground()
                self.attach_mode = False
                self.attach_start_time = None

    def get_gripper_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.gripper_frame,
                Time()
            )
            return (
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z
            )
        except Exception as e:
            self.get_logger().warn(f"Could not get TF world->{self.gripper_frame}: {e}")
            return None

    def lock_attach_offset(self):
        gripper_pose = self.get_gripper_pose()
        if gripper_pose is None:
            self.get_logger().error("Could not lock attach offset")
            return

        gx, gy, gz = gripper_pose

        self.offset_x = self.cube_x - gx
        self.offset_y = self.cube_y - gy
        self.offset_z = self.cube_z - gz

        self.get_logger().info(
            f"DYNAMIC ATTACH OFFSET LOCKED: "
            f"offset_x={self.offset_x:.3f}, "
            f"offset_y={self.offset_y:.3f}, "
            f"offset_z={self.offset_z:.3f}"
        )

    def update_attached_cube(self):
        if not self.attach_mode:
            return

        gripper_pose = self.get_gripper_pose()
        if gripper_pose is None:
            return

        gx, gy, gz = gripper_pose

        raw_x = gx + self.offset_x
        raw_y = gy + self.offset_y
        raw_z = gz + self.offset_z

        elapsed = 0.0
        if self.attach_start_time is not None:
            elapsed = time.monotonic() - self.attach_start_time

        # First 1 second: stay on ground.
        # Then ramp up to carry_z.
        if elapsed < self.lift_delay_sec:
            target_z = self.ground_z
        else:
            ramp = min(1.0, (elapsed - self.lift_delay_sec) / self.lift_ramp_sec)
            target_z = self.ground_z + ramp * (self.carry_z - self.ground_z)

        cube_x = raw_x
        cube_y = raw_y
        cube_z = max(raw_z, target_z)

        self.cube_x = cube_x
        self.cube_y = cube_y
        self.cube_z = cube_z

        if elapsed >= self.lift_delay_sec and not self.lift_started_logged:
            self.lift_started_logged = True
            self.get_logger().info("CUBE LIFT STARTED")

        if target_z >= self.carry_z - 0.001 and not self.lift_complete_logged:
            self.lift_complete_logged = True
            self.get_logger().info(f"CUBE LIFTED: z={cube_z:.3f}")

        self.call_set_pose(cube_x, cube_y, cube_z)

    def release_to_ground(self):
        self.cube_z = self.ground_z
        self.call_set_pose(self.cube_x, self.cube_y, self.cube_z)
        self.get_logger().info(
            f"CUBE RELEASED TO GROUND: cube x={self.cube_x:.3f}, "
            f"y={self.cube_y:.3f}, z={self.cube_z:.3f}"
        )

    def call_set_pose(self, x, y, z):
        if not self.client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn("Set pose service not available")
            return

        req = SetEntityPose.Request()
        req.entity.name = self.cube_name
        req.entity.type = 2  # MODEL

        req.pose = Pose()
        req.pose.position.x = float(x)
        req.pose.position.y = float(y)
        req.pose.position.z = float(z)

        req.pose.orientation.x = 0.0
        req.pose.orientation.y = 0.0
        req.pose.orientation.z = 0.0
        req.pose.orientation.w = 1.0

        self.client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboCubeDynamicAttach()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
