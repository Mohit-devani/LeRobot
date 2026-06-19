from launch import LaunchDescription
from launch_ros.actions import Node

from pathlib import Path

def generate_launch_description():

    urdf_file = str(
        Path.home()
        / "ros2_ws/src/so101_description/urdf/so101_new_calib.urdf"
    )

    with open(urdf_file, "r") as f:
        robot_description = f.read()

    return LaunchDescription([

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description}
            ]
        ),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui"
        ),

        Node(
            package="rviz2",
            executable="rviz2"
        ),
    ])
