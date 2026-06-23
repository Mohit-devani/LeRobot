from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cube_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gazebo_cube_tf",
        arguments=[
            "0.30", "0.00", "0.025",
            "0", "0", "0",
            "base_link", "cube_link"
        ],
        output="screen"
    )

    return LaunchDescription([
        cube_tf
    ])
