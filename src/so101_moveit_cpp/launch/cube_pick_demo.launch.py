from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("so101_moveit_config"),
                "launch",
                "demo.launch.py"
            )
        )
    )

    cube_marker = Node(
        package="rviz_objects",
        executable="cube_marker",
        name="cube_marker",
        output="screen"
    )

    return LaunchDescription([
        moveit_demo,
        cube_marker
    ])
