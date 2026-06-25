from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    gazebo_pkg = get_package_share_directory("so101_gazebo")

    spawn_ground_cube = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_pkg,
                "launch",
                "spawn_so101_ground_cube.launch.py"
            )
        )
    )

    camera_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_pkg,
                "launch",
                "wrist_camera_bridge.launch.py"
            )
        )
    )

    color_detector = Node(
        package="so101_gazebo",
        executable="wrist_color_detector.py",
        name="wrist_color_detector",
        output="screen"
    )

    camera_search = Node(
        package="so101_gazebo",
        executable="ground_cube_camera_search.py",
        name="ground_cube_camera_search",
        output="screen"
    )

    alignment_check = Node(
        package="so101_gazebo",
        executable="ground_cube_alignment_check.py",
        name="ground_cube_alignment_check",
        output="screen"
    )

    return LaunchDescription([
        spawn_ground_cube,

        TimerAction(period=7.0, actions=[camera_bridge]),
        TimerAction(period=10.0, actions=[color_detector]),

        # Move arm once to stable ground-cube camera pose.
        TimerAction(period=12.0, actions=[camera_search]),

        # Check whether cube is in the correct lower-middle camera window.
        TimerAction(period=18.0, actions=[alignment_check]),
    ])
