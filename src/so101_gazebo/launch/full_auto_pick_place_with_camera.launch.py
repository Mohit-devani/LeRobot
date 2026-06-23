from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    gazebo_pkg = get_package_share_directory("so101_gazebo")

    spawn_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_pkg,
                "launch",
                "spawn_so101_gazebo.launch.py"
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

    cube_attach = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_pkg,
                "launch",
                "cube_attach.launch.py"
            )
        )
    )

    auto_pick_place = Node(
        package="so101_gazebo",
        executable="gazebo_pick_place_sequence.py",
        name="gazebo_pick_place_sequence",
        output="screen"
    )

    return LaunchDescription([
        spawn_gazebo,

        # Camera bridge after Gazebo camera topic exists
        TimerAction(
            period=7.0,
            actions=[camera_bridge]
        ),

        # Attach system after robot/controllers exist
        TimerAction(
            period=9.0,
            actions=[cube_attach]
        ),

        # Auto pick-place after everything is alive
        TimerAction(
            period=16.0,
            actions=[auto_pick_place]
        ),
    ])
