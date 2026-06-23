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

        # Start attach system after Gazebo starts spawning robot
        TimerAction(
            period=9.0,
            actions=[cube_attach]
        ),

        # Start automatic sequence after controllers + attach system are ready
        TimerAction(
            period=16.0,
            actions=[auto_pick_place]
        ),
    ])
