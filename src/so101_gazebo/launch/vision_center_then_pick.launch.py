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

    set_pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="vision_set_pose_bridge",
        output="screen",
        arguments=[
            "/world/so101_world/set_pose@ros_gz_interfaces/srv/SetEntityPose"
        ]
    )

    vision_setup = Node(
        package="so101_gazebo",
        executable="wrist_vision_test_setup.py",
        name="wrist_vision_test_setup",
        output="screen"
    )

    color_detector = Node(
        package="so101_gazebo",
        executable="wrist_color_detector.py",
        name="wrist_color_detector",
        output="screen"
    )

    visual_servo_error = Node(
        package="so101_gazebo",
        executable="visual_servo_error.py",
        name="visual_servo_error",
        output="screen"
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

    visual_centering = Node(
        package="so101_gazebo",
        executable="visual_servo_centering_sequence.py",
        name="visual_servo_centering_sequence",
        output="screen"
    )

    center_then_pick = Node(
        package="so101_gazebo",
        executable="center_then_pick_sequence.py",
        name="center_then_pick_sequence",
        output="screen"
    )

    return LaunchDescription([
        spawn_gazebo,

        TimerAction(
            period=7.0,
            actions=[camera_bridge]
        ),

        TimerAction(
            period=8.0,
            actions=[set_pose_bridge]
        ),

        TimerAction(
            period=10.0,
            actions=[cube_attach]
        ),

        TimerAction(
            period=11.0,
            actions=[vision_setup]
        ),

        TimerAction(
            period=16.0,
            actions=[color_detector]
        ),

        TimerAction(
            period=17.0,
            actions=[visual_servo_error]
        ),

        # Start this early so it is already waiting for done signal.
        TimerAction(
            period=18.0,
            actions=[center_then_pick]
        ),

        TimerAction(
            period=20.0,
            actions=[visual_centering]
        ),
    ])
