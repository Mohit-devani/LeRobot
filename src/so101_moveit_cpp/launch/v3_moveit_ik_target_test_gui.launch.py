from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():
    gazebo_pkg = get_package_share_directory("so101_gazebo")
    moveit_config_pkg = get_package_share_directory("so101_moveit_config")

    moveit_config = (
        MoveItConfigsBuilder(
            "so101_new_calib",
            package_name="so101_moveit_config"
        )
        .to_moveit_configs()
    )

    gazebo_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_pkg,
                "launch",
                "spawn_so101_ground_cube.launch.py"
            )
        )
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ],
        output="screen"
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="moveit_rviz",
        output="screen",
        arguments=[
            "-d",
            os.path.join(
                moveit_config_pkg,
                "config",
                "moveit.rviz"
            )
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True}
        ],
    )

    ik_test = Node(
        package="so101_moveit_cpp",
        executable="v3_moveit_ik_target_test",
        name="v3_moveit_ik_target_test",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}
        ],
    )

    return LaunchDescription([
        gazebo_robot,
        clock_bridge,

        TimerAction(period=6.0, actions=[move_group]),
        TimerAction(period=8.0, actions=[rviz]),
        TimerAction(period=18.0, actions=[ik_test]),
    ])
