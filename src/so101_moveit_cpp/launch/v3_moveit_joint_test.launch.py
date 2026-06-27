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

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                moveit_config_pkg,
                "launch",
                "move_group.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": "true"
        }.items()
    )

    moveit_joint_test = Node(
        package="so101_moveit_cpp",
        executable="moveit_pick_sequence",
        name="v3_moveit_joint_test",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}
        ]
    )

    return LaunchDescription([
        gazebo_robot,
        clock_bridge,

        # Start MoveIt after robot_state_publisher and clock are alive.
        TimerAction(period=6.0, actions=[move_group]),

        # Start test after Gazebo controllers and move_group are alive.
        TimerAction(period=16.0, actions=[moveit_joint_test]),
    ])
