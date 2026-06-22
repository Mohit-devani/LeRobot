from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("so101_new_calib", package_name="so101_moveit_config")
        .to_moveit_configs()
    )

    return LaunchDescription([
        Node(
            package="so101_moveit_cpp",
            executable="moveit_reach_cube_position",
            name="moveit_reach_cube_position",
            output="screen",
            parameters=[
                moveit_config.to_dict()
            ],
        )
    ])
