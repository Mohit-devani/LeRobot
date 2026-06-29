from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    gazebo_pkg = get_package_share_directory("so101_gazebo")

    world_path = os.path.join(gazebo_pkg, "worlds", "so101_multi_cube.sdf")
    urdf_path = os.path.join(gazebo_pkg, "urdf", "so101_gazebo.urdf")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": f"-r {world_path}"
        }.items()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }]
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "so101",
            "-x", "0",
            "-y", "0",
            "-z", "0.0"
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        TimerAction(period=3.0, actions=[spawn_robot]),
        TimerAction(period=6.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=7.0, actions=[arm_controller_spawner]),
        TimerAction(period=8.0, actions=[gripper_controller_spawner]),
    ])
