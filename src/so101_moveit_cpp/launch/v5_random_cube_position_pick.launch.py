from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():
    gazebo_pkg = get_package_share_directory("so101_gazebo")

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

    wrist_camera_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_pkg,
                "launch",
                "wrist_camera_bridge.launch.py"
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

    set_pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="moveit_set_pose_bridge",
        arguments=[
            "/world/so101_world/set_pose@ros_gz_interfaces/srv/SetEntityPose"
        ],
        output="screen"
    )

    wrist_detector = Node(
        package="so101_gazebo",
        executable="wrist_cube_center_detector.py",
        name="wrist_cube_center_detector",
        output="screen",
        parameters=[
            {"use_sim_time": True}
        ],
    )

    simple_gripper_tf = Node(
        package="so101_gazebo",
        executable="simple_gripper_tf.py",
        name="simple_gripper_tf",
        output="screen",
        parameters=[
            {"use_sim_time": True}
        ],
    )

    cube_pose_publisher = Node(
        package="so101_gazebo",
        executable="v4_camera_cube_pose_publisher.py",
        name="v4_camera_cube_pose_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": True}
        ],
    )

    random_cube_positioner = Node(
        package="so101_gazebo",
        executable="v5_random_cube_positioner.py",
        name="v5_random_cube_positioner",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"cube_position": LaunchConfiguration("cube_position")},
        ],
    )


    cube_attach = Node(
        package="so101_gazebo",
        executable="gazebo_cube_dynamic_attach.py",
        name="gazebo_cube_dynamic_attach",
        output="screen",
        parameters=[
            {"use_sim_time": True}
        ],
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

    cube_joint_pick = Node(
        package="so101_moveit_cpp",
        executable="v3_moveit_cube_joint_pick",
        name="v3_moveit_cube_joint_pick",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "cube_position",
            default_value="RANDOM",
            description="Cube start position: RANDOM, LEFT, CENTER, or RIGHT"
        ),

        gazebo_robot,
        clock_bridge,

        TimerAction(period=4.0, actions=[
            set_pose_bridge,
            wrist_camera_bridge,
            simple_gripper_tf,
            cube_pose_publisher,
            cube_attach,
        ]),

        # Move cube to a random LEFT / CENTER / RIGHT ground position
        # after the set_pose bridge is alive, before the pick node starts.
        TimerAction(period=6.0, actions=[random_cube_positioner]),

        TimerAction(period=7.0, actions=[wrist_detector]),
        TimerAction(period=8.0, actions=[move_group]),

        # Pick node starts after detector and controllers are alive.
        # It still waits internally for /object_detected and /pick_cube_pose.
        TimerAction(period=18.0, actions=[cube_joint_pick]),
    ])
