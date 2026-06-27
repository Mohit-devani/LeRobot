from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
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
        executable="dynamic_gazebo_cube_pose_publisher.py",
        name="dynamic_gazebo_cube_pose_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": True}
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
        gazebo_robot,
        clock_bridge,

        TimerAction(period=4.0, actions=[
            set_pose_bridge,
            simple_gripper_tf,
            cube_pose_publisher,
            cube_attach,
        ]),

        TimerAction(period=5.0, actions=[move_group]),
        TimerAction(period=18.0, actions=[cube_joint_pick]),
    ])
