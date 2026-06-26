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

    set_pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ground_pose_set_pose_bridge",
        output="screen",
        arguments=[
            "/world/so101_world/set_pose@ros_gz_interfaces/srv/SetEntityPose"
        ]
    )

    pose_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gazebo_pose_info_bridge",
        output="screen",
        arguments=[
            "/world/so101_world/pose/info@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V"
        ]
    )

    simple_gripper_tf = Node(
        package="so101_gazebo",
        executable="simple_gripper_tf.py",
        name="simple_gripper_tf",
        output="screen"
    )

    dynamic_cube_attach = Node(
        package="so101_gazebo",
        executable="gazebo_cube_dynamic_attach.py",
        name="gazebo_cube_dynamic_attach",
        output="screen"
    )

    # Old fixed-offset cube_attach removed for ground final demo.
    camera_search = Node(
        package="so101_gazebo",
        executable="ground_cube_camera_search.py",
        name="ground_cube_camera_search",
        output="screen"
    )

    color_detector = Node(
        package="so101_gazebo",
        executable="wrist_cube_center_detector.py",
        name="wrist_cube_center_detector",
        output="screen"
    )

    cube_pose_pub = Node(
        package="so101_gazebo",
        executable="dynamic_gazebo_cube_pose_publisher.py",
        name="gazebo_cube_pose_publisher",
        output="screen"
    )

    ground_pose_pick = Node(
        package="so101_gazebo",
        executable="ground_pose_pick_sequence.py",
        name="ground_pose_pick_sequence",
        output="screen"
    )

    return LaunchDescription([
        spawn_ground_cube,

        # Core Gazebo services.
        TimerAction(period=6.0, actions=[set_pose_bridge]),
        TimerAction(period=7.0, actions=[simple_gripper_tf, dynamic_cube_attach]),
        TimerAction(period=8.0, actions=[pose_info_bridge]),

        # Camera bridge.
        TimerAction(period=9.0, actions=[camera_bridge]),

        # Publish live cube pose from Gazebo.
        TimerAction(period=10.0, actions=[cube_pose_pub]),

        # Move to camera-search pose first.
        TimerAction(period=11.0, actions=[camera_search]),

        # Start detector after camera-search pose has enough time to finish.
        TimerAction(period=24.0, actions=[color_detector]),

        # Start pick sequence after detector has stable cube visibility.
        TimerAction(period=28.0, actions=[ground_pose_pick]),
    ])
