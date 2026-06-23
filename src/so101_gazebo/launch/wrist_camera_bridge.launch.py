from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    # Dedicated Gazebo image bridge.
    # This is more reliable for camera image topics than parameter_bridge.
    wrist_image_bridge = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "ros_gz_image",
            "image_bridge",
            "/wrist_camera/image"
        ],
        output="screen"
    )

    # CameraInfo bridge can stay as parameter_bridge.
    wrist_camera_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="wrist_camera_info_bridge",
        output="screen",
        arguments=[
            "/wrist_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo"
        ]
    )

    return LaunchDescription([
        wrist_image_bridge,
        wrist_camera_info_bridge
    ])
