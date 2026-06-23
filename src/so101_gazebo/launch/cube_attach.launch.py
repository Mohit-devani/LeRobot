from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    set_pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="set_pose_bridge",
        output="screen",
        arguments=[
            "/world/so101_world/set_pose@ros_gz_interfaces/srv/SetEntityPose"
        ]
    )

    simple_gripper_tf = Node(
        package="so101_gazebo",
        executable="simple_gripper_tf.py",
        name="simple_gripper_tf",
        output="screen"
    )

    cube_attach = Node(
        package="so101_gazebo",
        executable="gazebo_cube_attach.py",
        name="gazebo_cube_attach",
        output="screen"
    )

    return LaunchDescription([
        set_pose_bridge,
        simple_gripper_tf,
        cube_attach
    ])
