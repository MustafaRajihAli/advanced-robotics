from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="novus_robotics_bridge",
                executable="bridge_node",
                name="novus_robotics_bridge",
                output="screen",
            ),
        ]
    )
