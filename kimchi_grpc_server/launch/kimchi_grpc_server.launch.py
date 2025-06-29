from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kimchi_grpc_server',
            executable='kimchi_grpc_server',
        ),
    ])
