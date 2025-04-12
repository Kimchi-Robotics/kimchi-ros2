import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_kimchi_state_server = get_package_share_directory("kimchi_state_server")
    pkg_kimchi_map = get_package_share_directory("kimchi_map")
 
    kimchi_state_server_node = Node(
        package="kimchi_state_server",
        executable="kimchi_state_server",
    )

    kimchi_map_handler_node = Node(
        package="kimchi_map",
        executable="kimchi_map_handler",
    )

    ld = LaunchDescription()

    # Nodes
    ld.add_action(kimchi_state_server_node)
    ld.add_action(kimchi_map_handler_node)
    return ld
