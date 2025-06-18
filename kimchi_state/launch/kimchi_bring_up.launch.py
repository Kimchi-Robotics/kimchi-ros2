import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_kimchi_navigation = get_package_share_directory("kimchi_navigation")
 
    use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time",
    )

    kimchi_state_server_node = Node(
        package="kimchi_state",
        executable="kimchi_state_server",
    )

    kimchi_map_handler_node = Node(
        package="kimchi_map",
        executable="kimchi_map_handler",
    )

    kimchi_nav_and_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_kimchi_navigation, "launch", "kimchi_nav_and_slam.launch.py")),
        launch_arguments={
            'use_sim_time': LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    ld = LaunchDescription()

    # Arguments
    ld.add_action(use_sim_time_argument)

    # Nodes
    ld.add_action(kimchi_state_server_node)
    ld.add_action(kimchi_map_handler_node)

    # Launch files
    ld.add_action(kimchi_nav_and_slam_launch)

    return ld
