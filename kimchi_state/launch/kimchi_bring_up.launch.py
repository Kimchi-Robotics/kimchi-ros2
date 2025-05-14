import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
# from launch.actions import ExecuteProcess


def generate_launch_description():
    pkg_kimchi_state = get_package_share_directory("kimchi_state")
    pkg_kimchi_map = get_package_share_directory("kimchi_map")
    pkg_kimchi_navigation = get_package_share_directory("kimchi_navigation")
 
    kimchi_state_server_node = Node(
        package="kimchi_state",
        executable="kimchi_state_server",
    )

    kimchi_map_handler_node = Node(
        package="kimchi_map",
        executable="kimchi_map_handler",
    )

    kimchi_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_kimchi_navigation, "launch", "kimchi_slam.launch.py")),
    )

    # # TODO: Add another command to deactivate navigation
    # deactivate_slam_toolbox_node = ExecuteProcess(
    #     cmd=[[
    #         'ros2',
    #         " service call ",
    #         "/slam_toolbox/change_state",
    #         " lifecycle_msgs/srv/ChangeState ",
    #         '"{transition: {id: 4}}"',
    #     ]],
    #     shell=True
    # )

    ld = LaunchDescription()

    # Nodes
    ld.add_action(kimchi_state_server_node)
    ld.add_action(kimchi_map_handler_node)

    # Launch files
    # ld.add_action(kimchi_slam_launch)

    # Commands
    # ld.add_action(deactivate_slam_toolbox_node)
    return ld
