import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_kimchi_nav = get_package_share_directory("kimchi_navigation")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")
    pkg_slam_toolbox = get_package_share_directory("slam_toolbox")
    pkg_nav2_map_server = get_package_share_directory("nav2_map_server")

    rviz_config_file_argunment = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(pkg_kimchi_nav, "rviz", "kimchi_navigation.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time",
    )


    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, "launch", "navigation_launch.py")),
        launch_arguments={
            'use_sim_time': LaunchConfiguration("use_sim_time"),
        }.items(),
        # launch_arguments={
        #     'params_file': os.path.join(pkg_kimchi_nav, 'params', 'nav2_params.yaml'),
        # }.items(),
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_slam_toolbox, "launch", "online_async_launch.py")),
        launch_arguments={
            'use_sim_time': LaunchConfiguration("use_sim_time"),
            'slams_param_file': os.path.join(pkg_slam_toolbox, 'config', 'mapper_params_online_async.yaml'),
        }.items(),
    )

    map_saver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_map_server, "launch", "map_saver_server.launch.py")),
    )

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config_file")],
    )

    ld = LaunchDescription()

    # Arguments
    ld.add_action(rviz_config_file_argunment)
    ld.add_action(use_sim_time_argument)

    # Launch files.
    ld.add_action(navigation_launch)
    ld.add_action(slam_toolbox_launch)
    ld.add_action(map_saver_launch)

    # Nodes
    ld.add_action(rviz)
    return ld
