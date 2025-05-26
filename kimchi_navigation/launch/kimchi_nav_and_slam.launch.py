import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    pkg_kimchi_nav = get_package_share_directory("kimchi_navigation")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")
    pkg_slam_toolbox = get_package_share_directory("slam_toolbox")
    pkg_nav2_map_server = get_package_share_directory("nav2_map_server")

    rviz_config_file_argunment = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(pkg_nav2_bringup, "rviz", "nav2_default_view.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time",
    )

    map_argunment = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(pkg_kimchi_nav, "maps/hq_map", "map.yaml"),
        description="Full path to the map file to use",
    )

    navigation_bring_up_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, "launch", "bringup_launch.py")),
        launch_arguments={
            'params_file': os.path.join(pkg_kimchi_nav, 'params', 'nav2_params.yaml'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'false',
            'map': LaunchConfiguration('map'),
        }.items(),
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_slam_toolbox, "launch", "online_async_launch.py")),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
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

    # TODO: Add another command to deactivate navigation
    deactivate_slam_toolbox_node = ExecuteProcess(
        cmd=[[
            'ros2',
            " service call ",
            "/slam_toolbox/change_state",
            " lifecycle_msgs/srv/ChangeState ",
            '"{transition: {id: 4}}"',
        ]],
        shell=True
    )

    deactivate_bt_navigator_node = ExecuteProcess(
        cmd=[[
            'ros2',
            " service call ",
            "/bt_navigator/change_state",
            " lifecycle_msgs/srv/ChangeState ",
            '"{transition: {id: 4}}"',
        ]],
        shell=True
    )

    # WARNING: This makes a lot of nodes dissapear
    deactivate_controller_server_node = ExecuteProcess(
        cmd=[[
            'ros2',
            " service call ",
            "/controller_server/change_state",
            " lifecycle_msgs/srv/ChangeState ",
            '"{transition: {id: 4}}"',
        ]],
        shell=True
    )

    deactivate_local_costmap_node = ExecuteProcess(
        cmd=[[
            'ros2',
            " service call ",
            "/local_costmap/local_costmap/change_state",
            " lifecycle_msgs/srv/ChangeState ",
            '"{transition: {id: 4}}"',
        ]],
        shell=True
    )

    deactivate_global_costmap_node = ExecuteProcess(
        cmd=[[
            'ros2',
            " service call ",
            "/global_costmap/global_costmap/change_state",
            " lifecycle_msgs/srv/ChangeState ",
            '"{transition: {id: 4}}"',
        ]],
        shell=True
    )

    ld = LaunchDescription()

    # Arguments
    ld.add_action(map_argunment)
    ld.add_action(use_sim_time_argument)
    ld.add_action(rviz_config_file_argunment)

    # Launch files.
    ld.add_action(navigation_bring_up_launch)
    ld.add_action(slam_toolbox_launch)

    # Nodes
    ld.add_action(rviz)

    # Commands
    ld.add_action(deactivate_slam_toolbox_node)
    return ld
