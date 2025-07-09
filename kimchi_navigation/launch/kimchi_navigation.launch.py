# BSD 3-Clause License

# Copyright (c) 2023, Ekumen Inc.
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

pkg_kimchi_nav = get_package_share_directory("kimchi_navigation")
pkg_nav2_bringup = get_package_share_directory("nav2_bringup")
pkg_nav2_map_server = get_package_share_directory("nav2_map_server")
pkg_kimchi_nav_conf = get_package_share_directory(
    'kimchi_navigation_configurations'
)

planner_configs_install_folder = os.path.join(
    pkg_kimchi_nav_conf,
    'planner_config',
)
controller_configs_install_folder = os.path.join(
    pkg_kimchi_nav_conf,
    'controller_config',
)


def launch_setup(context, *args, **kwargs):
    complete_params_filename = '/tmp/combined_params.yaml'
    params_filename = os.path.join(
        pkg_kimchi_nav_conf, 'configurable_nav2_params', 'configurable_nav2_params.yaml')

    with open(params_filename, 'r') as file:
        params_dict = yaml.safe_load(file)

    planner_dir = os.path.join(planner_configs_install_folder, LaunchConfiguration(
        'nav_planner').perform(context))
    controller_dir = os.path.join(controller_configs_install_folder, LaunchConfiguration(
        'nav_controller').perform(context))
    overall_params = [
        os.path.join(planner_dir, file)
        for file in os.listdir(planner_dir)
    ] + [
        os.path.join(controller_dir, file)
        for file in os.listdir(controller_dir)
    ]

    for param_file_name in overall_params:
        with open(param_file_name, 'r') as file:
            data = yaml.safe_load(file)
            params_dict = {**params_dict, **data}

    with open(complete_params_filename, 'w') as outfile:
        yaml.dump(params_dict, outfile, default_flow_style=False)

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_nav2_bringup, "launch", "bringup_launch.py")),
        launch_arguments={
            'params_file': complete_params_filename,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'false',
            'map': LaunchConfiguration('map'),
        }.items(),
    )

    return [navigation_launch]


def generate_launch_description():
    available_planners = os.listdir(planner_configs_install_folder)
    available_controllers = os.listdir(controller_configs_install_folder)

    planner_arg = DeclareLaunchArgument(
        name='nav_planner',
        description='Planner to use for navigation',
        choices=available_planners,
        default_value='navfn',
    )

    controller_arg = DeclareLaunchArgument(
        name='nav_controller',
        description='Controller to use for navigation',
        choices=available_controllers,
        default_value='dwb',
    )

    rviz_config_file_argunment = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(
            pkg_nav2_bringup, "rviz", "nav2_default_view.rviz"),
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

    map_saver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_nav2_map_server, "launch", "map_saver_server.launch.py")),
    )

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config_file")],
    )

    ld = LaunchDescription()

    # Arguments
    ld.add_action(map_argunment)
    ld.add_action(use_sim_time_argument)
    ld.add_action(rviz_config_file_argunment)
    ld.add_action(planner_arg)
    ld.add_action(controller_arg)

    # Opaque function to handle the launch setup
    ld.add_action(OpaqueFunction(function=launch_setup))

    # Launch files.
    ld.add_action(map_saver_launch)

    # Nodes
    ld.add_action(rviz)

    return ld
