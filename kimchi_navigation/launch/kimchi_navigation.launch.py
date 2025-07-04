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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_kimchi_nav = get_package_share_directory("kimchi_navigation")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")
    pkg_nav2_map_server = get_package_share_directory("nav2_map_server")

    rviz_config_file_argunment = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(pkg_nav2_bringup, "rviz", "nav2_default_view.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time",
    )

    map_argunment = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(pkg_kimchi_nav, "maps/hq_map", "map.yaml"),
        description="Full path to the map file to use",
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, "launch", "bringup_launch.py")),
        launch_arguments={
            'params_file': os.path.join(pkg_kimchi_nav, 'params', 'nav2_params.yaml'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'false',
            'map': LaunchConfiguration('map'),
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
    ld.add_action(map_argunment)
    ld.add_action(use_sim_time_argument)
    ld.add_action(rviz_config_file_argunment)

    # Launch files.
    ld.add_action(navigation_launch)
    ld.add_action(map_saver_launch)

    # Nodes
    ld.add_action(rviz)
    return ld
