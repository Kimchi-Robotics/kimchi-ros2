import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_andino_gz = get_package_share_directory("andino_gz")
    pkg_kimchi_state = get_package_share_directory("kimchi_state")
    pkg_kimchi_grpc_server = get_package_share_directory("kimchi_grpc_server")

    use_sim_time = 'True'  # Set to True for simulation

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_andino_gz, "launch", "andino_gz.launch.py")),
        launch_arguments={
            'nav2': 'False',
            'rviz': 'False',
            'world_name': 'populated_office.sdf',
        }.items(),
    )

    kimchi_state_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_kimchi_state, "launch", "kimchi_state_server.launch.py")),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
    )

    kimchi_grpc_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_kimchi_grpc_server, "launch", "kimchi_grpc_server.launch.py")),
    )

    ld = LaunchDescription()

    # Launch files
    ld.add_action(simulation_launch)
    ld.add_action(kimchi_state_server_launch)
    ld.add_action(kimchi_grpc_server_launch)

    return ld