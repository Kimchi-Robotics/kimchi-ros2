import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generates launch description to bring up the abc_world

    Returns
    -------
        LaunchDescription: launchfile description

    """
    default_world_name = "abc_world.world"

    # Selecting world for Gazebo
    world_argument = DeclareLaunchArgument(
        "abc_world_name",
        default_value=default_world_name,
        choices=[default_world_name, "abc_dynamic_world.world.erb"],
        description="World file name.",
    )

    gz_verbose = DeclareLaunchArgument(
        "gz_verbose", default_value="true", choices=["true", "false"], description="Level of verbosity for Gazebo."
    )

    # Gazebo app
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("gazebo_ros"), "launch"), "/gazebo.launch.py"]
        ),
        launch_arguments={
            "world": LaunchConfiguration("abc_world_name"),
            "verbose": LaunchConfiguration("gz_verbose"),
        }.items(),
    )

    # Run the node
    return LaunchDescription(
        [
            world_argument,
            gz_verbose,
            gazebo,
        ]
    )
