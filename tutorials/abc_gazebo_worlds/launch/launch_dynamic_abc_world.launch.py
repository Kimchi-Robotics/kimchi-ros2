import os
import subprocess as sp

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generates launch description to bring up the abc_world, created dynamically from an erb file

    Returns
    -------
        LaunchDescription: launchfile description

    """
    default_world_name = "abc_dynamic_world.world.erb"
    world_dir_path = get_package_share_directory("abc_gazebo_worlds")
    world_path = os.path.join(world_dir_path, "dynamic_worlds", default_world_name)

    # Selecting world for Gazebo
    world_argument = DeclareLaunchArgument(
        "abc_world_name",
        default_value=default_world_name,
        choices=[default_world_name, "abc_world.world"],
        description="SDF world file name",
    )

    gz_verbose = DeclareLaunchArgument("gz_verbose", default_value="false", description="Level of verbosity for Gazebo")

    # Load the dynamic world and save it in a tmp file
    world_file_dynamic = sp.run(["erb", world_path], stdout=sp.PIPE, encoding="utf-8", check=False)
    with open("/tmp/sdf_loaded", "w") as tmp_file:
        tmp_file.write(world_file_dynamic.stdout)

    # Gazebo app
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("gazebo_ros"), "launch"), "/gazebo.launch.py"]
        ),
        launch_arguments={"world": "/tmp/sdf_loaded", "verbose": LaunchConfiguration("gz_verbose")}.items(),
    )
    # Run the node
    return LaunchDescription([world_argument, gz_verbose, gazebo])
