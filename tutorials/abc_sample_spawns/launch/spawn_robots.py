import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generates launch description to spawn multiple robots

    Returns
    -------
        LaunchDescription: launchfile description

    """
    # Specify the name of the package and path to xacro file within the package
    pkg_name = "abc_sample_spawns"
    file_subpath_xacro = "urdf/rrbot.urdf.xacro"

    # Convert XACRO to URDF
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath_xacro)
    robot_desc_xacro = xacro.process_file(xacro_file).toxml()

    # Load plain URDF
    urdf_file = os.path.join(get_package_share_directory(pkg_name), "urdf/rrbot.urdf")
    with open(urdf_file, "rt", encoding="utf-8") as urdf_file:
        robot_desc_urdf = urdf_file.read()

    # Load plain SDF
    robot_desc_sdf = os.path.join(get_package_share_directory(pkg_name), "sdf/rrbot.sdf")

    # robot-state-publisher [XACRO mode]
    robot_state_publisher_xacro = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc_xacro, "use_sim_time": True}],
        remappings=[("robot_description", "rrbot_desc_xacro")],
    )

    # robot-state-publisher [URDF mode]
    robot_state_publisher_urdf = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc_urdf, "use_sim_time": True}],
        remappings=[("robot_description", "rrbot_desc_urdf")],
    )

    """
    Spawn the rrbot 3 times
    """

    # Spawn the robot [XACRO mode]
    spawn_entity_xacro = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "rrbot_desc_xacro", "-entity", "rrbot_xacro", "-x", "1", "-y", "0"],
        output="screen",
    )

    # Spawn the robot [URDF mdoe]
    spawn_entity_urdf = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "rrbot_desc_urdf", "-entity", "rrbot_urdf", "-x", "1", "-y", "1"],
        output="screen",
    )

    # Spawn the robot [SDF mode]
    spawn_entity_sdf = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-f", robot_desc_sdf, "-entity", "rrbot_sdf", "-x", "1", "-y", "-1"],
        output="screen",
    )

    # Run the nodes
    return LaunchDescription(
        [
            spawn_entity_sdf,
            spawn_entity_urdf,
            spawn_entity_xacro,
            robot_state_publisher_urdf,
            robot_state_publisher_xacro,
        ]
    )
