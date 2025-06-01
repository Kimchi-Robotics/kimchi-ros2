import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the path to your robot's configuration directory
    # IMPORTANT: Replace 'your_robot_package' with the actual name of your ROS2 package
    pkg_kimchi_nav = get_package_share_directory('kimchi_navigation')

    # Define the path to your map file
    map_file_path = os.path.join(pkg_kimchi_nav, 'maps/hq_map', 'map.yaml')

    # Declare launch arguments for flexibility
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=map_file_path,
        description='Full path to map file to load'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true', # Set to 'true' if running in simulation (Gazebo, etc.)
        description='Use simulation (Gazebo) clock if true'
    )

    # --- Map Server Node ---
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # --- AMCL Node ---
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),

            # IMPORTANT: For global localization without an initial guess:
            # Do NOT set 'set_initial_pose: true' here, and do NOT provide
            # specific 'initial_pose.x', 'initial_pose.y', etc.
            # AMCL will then default to spreading particles across the map.

            # Lidar sensor parameters
            'laser_model_type': 'likelihood_field',
            'laser_max_range': 30.0,
            'laser_min_range': 0.1,
            'laser_max_beams': 180,

            # Frame IDs (crucial for TF tree consistency)
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
            'global_frame_id': 'map',
            'tf_broadcast': True,

            # Particle filter parameters (tune for performance vs. accuracy)
            'min_particles': 500,
            'max_particles': 5000,
            'kld_err': 0.05,
            'kld_z': 0.99,
            'update_min_d': 0.25,
            'update_min_a': 0.20,
            'resample_interval': 1,

            # Odometry model parameters (tune these to match your robot's motion characteristics)
            'odom_alpha1': 0.1,
            'odom_alpha2': 0.1,
            'odom_alpha3': 0.1,
            'odom_alpha4': 0.1,
            'odom_alpha5': 0.1,

            # Other important parameters
            'transform_tolerance': 0.1,
            'recovery_alpha_slow': 0.001,
            'recovery_alpha_fast': 0.1,
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    return LaunchDescription([
        map_arg,
        use_sim_time_arg,
        map_server_node,
        amcl_node
    ])
