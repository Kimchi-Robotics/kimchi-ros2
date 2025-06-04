from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, LogInfo
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    movement_pattern = LaunchConfiguration('movement_pattern', default='rotate_and_advance')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('movement_pattern', default_value='rotate_and_advance',
                              description='Movement pattern for localization (spiral, rotate, zigzag, rotate_and_advance)'),
        
        # Custom nodes for auto-localization
        Node(
            package='kimchi_navigation',
            executable='auto_initial_pose_estimator',
            name='auto_initial_pose_estimator',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'convergence_threshold': 0.5,
                'min_samples_before_convergence': 5
            }],
            # Launch this after AMCL is started
            on_exit=LogInfo(msg="Initial pose estimator has completed its task")
        ),

        Node(
            package='kimchi_navigation',
            executable='localization_movement_controller',
            name='localization_movement_controller',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'linear_speed': 0.1,
                'angular_speed': 0.3,
                'movement_pattern': movement_pattern
            }]
        )
    ])