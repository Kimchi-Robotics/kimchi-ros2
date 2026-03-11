#!/usr/bin/env python3
"""Launch file for the data logger node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for data logger node."""
    
    # Declare launch arguments
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='~/kimchi_logs',
        description='Directory to save CSV log files'
    )
    
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Odometry topic to subscribe to'
    )
    
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Velocity command topic to subscribe to'
    )
    
    log_rate_arg = DeclareLaunchArgument(
        'log_rate',
        default_value='10.0',
        description='Rate (Hz) at which to log data'
    )
    
    session_name_arg = DeclareLaunchArgument(
        'session_name',
        default_value='',
        description='Session name for log files (defaults to timestamp)'
    )
    
    # Create the data logger node
    data_logger_node = Node(
        package='kimchi_data_logger',
        executable='data_logger_node',
        name='data_logger_node',
        output='screen',
        parameters=[{
            'output_dir': LaunchConfiguration('output_dir'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'log_rate': LaunchConfiguration('log_rate'),
            'session_name': LaunchConfiguration('session_name'),
        }]
    )
    
    return LaunchDescription([
        output_dir_arg,
        odom_topic_arg,
        cmd_vel_topic_arg,
        log_rate_arg,
        session_name_arg,
        data_logger_node
    ])
