#!/usr/bin/env python3
"""
Data Logger Node for Kimchi Robot

This node subscribes to /odom and /cmd_vel topics and logs the data to CSV files.
The data can be used for plotting and analysis.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import csv
import os
from datetime import datetime
from pathlib import Path


class DataLoggerNode(Node):
    """
    ROS 2 node that subscribes to odometry and velocity command topics
    and saves the data to CSV files.
    """

    def __init__(self):
        super().__init__('data_logger_node')

        # Declare parameters
        self.declare_parameter('output_dir', '~/kimchi_logs')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('log_rate', 10.0)  # Hz
        self.declare_parameter('session_name', '')

        # Get parameters
        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.log_rate = self.get_parameter('log_rate').get_parameter_value().double_value
        session_name = self.get_parameter('session_name').get_parameter_value().string_value

        # Expand the output directory path
        output_dir = os.path.expanduser(output_dir)
        Path(output_dir).mkdir(parents=True, exist_ok=True)

        # Create session name if not provided
        if not session_name:
            session_name = datetime.now().strftime('%Y%m%d_%H%M%S')

        # Create CSV files
        self.odom_csv_path = os.path.join(output_dir, f'odom_{session_name}.csv')
        self.cmd_vel_csv_path = os.path.join(output_dir, f'cmd_vel_{session_name}.csv')

        # Initialize CSV files with headers
        self._init_odom_csv()
        self._init_cmd_vel_csv()

        # Data storage for rate-limited logging
        self.last_odom = None
        self.last_cmd_vel = None

        # Create subscriptions
        self.odom_subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )

        # Create timer for periodic logging
        timer_period = 1.0 / self.log_rate
        self.timer = self.create_timer(timer_period, self.log_data)

        # Logging statistics
        self.odom_count = 0
        self.cmd_vel_count = 0

        self.get_logger().info(f'Data Logger Node initialized')
        self.get_logger().info(f'Logging odometry to: {self.odom_csv_path}')
        self.get_logger().info(f'Logging cmd_vel to: {self.cmd_vel_csv_path}')
        self.get_logger().info(f'Log rate: {self.log_rate} Hz')

    def _init_odom_csv(self):
        """Initialize the odometry CSV file with headers."""
        with open(self.odom_csv_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                'timestamp_sec',
                'timestamp_nsec',
                'pos_x',
                'pos_y',
                'pos_z',
                'orient_x',
                'orient_y',
                'orient_z',
                'orient_w',
                'linear_vel_x',
                'linear_vel_y',
                'linear_vel_z',
                'angular_vel_x',
                'angular_vel_y',
                'angular_vel_z'
            ])

    def _init_cmd_vel_csv(self):
        """Initialize the cmd_vel CSV file with headers."""
        with open(self.cmd_vel_csv_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                'timestamp_sec',
                'timestamp_nsec',
                'linear_vel_x',
                'linear_vel_y',
                'linear_vel_z',
                'angular_vel_x',
                'angular_vel_y',
                'angular_vel_z'
            ])

    def odom_callback(self, msg):
        """Callback for odometry messages."""
        self.last_odom = msg

    def cmd_vel_callback(self, msg):
        """Callback for cmd_vel messages."""
        self.last_cmd_vel = msg

    def log_data(self):
        """Log data at the specified rate."""
        current_time = self.get_clock().now()

        # Log odometry data
        if self.last_odom is not None:
            self._log_odom(self.last_odom, current_time)

        # Log cmd_vel data
        if self.last_cmd_vel is not None:
            self._log_cmd_vel(self.last_cmd_vel, current_time)

    def _log_odom(self, msg, timestamp):
        """Log odometry message to CSV."""
        with open(self.odom_csv_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                timestamp.seconds_nanoseconds()[0],
                timestamp.seconds_nanoseconds()[1],
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z
            ])
        self.odom_count += 1

    def _log_cmd_vel(self, msg, timestamp):
        """Log cmd_vel message to CSV."""
        with open(self.cmd_vel_csv_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                timestamp.seconds_nanoseconds()[0],
                timestamp.seconds_nanoseconds()[1],
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.x,
                msg.angular.y,
                msg.angular.z
            ])
        self.cmd_vel_count += 1

    def destroy_node(self):
        """Clean up when node is destroyed."""
        self.get_logger().info(f'Logged {self.odom_count} odometry messages')
        self.get_logger().info(f'Logged {self.cmd_vel_count} cmd_vel messages')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = DataLoggerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
