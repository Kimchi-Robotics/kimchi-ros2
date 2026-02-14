# Kimchi Data Logger

A ROS 2 Python package for logging odometry and velocity command data to CSV files for analysis and plotting.

## Overview

This package subscribes to `/odom` and `/cmd_vel` topics and saves the data to CSV files with timestamps. The logged data can be used for analysis, plotting, and debugging robot behavior.

## Features

- Subscribes to odometry (`nav_msgs/Odometry`) and velocity commands (`geometry_msgs/Twist`)
- Logs data at a configurable rate (default: 10 Hz)
- Saves data to CSV files with timestamps
- Configurable output directory and topic names
- Includes a plotting script for data visualization

## Installation

1. Navigate to your ROS 2 workspace:
   ```bash
   cd /home/arilow/Desktop/TPP/kimchi/kimchi-ros2
   ```

2. Build the package:
   ```bash
   colcon build --packages-select kimchi_data_logger
   ```

3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Usage

### Running the Data Logger

**Basic usage:**
```bash
ros2 launch kimchi_data_logger data_logger.launch.py
```

**With custom parameters:**
```bash
ros2 launch kimchi_data_logger data_logger.launch.py \
    output_dir:=/path/to/logs \
    odom_topic:=/robot/odom \
    cmd_vel_topic:=/robot/cmd_vel \
    log_rate:=20.0 \
    session_name:=my_experiment
```

**Run the node directly:**
```bash
ros2 run kimchi_data_logger data_logger_node
```

### Parameters

- `output_dir` (string, default: `~/kimchi_logs`): Directory where CSV files will be saved
- `odom_topic` (string, default: `/odom`): Odometry topic to subscribe to
- `cmd_vel_topic` (string, default: `/cmd_vel`): Velocity command topic to subscribe to
- `log_rate` (double, default: `10.0`): Rate in Hz at which to log data
- `session_name` (string, default: timestamp): Name prefix for log files

### Output Files

The package creates two CSV files per session:
- `odom_<session_name>.csv`: Odometry data
- `cmd_vel_<session_name>.csv`: Velocity command data

#### Odometry CSV Columns
- `timestamp_sec`, `timestamp_nsec`: ROS time
- `pos_x`, `pos_y`, `pos_z`: Position
- `orient_x`, `orient_y`, `orient_z`, `orient_w`: Orientation (quaternion)
- `linear_vel_x`, `linear_vel_y`, `linear_vel_z`: Linear velocities
- `angular_vel_x`, `angular_vel_y`, `angular_vel_z`: Angular velocities

#### Cmd_vel CSV Columns
- `timestamp_sec`, `timestamp_nsec`: ROS time
- `linear_vel_x`, `linear_vel_y`, `linear_vel_z`: Commanded linear velocities
- `angular_vel_x`, `angular_vel_y`, `angular_vel_z`: Commanded angular velocities

## Plotting Data

Use the included plotting script to visualize the logged data:

```bash
python3 src/kimchi_data_logger/kimchi_data_logger/plot_data.py \
    --odom ~/kimchi_logs/odom_<session_name>.csv \
    --cmd_vel ~/kimchi_logs/cmd_vel_<session_name>.csv
```

Or simply provide the directory:
```bash
python3 src/kimchi_data_logger/kimchi_data_logger/plot_data.py \
    --log_dir ~/kimchi_logs \
    --session <session_name>
```

## Example Workflow

1. Start your robot or simulation
2. Launch the data logger:
   ```bash
   ros2 launch kimchi_data_logger data_logger.launch.py session_name:=test_run_1
   ```
3. Operate your robot (teleop, navigation, etc.)
4. Stop the logger (Ctrl+C)
5. Plot the data:
   ```bash
   python3 src/kimchi_data_logger/kimchi_data_logger/plot_data.py \
       --log_dir ~/kimchi_logs --session test_run_1
   ```

## Dependencies

- ROS 2 (tested on Humble)
- rclpy
- nav_msgs
- geometry_msgs
- Python 3
- matplotlib (for plotting)
- pandas (for data analysis)

## License

Apache License 2.0

## Maintainer

arilow (alowy@fi.uba.ar)
