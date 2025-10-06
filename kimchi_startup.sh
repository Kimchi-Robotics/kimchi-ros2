#!/bin/bash

# Kimchi Robot Auto-Start Script
# Launches the Kimchi bringup system automatically on boot
# Author: Leo Neumarkt
# Last Modified: $(date +%Y-%m-%d)

set -e  # Exit on error

# Configuration
WORKSPACE_PATH="${HOME}/ws"
LOG_DIR="${HOME}/ros_logs"
LAUNCH_PACKAGE="kimchi_bringup"
LAUNCH_FILE="bringup_robot.launch.py"
ROS_DISTRO="jazzy"
ROS_DOMAIN_ID=10
NETWORK_WAIT=15

# Logger function
log() {
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] $1"
}

# Create log directory
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/kimchi_startup_$(date +%Y%m%d_%H%M%S).log"

# Redirect all output to log file
exec > "$LOG_FILE" 2>&1

log "===== Kimchi Robot Startup Sequence ====="
log "Hostname: $(hostname)"
log "User: $(whoami)"

# Wait for network to be ready
log "Waiting ${NETWORK_WAIT}s for network initialization..."
sleep "$NETWORK_WAIT"

# Get network information
LOCAL_IP=$(hostname -I | awk '{print $1}')
if [ -z "$LOCAL_IP" ]; then
    log "WARNING: No IP address found. Network may not be ready."
    LOCAL_IP="unknown"
fi
log "Local IP: $LOCAL_IP"

# Source ROS2 environment
log "Sourcing ROS2 ${ROS_DISTRO} environment..."
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    log "ROS2 environment sourced successfully"
else
    log "ERROR: ROS2 environment not found at /opt/ros/${ROS_DISTRO}/setup.bash"
    exit 1
fi

# Source workspace
log "Sourcing workspace at ${WORKSPACE_PATH}..."
if [ -f "${WORKSPACE_PATH}/install/setup.bash" ]; then
    source "${WORKSPACE_PATH}/install/setup.bash"
    log "Workspace sourced successfully"
else
    log "ERROR: Workspace not found at ${WORKSPACE_PATH}/install/setup.bash"
    exit 1
fi

# Set ROS environment variables
export ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
log "ROS_DOMAIN_ID set to: $ROS_DOMAIN_ID"

# Verify launch file exists
log "Verifying launch file..."
if ! ros2 pkg prefix "$LAUNCH_PACKAGE" > /dev/null 2>&1; then
    log "ERROR: Package ${LAUNCH_PACKAGE} not found"
    exit 1
fi
log "Package ${LAUNCH_PACKAGE} found"

# Launch the robot
log "===== Starting Kimchi Robot Bringup ====="
log "Command: ros2 launch ${LAUNCH_PACKAGE} ${LAUNCH_FILE} use_kimchi:=true"
log "Launch initiated at $(date)"

# Publish robot info for network discovery (runs in background)
(
    sleep 5  # Wait for ROS2 to initialize
    # Publish with transient_local durability to persist the last message
    ros2 topic pub /kimchi/robot_info std_msgs/msg/String "{data: 'Kimchi Robot - IP: ${LOCAL_IP} - Hostname: $(hostname)'}" --qos-durability transient_local --keep-all --rate 0.1 &
    PUBLISHER_PID=$!
    sleep 20  # Publish for 20 seconds
    kill $PUBLISHER_PID 2>/dev/null || true
) &

# Launch Kimchi bringup
ros2 launch "$LAUNCH_PACKAGE" "$LAUNCH_FILE" use_kimchi:=true

# This will only execute if launchfile exits
log "===== Kimchi Robot Bringup Exited ====="
log "Exit time: $(date)"