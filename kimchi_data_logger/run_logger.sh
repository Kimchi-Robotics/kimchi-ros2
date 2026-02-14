#!/bin/bash
# Example script to run the data logger with custom settings

# Set the session name
SESSION_NAME="test_$(date +%Y%m%d_%H%M%S)"

echo "Starting data logger with session: $SESSION_NAME"
echo "Output directory: ~/kimchi_logs"
echo "Press Ctrl+C to stop logging"
echo ""

# Launch the data logger
ros2 launch kimchi_data_logger data_logger.launch.py \
    session_name:=$SESSION_NAME \
    log_rate:=20.0

echo ""
echo "Logging stopped. Files saved to ~/kimchi_logs"
echo "To plot the data, run:"
echo "python3 src/kimchi_data_logger/kimchi_data_logger/plot_data.py --log_dir ~/kimchi_logs --session $SESSION_NAME"
