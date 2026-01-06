#!/bin/bash
# display.sh - Alternative to display.launch.py
# This script ensures proper environment variable propagation

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# Source the workspace
source "$WS_DIR/install/setup.bash"

echo "=== DOFBOT Visualization ==="
echo "Workspace: $WS_DIR"
echo "AMENT_PREFIX_PATH: $AMENT_PREFIX_PATH"
echo ""

# Get package paths
PKG_SHARE=$(ros2 pkg prefix dofbot_description)/share/dofbot_description
URDF_FILE="$PKG_SHARE/urdf/dofbot.urdf"
RVIZ_CONFIG="$PKG_SHARE/rviz/default.rviz"

echo "URDF: $URDF_FILE"
echo "Rviz Config: $RVIZ_CONFIG"
echo ""

# Check if URDF exists
if [ ! -f "$URDF_FILE" ]; then
    echo "ERROR: URDF file not found at $URDF_FILE"
    exit 1
fi

# Read URDF content
ROBOT_DESC=$(cat "$URDF_FILE")

# Launch robot_state_publisher in background
echo "Starting robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$ROBOT_DESC" &
RSP_PID=$!

sleep 1

# Launch joint_state_publisher_gui in background
echo "Starting joint_state_publisher_gui..."
ros2 run joint_state_publisher_gui joint_state_publisher_gui &
JSP_PID=$!

sleep 1

# Launch rviz2
echo "Starting rviz2..."
ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" &
RVIZ_PID=$!

# Wait for any process to exit
wait -n

# Cleanup
echo "Shutting down..."
kill $RSP_PID $JSP_PID $RVIZ_PID 2>/dev/null
