#!/bin/bash

# Setup script for ELEC49X UAV Tunnel Navigation Workspace
# Simple build script for teams with ROS 2 and Gazebo already installed

set -e

echo "Building ELEC49X UAV Tunnel Navigation Workspace..."

# Get the workspace directory (parent of this script)
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE_DIR"

echo "Workspace directory: $WORKSPACE_DIR"

# Check if ROS 2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS 2 is not sourced. Please run:"
    echo "  source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "ROS_DISTRO: $ROS_DISTRO"

# Install dependencies (skip if rosdep fails - team may have packages already)
echo "Installing dependencies (if needed)..."
rosdep install --from-paths src --ignore-src -r -y || echo "Warning: Some dependencies may need manual installation"

# Build the workspace
echo "Building workspace..."
colcon build --symlink-install

echo ""
echo "=========================================="
echo "Workspace build complete!"
echo "=========================================="
echo ""
echo "To use this workspace, source it with:"
echo "  source $WORKSPACE_DIR/install/setup.bash"
echo ""
echo "To launch the simulation:"
echo "  ros2 launch uav_bringup uav_sim.launch.py"
echo ""
