ELEC 49X: UAV Self-Navigation in Tunnels


Group 27: Hamza Iqbal, Walker Yee, Muhammad Saad Iqbal


This repository contains the complete ROS 2 software stack for the Queen's University ELEC 49X Capstone Project, "UAV Self-Navigation in Tunnels."

<!--
ACTION REQUIRED: I will create a  high-quality GIF of the Gazebo simulation
in action and place it here. This is to show what our project does.
For now, this is a placeholder.
-->

1. Project Overview

The purpose of this project is to design and simulate an Unmanned Aerial Vehicle (UAV) system capable of autonomous navigation in GNSS-denied environments such as tunnels, hallways, and mines.

Our methodology is simulation-first, focusing on developing a robust and modular software stack in ROS 2. The system prioritizes UAV safety (zero collisions) and mission efficiency. It uses simulated 2D LiDAR and IMU data to perform localization, path planning, and control without any external positioning signals.

The core of this project involves two main approaches:

MVP: A custom "Geometric Centering" node that uses LiDAR data to keep the UAV in the center of a corridor, with its navigation goals managed by the Nav2 stack.

Optional Goal: Integration of an advanced LiDAR-Inertial Odometry (LIO) library (such as LIO-SAM) to achieve high-precision localization for complex, long-distance missions.

2. Getting Started

These instructions will get a copy of the project up and running on your local machine for development and testing.

Prerequisites

Ubuntu 22.04 LTS

ROS 2 Humble (Desktop-Full install)

Gazebo Fortress: Required for simulation (install separately if not included)

colcon build tools: sudo apt install python3-colcon-common-extensions

rosdep tool: sudo apt install python3-rosdep

Installation & Build

**Quick Setup (Recommended):**

```bash
# Make sure ROS 2 Humble is sourced
source /opt/ros/humble/setup.bash

# Run the setup script
cd /path/to/ELEC49X_UAV_Tunnel_Nav
./setup_workspace.sh

# Source the workspace
source install/setup.bash
```

**Manual Setup:**

1. Source ROS 2 Humble:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Install dependencies:
   ```bash
   cd /path/to/ELEC49X_UAV_Tunnel_Nav
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build the workspace:
   ```bash
   colcon build --symlink-install
   ```

4. Source the workspace:
   ```bash
   source install/setup.bash
   ```

**Testing the Workspace:**

**Test ROS 2 Integration:**
```bash
# Launch the test node
ros2 launch uav_bringup test_workspace.launch.py

# In another terminal, verify topics are working
source install/setup.bash
ros2 topic list
ros2 topic echo /test_topic
```

**Launch Gazebo Simulation:**

Launch default tunnel world:
```bash
ros2 launch uav_simulation simulate.launch.py
```

Launch simpletunnel world:
```bash
ros2 launch uav_simulation simulate.launch.py world:=simpletunnel
# OR use the dedicated launch file:
ros2 launch uav_simulation simulate_simpletunnel.launch.py
```

**Project Structure:**

- `src/uav_simulation/` - Gazebo simulation package
  - `models/basic_drone/` - Basic quadrotor UAV model (for future use)
  - `worlds/` - Gazebo Fortress world files (SDF format)
    - `tunnel_world.sdf` - Simple rectangular tunnel environment
    - `simpletunnel.sdf` - Tunnel world with Fuel model
  - `launch/` - Launch files
    - `simulate.launch.py` - Launch file with world selection (default: tunnel_world)
    - `simulate_simpletunnel.launch.py` - Dedicated launch file for simpletunnel world
- `src/uav_bringup/` - Launch files and ROS 2 nodes
  - `launch/test_workspace.launch.py` - Test node launch file
  - `src/test_node.cpp` - Simple test node for workspace verification
  - `src/basic_node.cpp` - Basic UAV node (for future use with drone)
- `src/tunnel_controller/` - Control algorithms (to be implemented)
- `src/tunnel_perception/` - Perception and localization (to be implemented)
- `src/msgs/` - Custom message definitions (to be implemented)


3. Development Guide

For detailed instructions on how to develop on this project, see [DEVELOPMENT_GUIDE.md](DEVELOPMENT_GUIDE.md).

**Quick Start for Development:**
```bash
# Daily workflow - run these every time you start working
source /opt/ros/humble/setup.bash
source install/setup.bash

# Make changes to code, then rebuild
colcon build --symlink-install --packages-select <package_name>
source install/setup.bash

# Test your changes
ros2 launch uav_bringup test_workspace.launch.py
```

4. License

This project is licensed under the Apache 2.0 License. See the LICENSE file for full details.
