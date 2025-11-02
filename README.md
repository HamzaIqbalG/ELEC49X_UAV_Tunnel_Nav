ELEC 49X: UAV Self-Navigation in Tunnels

Group 27: Hamza Iqbal, Walker Yee, Muhammad Saad Iqbal


This repository contains the complete ROS 2 software stack for the Queen's University ELEC 490 Capstone Project, "UAV Self-Navigation in Tunnels."

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

ROS 2 Humble (Desktop-Full install): This is essential as it includes Gazebo and RViz.

colcon build tools: sudo apt install python3-colcon-common-extensions

rosdep tool: sudo apt install python3-rosdep

Installation & Build

Create a new workspace:

mkdir -p ~/capstone_ws/src
cd ~/capstone_ws/


Clone the repository:

git clone [https://github.com/](https://github.com/)[YOUR_USERNAME]/[YOUR_REPO_NAME].git src/uav_project


Install Dependencies: rosdep will automatically find all required packages (like Nav2, Gazebo plugins, etc.) defined in the package.xml files.

# Initialize rosdep (only need to do this once)
sudo rosdep init
rosdep update

# Install all dependencies from your workspace root (`~/capstone_ws`)
rosdep install --from-paths src --ignore-src -r -y


Build the workspace:

# From your workspace root (`~/capstone_ws`)
colcon build --symlink-install


3. Usage

All project functionalities are launched using ros2 launch.

Source your workspace:
In every new terminal, you must source the overlay to find your packages.

source ~/capstone_ws/install/setup.bash


Run the MVP Simulation (Geometric Centering):
This launch file starts Gazebo, RViz, Nav2, and our simple "Geometric Centering" node.

# Replace [your_package_name] and [mvp_sim.launch.py] with your actual file names
ros2 launch [your_package_name] mvp_sim.launch.py


Run the Enhanced Simulation (LIO):
This launch file starts the simulation but swaps the simple controller for the advanced LIO package.

# Replace [your_package_name] and [enhanced_lio_sim.launch.py]
ros2 launch [your_package_name] enhanced_lio_sim.launch.py


4. Citing This Project

If you use this work for academic purposes, please use the following citation.

CITATION.cff (Preferred)

A CITATION.cff file is included in this repository for easy import into citation managers like Zotero. GitHub also provides a "Cite this repository" button on the main page.

BibTeX

@software{UAV_Self_Navigation_2025,
  author = {Iqbal, Hamza and Yee, Walker and Iqbal, Muhammad Saad},
  title = {{UAV Self-Navigation in Tunnels: A ROS 2 Implementation}},
  month = {October},
  year = {2025},
  publisher = {GitHub},
  url = {[https://github.com/](https://github.com/)[HamzaIqbalG]/[ELEC49X_UAV_Tunnel_Nav]}
}


5. License

This project is licensed under the Apache 2.0 License. See the LICENSE file for full details.