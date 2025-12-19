#!/bin/bash

# Setup script for PX4 Autopilot and MicroXRCEAgent
# This script installs dependencies and builds PX4 for SITL usage.

set -e

echo "============================================================"
echo "Starting PX4 Autopilot Setup"
echo "This process may take 20-30 minutes depending on internet speed"
echo "============================================================"

# 1. Install dependencies
echo "Installing dependencies..."
sudo apt-get update
sudo apt-get install -y git python3-pip python3-colcon-common-extensions

# 2. Clone PX4-Autopilot
if [ -d "PX4-Autopilot" ] && [ -z "$(ls -A PX4-Autopilot)" ]; then
    echo "Deleting empty PX4-Autopilot Directory"
    rmdir PX4-Autopilot
fi

if [ ! -d "PX4-Autopilot" ]; then
    echo "Cloning PX4-Autopilot (v1.14)..."
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive -b release/1.14
else
    echo "PX4-Autopilot already exists. Skipping clone."
fi
 
    
# 3. Install PX4 dependencies
echo "Installing PX4 dependencies (this may ask for sudo password)..."
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh

# 4. Build PX4 SITL
echo "Building PX4 SITL..."
make px4_sitl_default

# 5. Install MicroXRCEAgent (ROS 2 Bridge)
cd ..
if [ -d "Micro-XRCE-DDS-Agent" ] && [ -z "$(ls -A Micro-XRCE-DDS-Agent)" ]; then
    echo "Deleting empty Micro-XRCE-DDS-Agent Directory"
    rmdir Micro-XRCE-DDS-Agent
fi
if [ ! -d "Micro-XRCE-DDS-Agent" ]; then
    echo "Cloning Micro-XRCE-DDS-Agent..."
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    cd Micro-XRCE-DDS-Agent
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    sudo ldconfig /usr/local/lib/
    cd ../..
else
    echo "Micro-XRCE-DDS-Agent already exists. Skipping."
fi

echo "============================================================"
echo "PX4 Setup Complete!"
echo "You may need to restart your computer for group permissions to take effect."
echo "============================================================"
