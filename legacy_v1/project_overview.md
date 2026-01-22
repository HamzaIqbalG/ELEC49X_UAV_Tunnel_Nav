# Project Overview: ELEC49X UAV Tunnel Navigation (PX4 Edition)

This project uses **PX4 Autopilot** for flight control and **ROS 2 Humble** for high-level decision making.

## 1. Project Structure

```
ELEC49X_UAV_Tunnel_Nav/
├── src/
│   ├── uav_simulation/          # Gazebo Simulation + Launch Files
│   ├── uav_bringup/             # ROS 2 Nodes (Offboard Control)
│   └── px4_msgs/                # ROS 2 Messages for PX4
│
├── PX4-Autopilot/               # PX4 Flight Controller Source
└── setup_px4.sh                 # Setup script
```

## 2. Architecture

*   **Simulator**: Gazebo Fortress (Ignition)
*   **Flight Controller**: PX4 Autopilot (SITL) running `x500` model.
*   **Bridge**: `MicroXRCEAgent` connects PX4 (uORB) to ROS 2 (DDS).
*   **Control**: `offboard_control` node sends trajectory setpoints to PX4.

## 3. How to Run

### Step 1: Build (First Time / After Changes)
```bash
colcon build --symlink-install
source install/setup.bash
```

### Step 2: Launch Simulation
This launches Gazebo, PX4 SITL, and the MicroXRCEAgent.
```bash
ros2 launch uav_simulation simulate.launch.py
```
*Wait for the drone to spawn and PX4 to initialize.*

### Step 3: Run Offboard Control
In a new terminal:
```bash
source install/setup.bash
ros2 run uav_bringup offboard_control
```
**What happens:**
1.  Node sends "Offboard Mode" signal.
2.  Node Arms the drone.
3.  Drone takes off and hovers at 5 meters.

## 4. Troubleshooting

*   **"PX4 binary not found"**: Run `./setup_px4.sh` and rebuild.
*   **"MicroXRCEAgent not found"**: Ensure it's installed (done by setup script).
*   **Drone doesn't take off**: Ensure `offboard_control` is running and sending setpoints (> 10Hz).
