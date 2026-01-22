# Quick Motor Speed Tuning Guide

## Method 1: Runtime Parameter (Recommended - No Rebuild Needed!)

While the simulation is running, you can change the motor speed instantly:

```bash
# In a new terminal (while simulation is running)
source install/setup.bash

# Try different speeds (adjust the number):
ros2 param set /motor_speed_controller hover_speed 650.0
ros2 param set /motor_speed_controller hover_speed 625.0
ros2 param set /motor_speed_controller hover_speed 600.0
ros2 param set /motor_speed_controller hover_speed 575.0
ros2 param set /motor_speed_controller hover_speed 550.0

# Check current value:
ros2 param get /motor_speed_controller hover_speed
```

**Tips:**
- Too high = drone ascends → reduce the value
- Too low = drone falls → increase the value
- Start with small increments (25-50 rad/s)

## Method 2: Launch Argument (Set at Startup)

```bash
# Launch with custom speed (requires code change to support, or use param file)
ros2 launch uav_simulation simulate.launch.py enable_controller:=true
# Then use Method 1 to adjust
```

## Method 3: Direct Topic Publishing (Quick Test)

For immediate testing without the controller node:

```bash
# In a new terminal
source install/setup.bash

# Publish motor speeds directly (replace 650 with your test value):
ros2 topic pub /X3/gazebo/command/motor_speed actuator_msgs/msg/Actuators \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, position: [], velocity: [650.0, 650.0, 650.0, 650.0], normalized: []}" \
  --once

# Or publish continuously (press Ctrl+C to stop):
ros2 topic pub /X3/gazebo/command/motor_speed actuator_msgs/msg/Actuators \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, position: [], velocity: [650.0, 650.0, 650.0, 650.0], normalized: []}" \
  -r 20
```

## Typical Speed Range

- **Too low (falls)**: < 550 rad/s
- **Hover range**: 550-650 rad/s (depends on environment)
- **Too high (ascends)**: > 650 rad/s

Start at 600 and adjust in 25 rad/s increments.

