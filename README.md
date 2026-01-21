# UAV Tunnel Demo (ROS 2 Humble + Gazebo Fortress)

This project builds a sandboxed, local-only simulation: a simple UAV with a
planar LiDAR navigating a straight tunnel in Gazebo Fortress. The UAV takes off
briefly, then moves forward while applying crude, deterministic wall avoidance
based on the LiDAR scan.

## Compatibility and Docs
The stack is intentionally scoped to components with clear compatibility:
- **ROS 2 Humble (Ubuntu 22.04 LTS)**: https://docs.ros.org/en/humble/
- **Gazebo Fortress (gz-sim7)**: https://gazebosim.org/docs/fortress
- **ROS–Gazebo bridge (`ros_gz`)**: https://github.com/gazebosim/ros_gz
- **Aerostack2 Gazebo Assets (`as2_gazebo_assets`)**: https://docs.ros.org/en/humble/p/as2_gazebo_assets/

This repo includes a minimal UAV model, a Gazebo system plugin for `/cmd_vel`,
and a ROS 2 navigation node that uses `/scan`.

From the Aerostack2 docs, `as2_gazebo_assets` is tested on **Gazebo Fortress**
and should match your ROS 2 version. We use the Humble package to stay
compatible with this stack.

## Sandboxed Environment (Docker)
All installations happen inside a container. The host system remains unchanged.

### Build the image
```bash
cd /home/saad/UAV-Tunnel-V2
docker build -t uav-tunnel-sim -f docker/Dockerfile .
```

If you see DNS resolution errors during the build, retry with host networking:
```bash
docker build --network=host -t uav-tunnel-sim -f docker/Dockerfile .
```

### Run the container
**Linux (X11):**
```bash
xhost +local:docker
docker run --rm -it \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  uav-tunnel-sim
```

**WSLg (Windows Subsystem for Linux):** use the same command above; WSLg
already exposes the display socket.

If you prefer docker compose:
```bash
docker compose run --rm uav-sim
```

## Build and Run (inside the container)
```bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch uav_tunnel_nav bringup.launch.py
```

If you're running inside WSL2 Docker and don't have GUI forwarding,
use the default headless mode. To force the GUI (when display is available):
```bash
ros2 launch uav_tunnel_nav bringup.launch.py headless:=false
```

## WSL2 + Docker GUI (Gazebo)
WSLg already provides X11/Wayland sockets; you must pass them into the container.
Use the helper script from the host:
```bash
./scripts/run_gui.sh
```

If you prefer the raw command, it must mount the WSLg sockets:
```bash
docker run --rm -it \
  --env="DISPLAY=$DISPLAY" \
  --env="WAYLAND_DISPLAY=$WAYLAND_DISPLAY" \
  --env="XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir" \
  --env="PULSE_SERVER=/mnt/wslg/PulseServer" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="QT_QPA_PLATFORM=xcb" \
  --volume="/mnt/wslg/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="/mnt/wslg:/mnt/wslg:rw" \
  uav-tunnel-sim
```

Inside the container:
```bash
cd /ros2_ws
source install/setup.bash
ros2 launch uav_tunnel_nav bringup.launch.py headless:=false
```

To start RViz2 with the LiDAR display:
```bash
ros2 launch uav_tunnel_nav bringup.launch.py headless:=false rviz:=true
```

## Project Layout
- `docker/`: sandbox image and entrypoint
- `ros2_ws/src/uav_tunnel_gz`: Gazebo Fortress world, UAV model, cmd_vel plugin
- `ros2_ws/src/uav_tunnel_nav`: ROS 2 node + launch to run sim and bridge

## Model and Sensors
- The UAV is a quadcopter-style model with body, arms, rotors, and an IMU.
- A 2D GPU LiDAR is mounted on a dedicated `lidar_link`.
- The LiDAR scan is republished as `/scan_fixed` with `frame_id=base_link`
  to make RViz visualization reliable.
 - Additional sensors in the AS2 configuration: barometer (`air_pressure`),
   magnetometer, and a downward-facing VGA camera for basic VIO wiring.

## Aerostack2 Quadcopter Option
Aerostack2 provides **Fortress-compatible** drone and sensor models. The assets
include the `quadrotor_base` model and a `planar_lidar` payload. The quadrotor
SDF template enables the Gazebo multicopter velocity controller, so publishing
to `cmd_vel` remains valid.

To use the Aerostack2 model in the tunnel:
```bash
ros2 launch uav_tunnel_nav bringup_as2.launch.py headless:=false rviz:=true
```

Key topics when using Aerostack2:
- Command: `/gz/uav0/cmd_vel`
- LiDAR scan: `/uav0/sensor_measurements/lidar/scan` (republished as `/scan_fixed`)
 - IMU: `/uav0/sensor_measurements/imu`
 - Magnetometer: `/uav0/sensor_measurements/magnetometer`
 - Barometer: `/uav0/sensor_measurements/air_pressure`
 - Basic odometry: `/uav0/basic_odom`

## Behavior
- The UAV spawns at the tunnel entrance.
- It takes off for `takeoff_seconds` using a vertical velocity command.
- It moves forward unless a front obstacle is closer than `safe_distance`.
- It turns away from the closer side wall when blocked.

You can tune speeds and thresholds via ROS parameters in
`uav_tunnel_nav/tunnel_navigator.py`.

Key navigation parameters (can be overridden via launch):
- `takeoff_seconds`: duration of upward velocity
- `takeoff_speed`: upward velocity used for takeoff
 - `enable_arm`: enable arming via `/gz/uav0/arm`
 - `arm_seconds`: duration to publish the arm signal before takeoff
- `center_gain`: yaw gain to keep the drone centered in the corridor
- `center_deadband`: ignore small left/right differences (meters)

LiDAR adjustments:
- The local `planar_lidar` model overrides the AS2 asset to provide a 360° scan.
- The LiDAR payload is mounted slightly below the quadcopter (`z = -0.05`).

Basic odometry:
- `basic_odometry` composes a simple odometry estimate without EKF fusion.
- Uses LiDAR for x/y positioning (front wall + corridor centering),
  barometer for altitude, IMU for roll/pitch, and magnetometer for yaw.
- Publishes TF `odom -> base_link` and `nav_msgs/Odometry` on `/uav0/basic_odom`.
