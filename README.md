# UAV Tunnel Demo (ROS 2 Humble + Gazebo Fortress)

This project builds a sandboxed, local-only simulation: a simple UAV with a
planar LiDAR navigating a straight tunnel in Gazebo Fortress. The UAV hovers
in place for a short time, then moves forward while applying crude, deterministic
wall avoidance based on the LiDAR scan.

## Compatibility and Docs
The stack is intentionally scoped to components with clear compatibility:
- **ROS 2 Humble (Ubuntu 22.04 LTS)**: https://docs.ros.org/en/humble/
- **Gazebo Fortress (gz-sim7)**: https://gazebosim.org/docs/fortress
- **ROS–Gazebo bridge (`ros_gz`)**: https://github.com/gazebosim/ros_gz

This repo includes a minimal UAV model, a Gazebo system plugin for `/cmd_vel`,
and a ROS 2 navigation node that uses `/scan`.

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

## Behavior
- The UAV spawns at the tunnel entrance.
- It "hovers" by holding position for `hover_seconds`.
- It moves forward unless a front obstacle is closer than `safe_distance`.
- It turns away from the closer side wall when blocked.

You can tune speeds and thresholds via ROS parameters in
`uav_tunnel_nav/tunnel_navigator.py`.
