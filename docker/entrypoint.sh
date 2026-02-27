#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

if [ -f /ardu_ws/install/setup.bash ]; then
  source /ardu_ws/install/setup.bash
fi

if [ -f /gz_ws/install/setup.bash ]; then
  source /gz_ws/install/setup.bash
fi

export GZ_VERSION="${GZ_VERSION:-garden}"

# Incremental rebuild of project packages when source is bind-mounted.
# Compares source mtimes against the last build marker; rebuilds only
# the two project packages (fast: typically < 10 s).
_marker=/ros2_ws/.last_build
_need_rebuild=0
if [ ! -f "$_marker" ]; then
  _need_rebuild=1
elif [ -n "$(find /ros2_ws/src/uav_tunnel_gz /ros2_ws/src/uav_tunnel_nav \
              -newer "$_marker" -name '*.py' -o -newer "$_marker" -name '*.xml' \
              -o -newer "$_marker" -name '*.yaml' -o -newer "$_marker" -name '*.sdf' \
              -o -newer "$_marker" -name '*.cmake' -o -newer "$_marker" -name 'CMakeLists.txt' \
              2>/dev/null | head -1)" ]; then
  _need_rebuild=1
fi

if [ "$_need_rebuild" -eq 1 ]; then
  echo "[entrypoint] Rebuilding project packages ..."
  cd /ros2_ws
  colcon build --packages-select uav_tunnel_gz uav_tunnel_nav 2>&1
  touch "$_marker"
  echo "[entrypoint] Rebuild complete."
fi

if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi

exec "$@"
