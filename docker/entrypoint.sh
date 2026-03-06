#!/usr/bin/env bash
set -e

# Fix ArduPilot git "dubious ownership" so sim_vehicle.py can build (git rev-parse).
# Required when repo is bind-mounted or owned by different UID.
git config --global --add safe.directory /ardu_ws/src/ardupilot 2>/dev/null || true

# ── One-time rebuild of ArduCopter SITL without DDS ──────────────────
# The Docker image ships an ArduCopter binary compiled WITH DDS, which
# spams "DDS: No ping response" every few seconds.  We reconfigure and
# rebuild without DDS.  A named Docker volume persists the build dir so
# this only runs once (typically 2-5 min).
_ap_marker=/ardu_ws/src/ardupilot/build/.no_dds_built
if [ ! -f "$_ap_marker" ]; then
  echo "[entrypoint] Rebuilding ArduCopter SITL without DDS (one-time, ~2-5 min) ..."
  _dds_gen=/home/ardupilot/.local/bin/microxrceddsgen
  [ -f "$_dds_gen" ] && mv "$_dds_gen" "${_dds_gen}.disabled"
  cd /ardu_ws/src/ardupilot
  ./waf configure --board sitl 2>&1
  ./waf copter -j"$(nproc)" 2>&1
  [ -f "${_dds_gen}.disabled" ] && mv "${_dds_gen}.disabled" "$_dds_gen"
  touch "$_ap_marker"
  echo "[entrypoint] ArduCopter SITL rebuilt without DDS."
  cd /ros2_ws
fi

source /opt/ros/humble/setup.bash

if [ -f /ardu_ws/install/setup.bash ]; then
  source /ardu_ws/install/setup.bash
fi

if [ -f /gz_ws/install/setup.bash ]; then
  source /gz_ws/install/setup.bash
fi

export GZ_VERSION="${GZ_VERSION:-garden}"
export GZ_SIM_RESOURCE_PATH="/gz_ws/src/ardupilot_gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

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
