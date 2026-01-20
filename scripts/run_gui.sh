#!/usr/bin/env bash
set -euo pipefail

IMAGE="${1:-uav-tunnel-sim}"

if [ -z "${DISPLAY:-}" ]; then
  echo "DISPLAY is not set. WSLg GUI requires DISPLAY=:0." >&2
  exit 1
fi

if [ ! -d /mnt/wslg ]; then
  echo "WSLg not detected at /mnt/wslg. GUI forwarding will not work." >&2
  exit 1
fi

X11_SRC="/mnt/wslg/.X11-unix"
if [ ! -e "${X11_SRC}/X0" ]; then
  echo "X11 socket not found at ${X11_SRC}/X0. WSLg may not be running." >&2
  exit 1
fi

DOCKER_DEVICE_ARGS=()
DOCKER_ENV_ARGS=(
  --env "DISPLAY=${DISPLAY}"
  --env "WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-}"
  --env "XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir"
  --env "PULSE_SERVER=/mnt/wslg/PulseServer"
  --env "QT_X11_NO_MITSHM=1"
  --env "QT_QPA_PLATFORM=xcb"
)

if [ -e /dev/dri ]; then
  DOCKER_DEVICE_ARGS+=(--device=/dev/dri)
else
  DOCKER_ENV_ARGS+=(--env "LIBGL_ALWAYS_SOFTWARE=1")
fi

docker run --rm -it \
  "${DOCKER_ENV_ARGS[@]}" \
  --volume "${X11_SRC}:/tmp/.X11-unix:rw" \
  --volume "/mnt/wslg:/mnt/wslg:rw" \
  "${DOCKER_DEVICE_ARGS[@]}" \
  "${IMAGE}"
