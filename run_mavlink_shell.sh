#!/bin/bash

# Helper script to connect to PX4 MAVLink Shell
# PX4 SITL usually exposes the shell on UDP port 14550 (if not used by QGC) or we can connect to the instance directly.

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="$WORKSPACE_DIR/PX4-Autopilot"

# Check if PX4 dir exists here, otherwise check home
if [ ! -d "$PX4_DIR" ]; then
    PX4_DIR="$HOME/PX4-Autopilot"
fi

if [ ! -d "$PX4_DIR" ]; then
    echo "Error: Could not find PX4-Autopilot directory."
    exit 1
fi

echo "Connecting to PX4 MAVLink Shell..."
echo "Note: Ensure the simulation is running first!"
echo "Press Ctrl+C to exit."

# Try connecting to the standard UDP port for SITL
# If QGroundControl is running, it might be hogging 14550.
# We can try connecting to the instance's temp file if it's a local SITL.
# But UDP is the most reliable way for SITL.

python3 "$PX4_DIR/Tools/mavlink_shell.py" 0.0.0.0:14550
