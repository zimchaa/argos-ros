#!/bin/bash
# Launch calibration session from atlas-controller.
#
# Starts robot-side nodes via SSH, then local nodes for ArUco tracking + control.
# Camera is expected to be plugged into atlas-controller.
#
# Usage:
#   ./scripts/calibrate.sh
#   ./scripts/calibrate.sh --no-robot   # skip SSH, robot nodes already running

set -e

ROBOT_HOST="${ROBOT_HOST:-zimchaa@argos-ros.local}"
ROBOT_WS="${ROBOT_WS:-/home/zimchaa/Projects/Argos/argos-ros}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

ROBOT_PID=""

cleanup() {
    echo ""
    echo "Shutting down..."
    # Kill the SSH background process (which kills remote nodes)
    if [ -n "$ROBOT_PID" ] && kill -0 "$ROBOT_PID" 2>/dev/null; then
        echo "Stopping robot nodes..."
        kill "$ROBOT_PID" 2>/dev/null || true
        wait "$ROBOT_PID" 2>/dev/null || true
    fi
    # Kill any local background jobs
    jobs -p | xargs -r kill 2>/dev/null || true
    echo "Done."
}
trap cleanup EXIT INT TERM

# --- Robot side ---
if [ "$1" != "--no-robot" ]; then
    echo "Starting robot nodes on ${ROBOT_HOST}..."
    ssh -tt "$ROBOT_HOST" "
        cd ${ROBOT_WS} &&
        source /opt/ros/humble/setup.bash &&
        source install/setup.bash &&
        ros2 launch argos_bringup robot_calibration.launch.py
    " &
    ROBOT_PID=$!
    echo "Robot nodes started (PID $ROBOT_PID)"
    sleep 3  # let robot nodes initialise before starting local nodes
else
    echo "Skipping robot SSH (--no-robot)"
fi

# --- Controller side ---
echo "Starting local nodes (camera + aruco + estimator + rviz)..."
source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"

# Launch aruco_viz with camera enabled (camera is local)
ros2 launch argos_bringup aruco_viz.launch.py camera:=true &
LOCAL_PID=$!
sleep 2

# Control panel in foreground (needs terminal input)
echo ""
echo "====================================="
echo " Control panel starting..."
echo " Use I/K O/L P/; to position arm"
echo " Press C to calibrate zero"
echo " Press Q to quit"
echo "====================================="
echo ""
ros2 run argos_hardware control_panel

# When control panel exits, cleanup runs via trap
