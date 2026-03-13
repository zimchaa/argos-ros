#!/usr/bin/env bash
# Start a virtual framebuffer + VNC server inside the Docker container.
# Connect from host with any VNC client to localhost:5900.
#
# Usage (inside container):
#   ./start-vnc.sh                          # just VNC, then run commands manually
#   ./start-vnc.sh ros2 launch argos_description display.launch.py  # VNC + launch

set -e

DISPLAY=:99
export DISPLAY

# Start Xvfb (virtual framebuffer) on display :99
Xvfb $DISPLAY -screen 0 1280x800x24 &
sleep 1

# Start x11vnc listening on port 5900
mkdir -p ~/.vnc
x11vnc -storepasswd argos ~/.vnc/passwd
x11vnc -display $DISPLAY -forever -rfbauth ~/.vnc/passwd -quiet &
sleep 0.5

echo "VNC server running — connect to localhost:5900"

# If arguments were passed, run them (e.g. ros2 launch ...)
if [ $# -gt 0 ]; then
    exec "$@"
else
    exec bash
fi
