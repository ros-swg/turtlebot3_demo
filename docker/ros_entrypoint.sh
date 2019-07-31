#!/bin/bash
set -e

# setup ros2 environment
source "$ROS2_OVERLAY_WS/install/setup.bash"
exec "$@"
