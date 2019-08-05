#!/bin/bash
set -e

export TB3_DEMO_DIR=$HOME/tb3_demo_roscon2019
export TB3_UNDERLAY_WS=$TB3_DEMO_DIR/tb3_underlay_ws
export TB3_OVERLAY_WS=$TB3_DEMO_DIR/tb3_overlay_ws 
export TURTLEBOT3_MODEL=burger
export TB3_DEMO_POLICY_FILE="$TB3_DEMO_DIR/policies/tb3_gazebo_policy.xml"
# setup ros2 environment
source "$ROS2_OVERLAY_WS/install/setup.bash"
exec "$@"
