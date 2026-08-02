#!/bin/bash
# Boot-time entry point for robot-behavior.service: the full Nav2 + YOLO
# tracker + web control UI stack (robot_behavior nav2_bringup.launch.py,
# which now also launches web_ui_bridge_node and costmap_to_image_node).
#
# Waits for ODIN to actually be streaming odometry -- not just started, the
# device handshake alone takes ~10-15s (see robot-odin.service) -- before
# launching. Nav2's local costmap checks for a valid odom->odin1_base_link
# TF immediately on activation; launching before ODIN is truly up just
# produces a noisy (non-fatal) transform-timeout warning instead of a clean
# start.
set -e

# See start_odin.sh: force LOCALHOST discovery so switching to hotspot mode
# (which bounces the WiFi interface onto a new subnet) doesn't break DDS
# discovery between the already-running nodes in this stack.
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

source /opt/ros/jazzy/setup.bash
source "$HOME/odin_ws/install/setup.bash"
source "$HOME/autonomous-tank-system/ros2_ws/install/setup.bash"

for i in $(seq 1 60); do
    if ros2 topic list 2>/dev/null | grep -q '^/odin1/odometry$'; then
        break
    fi
    sleep 1
done

exec ros2 launch robot_behavior nav2_bringup.launch.py
