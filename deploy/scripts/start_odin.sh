#!/bin/bash
# Boot-time entry point for robot-odin.service. Must be the FIRST thing up
# in the full stack: Nav2's local costmap needs a valid odom->odin1_base_link
# TF the moment it activates, which only exists once this driver is running.
set -e

# Everything in this stack (ODIN, nav2, costmap, web UI) runs on this one
# Jetson. ROS Jazzy defaults DDS discovery to SUBNET, which ties node
# discovery to the WiFi interface's current subnet -- switching to hotspot
# mode bounces the WiFi interface onto a new subnet and breaks discovery
# between already-running nodes. LOCALHOST discovery is immune to that.
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

source /opt/ros/jazzy/setup.bash
source "$HOME/odin_ws/install/setup.bash"

exec ros2 launch odin_ros_driver odin1_ros2.launch.py
