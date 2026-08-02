#!/bin/bash
# Boot-time entry point for robot-manual.service -- sources ROS + the one
# combined workspace manual mode actually needs (motor_controller for
# serial_driver, robot_behavior for everything else; ODIN/legacy aruco are
# NOT needed here, see manual_mode.launch.py), waits for the USB devices to
# enumerate (boot-time USB detection can lag service start), then launches.
set -e

source /opt/ros/jazzy/setup.bash
source "$HOME/autonomous-tank-system/ros2_ws/install/setup.bash"

# Wait up to 30s for the motor ESP32 and joystick to show up. If the
# joystick isn't paired/connected yet, joy_node will just sit with no data
# (safe -- no /manual_cmd_vel means drive_mux outputs zero) so we only hard-
# wait on the serial port, which serial_driver cannot start without at all.
for i in $(seq 1 30); do
    if [ -e /dev/ttyACM0 ]; then
        break
    fi
    sleep 1
done

exec ros2 launch robot_behavior manual_mode.launch.py
