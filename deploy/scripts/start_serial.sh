#!/bin/bash
# Boot-time entry point for robot-serial.service. Waits for the motor
# ESP32's USB serial port to enumerate (boot-time USB detection can lag
# service start) -- serial_driver cannot start at all without it.
set -e

# See start_odin.sh / start_full_behavior.sh / start_rear_cam.sh: must match
# their discovery range or this node's DDS participant can't be discovered
# by the rest of the stack. Missed here too (2026-07-25) -- serial_driver
# was silently NOT receiving /cmd_vel from drive_mux_node as a result
# (confirmed live: commanding a real /manual_cmd_vel Twist produced zero
# odometry change, and serial_driver's own log showed it continuously
# sending L=0.0 R=0.0 regardless). This is the actual root cause of "no
# behavior change" reports for BOTH follow and flee testing -- the whole
# autonomous/manual driving pipeline was disconnected from the motors, not
# a bearing-sign or DWB-tuning problem.
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

source /opt/ros/jazzy/setup.bash
source "$HOME/autonomous-tank-system/ros2_ws/install/setup.bash"

for i in $(seq 1 30); do
    if [ -e /dev/ttyACM0 ]; then
        break
    fi
    sleep 1
done

exec ros2 run motor_controller serial_driver
