#!/bin/bash
# Boot-time entry point for robot-rear-cam.service. Waits for the rear USB
# webcam to enumerate. pixel_format=mjpeg2rgb is REQUIRED -- without it,
# usb_cam falls back to raw YUYV capture, which on this camera only manages
# ~3.5Hz jittery instead of its real ~27-30Hz MJPEG-mode rate (found live,
# 2026-07-24).
#
# video_device uses the udev by-id symlink, not /dev/videoN -- this camera
# exposes two video nodes (video0=capture, video1=metadata/no-capture-cap)
# and enumeration order is not guaranteed to survive a reboot or another
# USB video device being plugged in. The by-id symlink is keyed off the
# camera's USB serial and always points at the capture-capable node.
#
# CAM_DEVICE below is specific to the exact rear webcam model used on this
# robot -- find yours with `ls /dev/v4l/by-id/` and replace it.
CAM_DEVICE=/dev/v4l/by-id/usb-<YOUR_REAR_CAMERA>-video-index0
set -e

# See start_odin.sh / start_full_behavior.sh: must match their discovery
# range or this node's DDS participant can't be discovered by the rest of
# the stack. Missed here originally (2026-07-25) -- rear_yolo_tracker_node
# was silently receiving zero frames from this node as a result (confirmed
# live: /image_raw showed 0 publishers from a LOCALHOST-scoped query while
# this process was demonstrably running).
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

source /opt/ros/jazzy/setup.bash

for i in $(seq 1 30); do
    if [ -e "$CAM_DEVICE" ]; then
        break
    fi
    sleep 1
done

# Root cause of the above zero-publishers bug (found live, 2026-07-26): it
# wasn't the discovery range -- it was start order. This service starts
# almost immediately at boot, while robot-behavior.service's ~18 DDS
# participants (TensorRT cold cache) can take several minutes to come up.
# Fast-DDS's LOCALHOST discovery range only resends SPDP announcements for
# a short window after participant creation. If this node finishes that
# window before the behavior stack's participants exist, they never
# retroactively discover it and /image_raw is silently never subscribed
# to. Confirmed live: manually restarting this node once the behavior
# stack was already up fixed discovery instantly with no other change.
# Fix: don't start the camera driver until rear_yolo_tracker_node (the
# consumer) already exists, so this node is always the late joiner --
# late joiners were confirmed to discover/be discovered reliably. Falls
# back to starting anyway after 10 minutes so a broken behavior stack
# doesn't permanently strand the camera.
for i in $(seq 1 300); do
    if ros2 node list 2>/dev/null | grep -q '^/rear_yolo_tracker_node$'; then
        break
    fi
    sleep 2
done

# usb_cam_node_exe does not fully resolve a relative-target symlink (it
# mis-joins to a bogus path like /dev/../../video0) -- resolve it ourselves
# so we still self-correct to whatever index the kernel assigns this boot.
RESOLVED_CAM_DEVICE=$(readlink -f "$CAM_DEVICE")

exec ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:="$RESOLVED_CAM_DEVICE" \
  -p framerate:=30.0 \
  -p pixel_format:=mjpeg2rgb \
  -p image_width:=640 \
  -p image_height:=480 \
  -r image_raw:=/image_raw
