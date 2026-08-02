# Legacy: ArUco marker tracker

**Superseded by the YOLO11n/TensorRT person trackers** in
[`ros2_ws/src/robot_behavior`](../../ros2_ws/src/robot_behavior)
(`odin_yolo_tracker_node`, `rear_yolo_tracker_node`). Kept here, archived
rather than deleted, because it documents an early real phase of the
project and the reasoning for the swap: marker tracking required the
target to wear/hold a printed marker and lose lock immediately outside a
narrow viewing angle, whereas the YOLO trackers detect a person directly
with no cooperating marker needed. See
[`PHASE6_HANDOFF_REPORT.md`](../../docs/PHASE6_HANDOFF_REPORT.md) for the
mode model and system architecture from that era, when ArUco tracking was
still the active perception method.

This code is not built or run as part of the current stack and is not
wired into `deploy/`. It's a standalone `aruco_tracker` ROS 2 package plus
the `usb_cam` params and printed marker image it was tested against.
