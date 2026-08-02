# Phase 7 bringup: Phase 6 behaviour layer + Nav2 (local-only) integration.
#
# USAGE (same prerequisites as bringup.launch.py — start these first):
#     ros2 launch odin_ros_driver odin1_ros2.launch.py     # ODIN: odom, clouds
#     ros2 run motor_controller serial_driver              # consumes /cmd_vel
#   then:
#     ros2 launch robot_behavior nav2_bringup.launch.py
#
# What changed vs bringup.launch.py:
#   - ADDED: controller_server (DWB + local costmap from /odin1/cloud_slam),
#            velocity_smoother, lifecycle_manager, and intent_to_goal_node.
#   - target_pid_node is NOT launched here. Its old direct-velocity role is
#     replaced by Nav2. It stays in the package as a documented fallback (revert
#     by relaunching bringup.launch.py, or by setting drive_mux's
#     auto_source_topic:=/cmd_vel_pid and running target_pid). Only ONE
#     autonomous velocity source drives at a time.
#   - Phase 7.1 ADDED then REMOVED same session (2026-07-14): collision_monitor
#     as a downstream stop/slowdown safety net. Removed because ODIN's point
#     cloud timestamps run on the sensor's own hardware clock, tens of minutes
#     offset from ROS time, which made collision_monitor's staleness check
#     permanently distrust live data and silently withhold ALL /cmd_vel output.
#     User's explicit call: not worth it — the PS-button e-stop and a physical
#     kill switch are the real safety net. drive_mux publishes /cmd_vel
#     directly again. See drive_mux_node.py CMD_VEL_OUT_TOPIC comment.
#   - Phase 7.1 ADDED (kept): intent_to_goal now also accepts the JOYSTICK as
#     a goal source in semi mode when no marker is detected (projects a goal
#     from the commanded Twist, DWB routes around obstacles to reach it) —
#     see intent_to_goal_node.py module docstring.
#   - Phase 7.1 ADDED (kept): R2 deadman gate for auto/semi. intent_to_goal
#     and DWB keep computing goals/routes regardless (visible on the costmap
#     viewer at all times), but drive_mux only lets that output reach the
#     motors while R2 (/nav_enable, from ps_controller_bridge) is held. See
#     drive_mux_node.py NAV_ENABLE_STALE_TIMEOUT_SEC comment. Manual is not
#     gated (no goal/autonomy concept there; releasing the stick already
#     stops it).
#
# Pipeline (auto/semi):
#   intent_to_goal --FollowPath--> controller_server --> /nav2/cmd_vel_raw
#     --> velocity_smoother --> /nav2/cmd_vel_smoothed --> drive_mux
#     [gated on /nav_enable, i.e. R2 held] --> /cmd_vel
# Pipeline (manual):
#   ps_controller_bridge --> /manual_cmd_vel --> drive_mux --> /cmd_vel
# Manual is NOT velocity-smoothed (raw controller feel is intentional).
#
# Phase 8 (2026-07-24): added web_ui_bridge_node (package robot_web_ui) —
# serves the browser control/telemetry UI on :8088, republishes the Steam
# Deck's browser Gamepad API state as /joy (replacing joy_node as
# ps_controller_bridge's input source; see web_ui_bridge_node's own comment
# below), and streams both camera feeds + all telemetry over WebSocket/MJPEG.
# Also added the /target_bbox -> /rear_target_bbox remap on rear_yolo_tracker
# (front tracker's /odin_target_bbox needs no remap) for the web UI's live
# bounding-box overlay.
#
# Target state estimator + behavior state machine refactor (see
# target_state_estimator_node.py / behavior_state_machine_node.py module
# docstrings for the full rationale):
#   - target_selector_node is RETIRED from this launch file -- NOT deleted,
#     documented fallback only, same pattern as target_pid/the old ArUco
#     trackers above. Its mode-based-selection job is absorbed by
#     target_state_estimator_node, which also fixes the core "goal computed
#     from the robot's own current, possibly-stale pose" bug and adds
#     world-frame filtering + persistence telemetry.
#   - ADDED target_state_estimator_node (subscribes to both trackers' new
#     stamped /*_target_detection_raw topics + track_mode_state; publishes
#     the same /target_detected /target_bearing /target_distance
#     target_selector used to, so intent_to_goal/target_pid need no changes,
#     plus a richer /target_state for the state machine below).
#   - ADDED behavior_state_machine_node -- owns persistence/search/recovery
#     (TRACKING/COASTING/SEARCHING/RECOVERING/LOST). It now sits between
#     Nav2 and drive_mux: drive_mux's auto_source_topic is changed from
#     /nav2/cmd_vel_smoothed directly to this node's /behavior/cmd_vel
#     output (which passes /nav2/cmd_vel_smoothed straight through during
#     normal TRACKING/COASTING and substitutes its own search/recovery
#     Twist otherwise). intent_to_goal/DWB/velocity_smoother are UNCHANGED.
#   - The rear tracker's new /target_detection_raw native topic gets the
#     same /rear_target_* remap treatment as its other /target_* topics.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_behavior')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # ---- Phase 8 behaviour nodes (unchanged, minus target_pid) ----
    # Phase 8 (2026-07-24): ArUco marker tracking replaced with YOLO11n
    # (TensorRT FP16 engine, GPU) person detection on both cameras. The old
    # aruco_tracker (rear) and odin_target_tracker (front) nodes are retired
    # from this launch file -- NOT deleted from the package/aruco_ws, kept as
    # a documented fallback only, same pattern already used for target_pid
    # vs. Nav2 above. Do not launch both the aruco and yolo trackers at once:
    # they publish the exact same topic names and would collide.
    rear_yolo_tracker = Node(
        package='robot_behavior',
        executable='rear_yolo_tracker_node',
        name='rear_yolo_tracker_node',
        output='screen',
        remappings=[
            ('/target_detected', '/rear_target_detected'),
            ('/target_bearing', '/rear_target_bearing'),
            ('/target_distance', '/rear_target_distance'),
            ('/target_bbox', '/rear_target_bbox'),
            ('/target_detection_raw', '/rear_target_detection_raw'),
        ],
    )
    odin_target_tracker = Node(
        package='robot_behavior', executable='odin_yolo_tracker_node',
        name='odin_yolo_tracker_node', output='screen')
    mode_manager = Node(
        package='robot_behavior', executable='mode_manager_node',
        name='mode_manager_node', output='screen')
    # target_selector_node RETIRED from this launch (see file header comment)
    # -- superseded by target_state_estimator_node below, which absorbs its
    # mode-based selection job. Kept in the package as a documented fallback.
    target_state_estimator = Node(
        package='robot_behavior', executable='target_state_estimator_node',
        name='target_state_estimator_node', output='screen')
    behavior_state_machine = Node(
        package='robot_behavior', executable='behavior_state_machine_node',
        name='behavior_state_machine_node', output='screen')
    # Phase 8 (2026-07-24): joy_node (local, physically-paired PS controller)
    # is RETIRED from this launch -- NOT deleted, same documented-fallback
    # pattern used elsewhere in this file (revert by adding it back + running
    # robot_web_ui's web_ui_node separately, or just removing web_ui_bridge
    # below). Its role as ps_controller_bridge's /joy source is now filled by
    # web_ui_bridge_node, which republishes the Steam Deck's browser Gamepad
    # API state (relayed over WebSocket) as synthetic /joy messages —
    # ps_controller_bridge_node itself is UNCHANGED, per the "replace the
    # input source only" instruction. See robot_web_ui/web_ui_node.py for the
    # button/axis index translation table (browser "standard gamepad"
    # mapping -> the exact array slots ps_controller_bridge already expects).
    web_ui_bridge = Node(
        package='robot_web_ui', executable='web_ui_node',
        name='web_ui_bridge_node', output='screen')
    ps_controller_bridge = Node(
        package='robot_behavior', executable='ps_controller_bridge_node',
        name='ps_controller_bridge_node', output='screen')
    drive_mux = Node(
        package='robot_behavior', executable='drive_mux_node',
        name='drive_mux_node', output='screen',
        # auto_source_topic changed from /nav2/cmd_vel_smoothed directly to
        # behavior_state_machine_node's arbitrated output -- see that node's
        # module docstring. It passes Nav2's smoothed output straight through
        # during normal TRACKING/COASTING and substitutes its own
        # search/recovery Twist otherwise, so drive_mux itself needs no
        # other changes (still just one autonomous source, per the existing
        # "only ONE autonomous velocity source" rule).
        parameters=[{'auto_source_topic': '/behavior/cmd_vel'}])
    safety_watchdog = Node(
        package='robot_behavior', executable='safety_watchdog_node',
        name='safety_watchdog_node', output='screen')

    # ---- Nav2 (local-only) ----
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
        # Keep Nav2's cmd_vel OFF the real /cmd_vel bus; route it through the
        # smoother first.
        remappings=[('cmd_vel', '/nav2/cmd_vel_raw')],
    )
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params],
        remappings=[
            ('cmd_vel', '/nav2/cmd_vel_raw'),
            # nav2_velocity_smoother's output topic is internally named
            # 'cmd_vel_smoothed' (NOT 'smoothed_cmd_vel' — verified live).
            ('cmd_vel_smoothed', '/nav2/cmd_vel_smoothed'),
        ],
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[nav2_params],
    )
    intent_to_goal = Node(
        package='robot_behavior', executable='intent_to_goal_node',
        name='intent_to_goal_node', output='screen')

    # Existing Phase 7 diagnostic node (costmap_to_image_node.py), previously
    # only run ad hoc alongside web_video_server for manual rviz-less
    # debugging — now wired into normal bringup because the web UI's costmap
    # panel streams its output directly (see robot_web_ui/ros_bridge.py).
    costmap_to_image = Node(
        package='robot_behavior', executable='costmap_to_image_node',
        name='costmap_to_image_node', output='screen')

    return LaunchDescription([
        rear_yolo_tracker,
        odin_target_tracker,
        mode_manager,
        target_state_estimator,
        behavior_state_machine,
        web_ui_bridge,
        ps_controller_bridge,
        drive_mux,
        safety_watchdog,
        controller_server,
        velocity_smoother,
        lifecycle_manager,
        intent_to_goal,
        costmap_to_image,
    ])
