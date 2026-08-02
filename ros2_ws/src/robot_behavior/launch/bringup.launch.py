# USAGE:
#   Source aruco_ws, odin_ws, motor_ws, then behavior_ws (each overlaying the
#   last), then:
#     ros2 launch robot_behavior bringup.launch.py
#
#   This launch file does NOT start odin_ros_driver or motor_controller's
#   serial_driver — those are separate, already-running workspaces this
#   layer only subscribes to. Start them yourself beforehand:
#     ros2 launch odin_ros_driver odin1_ros2.launch.py
#     ros2 run motor_controller serial_driver
#
#   Phase 8 (2026-07-24): ArUco marker tracking replaced with YOLO11n
#   (TensorRT FP16 engine, GPU) person detection on both cameras. This
#   launches rear_yolo_tracker_node (rear webcam) instead of aruco_ws's
#   aruco_tracker_node, with its output topics remapped from
#   /target_detected /target_bearing /target_distance to
#   /rear_target_detected /rear_target_bearing /rear_target_distance --
#   same remap aruco_tracker_node used, kept identical so target_selector
#   needs no changes. This avoids a topic collision with target_selector's
#   unified output, which republishes on the original /target_detected
#   /target_bearing /target_distance names.
#   The old aruco_tracker (rear) and odin_target_tracker (front) nodes are
#   retired from this launch file -- NOT deleted, kept in aruco_ws /
#   robot_behavior as a documented fallback only. Do not launch both the
#   aruco and yolo trackers at once: they publish the exact same topics.
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
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
        ],
    )

    odin_target_tracker = Node(
        package='robot_behavior',
        executable='odin_yolo_tracker_node',
        name='odin_yolo_tracker_node',
        output='screen',
    )

    mode_manager = Node(
        package='robot_behavior',
        executable='mode_manager_node',
        name='mode_manager_node',
        output='screen',
    )

    target_selector = Node(
        package='robot_behavior',
        executable='target_selector_node',
        name='target_selector_node',
        output='screen',
    )

    target_pid = Node(
        package='robot_behavior',
        executable='target_pid_node',
        name='target_pid_node',
        output='screen',
    )

    # Phase 8: joy_node (local, physically-paired PS controller) retired in
    # favor of the Steam Deck's browser Gamepad API, relayed as synthetic
    # /joy messages by web_ui_bridge_node — see nav2_bringup.launch.py's
    # Phase 8 comment for the full explanation. Same swap applied here for
    # consistency, since this file is kept as a documented fallback.
    web_ui_bridge = Node(
        package='robot_web_ui',
        executable='web_ui_node',
        name='web_ui_bridge_node',
        output='screen',
    )

    ps_controller_bridge = Node(
        package='robot_behavior',
        executable='ps_controller_bridge_node',
        name='ps_controller_bridge_node',
        output='screen',
    )

    drive_mux = Node(
        package='robot_behavior',
        executable='drive_mux_node',
        name='drive_mux_node',
        output='screen',
    )

    safety_watchdog = Node(
        package='robot_behavior',
        executable='safety_watchdog_node',
        name='safety_watchdog_node',
        output='screen',
    )

    return LaunchDescription([
        rear_yolo_tracker,
        odin_target_tracker,
        mode_manager,
        target_selector,
        target_pid,
        web_ui_bridge,
        ps_controller_bridge,
        drive_mux,
        safety_watchdog,
    ])
