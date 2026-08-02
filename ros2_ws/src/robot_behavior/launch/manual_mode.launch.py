# Minimal manual-mode-only bringup — 2026-07-15, for systemd auto-start on
# boot (see /etc/systemd/system/robot-manual.service).
#
# Deliberately NOT the full stack: manual mode's actual data path
# (ps_controller_bridge -> drive_mux -> /cmd_vel -> serial_driver) never
# touches TF, odometry, or the costmap, so ODIN and the whole Nav2/
# intent_to_goal/target-tracking chain are genuinely unnecessary here — they
# only matter for semi/auto. Leaving them out means a faster, simpler boot
# with fewer things that can fail. For semi-auto testing, bring up
# odin_ros_driver + nav2_bringup.launch.py separately, same as every session
# so far — this launch file does not replace that, it's boot-time manual-only.
#
# Safe to auto-start unattended: mode_manager's drive_source defaults to
# 'manual' (see mode_constants.py DEFAULT_DRIVE_SOURCE), and manual mode only
# ever produces a nonzero /cmd_vel when the joystick itself sends a nonzero
# command — nothing drives itself the instant power comes on.
#
# speed_scale defaults to 0.5 (50%) HERE ONLY, as a launch argument — this
# does NOT change serial_driver's own coded default (still the conservative
# 0.15 for plain `ros2 run motor_controller serial_driver` with no args).
# Was briefly 1.0 (100%) per an earlier explicit request, capped back down to
# 0.5 2026-07-15 (also explicit) after testing at full power. Override at
# launch: `ros2 launch robot_behavior manual_mode.launch.py speed_scale:=0.15`.
#
# joy_node's autorepeat_rate is explicitly set to 0.0 (disabling the
# ROS-level default of 20Hz) — see drive_mux_node.py INPUT_STALE_TIMEOUT_SEC
# comment for why: autorepeat republishes the joystick's LAST CACHED reading
# on a timer regardless of whether the physical controller is still actually
# connected, which silently defeats every downstream staleness check (they
# only look at WHEN a message arrived, not whether the device is real) —
# this was the actual root cause of a reported runaway-on-disconnect bug
# 2026-07-15, not a bug in drive_mux's own staleness logic. With this at 0,
# /joy genuinely stops publishing on a real disconnect, letting drive_mux's
# staleness timeout do its job as designed.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    speed_scale_arg = DeclareLaunchArgument(
        'speed_scale', default_value='0.5',
        description='Motor power fraction [0.0-1.0] for serial_driver. '
                     'Boot default is 50% per explicit user request 2026-07-15.')

    serial_driver = Node(
        package='motor_controller',
        executable='serial_driver',
        name='serial_driver',
        output='screen',
        parameters=[{'speed_scale': LaunchConfiguration('speed_scale')}],
    )
    joy_node = Node(
        package='joy', executable='joy_node', name='joy_node', output='screen',
        parameters=[{'autorepeat_rate': 0.0}],
    )
    ps_controller_bridge = Node(
        package='robot_behavior', executable='ps_controller_bridge_node',
        name='ps_controller_bridge_node', output='screen')
    mode_manager = Node(
        package='robot_behavior', executable='mode_manager_node',
        name='mode_manager_node', output='screen')
    drive_mux = Node(
        package='robot_behavior', executable='drive_mux_node',
        name='drive_mux_node', output='screen')
    safety_watchdog = Node(
        package='robot_behavior', executable='safety_watchdog_node',
        name='safety_watchdog_node', output='screen')

    return LaunchDescription([
        speed_scale_arg,
        serial_driver,
        joy_node,
        ps_controller_bridge,
        mode_manager,
        drive_mux,
        safety_watchdog,
    ])
