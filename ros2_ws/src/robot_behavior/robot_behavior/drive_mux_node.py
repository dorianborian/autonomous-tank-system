import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

from robot_behavior.mode_constants import (
    DEFAULT_DRIVE_SOURCE,
    TOPIC_DRIVE_SOURCE_STATE,
    latched_qos,
)

PUBLISH_RATE_HZ = 20.0  # matches serial_driver's own send rate

# Any input Twist older than this is treated as zero rather than replayed
# forever. serial_driver has no staleness check of its own on /cmd_vel (it
# just resends whatever it last received at 20Hz) — the ESP32's 500ms
# watchdog only protects against total silence, not a stuck-but-still-arriving
# stale command. This is a deliberate defense-in-depth addition beyond the
# literal spec, tunable.
#
# IMPORTANT — this check is only meaningful if upstream actually stops
# publishing on a real disconnect. joy_node's autorepeat_rate (ROS default
# 20Hz) republishes its last cached reading on a timer regardless of whether
# the physical controller is still connected, which silently defeats this
# entirely — messages keep arriving "fresh" by timestamp with frozen data.
# This was the actual root cause of a reported runaway-on-disconnect bug
# (2026-07-15), not a bug in this staleness check itself. Fixed at the
# source: autorepeat_rate=0.0 is now set on every joy_node launch (see
# manual_mode.launch.py / nav2_bringup.launch.py) so a real disconnect
# actually stops /joy, letting this timeout do its job.
#
# Tightened 0.3 -> 0.15s same session, per explicit "should stop immediately"
# request — still comfortably above a real controller's normal report
# interval (well under 20ms for a connected BT gamepad), so it shouldn't
# nuisance-trip during normal use, just cut the "coast on stale data" window.
INPUT_STALE_TIMEOUT_SEC = 0.15

# Clamp applied ONLY to the semi-mode blended sum (manual and auto pass
# their source through directly, already bounded upstream by
# ps_controller_bridge / target_pid). Set intentionally below the
# theoretical worst-case sum (target_pid max + full-scale bias) as an extra
# safety margin, since a blended command is less predictable than either
# input alone. Tunable.
MUX_SEMI_MAX_LINEAR_X = 0.6   # m/s
MUX_SEMI_MAX_ANGULAR_Z = 1.3  # rad/s

# Phase 7: the autonomous velocity source for 'auto' and 'semi' is now the
# Nav2 controller output AFTER the velocity_smoother (obstacle-aware +
# accel/jerk limited), NOT the old direct target_pid /cmd_vel_pid. target_pid
# is no longer wired into driving; it remains in the tree only as a documented
# fallback (set auto_source_topic back to /cmd_vel_pid to revert to Phase 6
# direct-velocity behaviour). There is intentionally only ONE autonomous source
# feeding the mux at a time — no two-source ambiguity. Tunable via parameter.
DEFAULT_AUTO_SOURCE_TOPIC = '/nav2/cmd_vel_smoothed'

# Phase 7.1 tried inserting nav2_collision_monitor downstream of drive_mux
# (cmd_vel_in=/cmd_vel_premonitor, cmd_vel_out=/cmd_vel) as a stop/slowdown
# safety net for manual/semi joystick driving. REMOVED same session
# (2026-07-14): ODIN's point-cloud timestamps run on the sensor's own
# hardware clock, tens of minutes offset from ROS system time, which made
# collision_monitor's source-staleness check permanently distrust live data
# and silently withhold ALL /cmd_vel output — a real, user-facing failure,
# not a tuning issue. User's explicit call: remove it rather than paper over
# the mismatch — the PS-button e-stop and a physical kill switch are the
# real safety net and don't depend on any of this. drive_mux is back to
# publishing /cmd_vel directly. 'auto'/'semi' obstacle avoidance still comes
# from DWB + the local costmap (unaffected, that path never depended on
# collision_monitor). intent_to_goal's joystick-goal projection (semi, no
# marker) is unaffected either — this only removes the downstream veto.
CMD_VEL_OUT_TOPIC = '/cmd_vel'

# Deadman gate (2026-07-14, user request): intent_to_goal keeps computing
# goals and controller_server/DWB keeps planning regardless — the goal marker
# and route stay visible on the costmap viewer at all times — but drive_mux
# only lets that motion actually reach the motors while R2 is held
# (/nav_enable, published by ps_controller_bridge). Scoped to auto/semi only:
# manual is raw joystick with no goal/autonomy concept, and releasing the
# stick already stops it there, so gating it too would be redundant. Fails
# safe: no/stale /nav_enable is treated as NOT held, same as every other
# input staleness check in this node.
NAV_ENABLE_STALE_TIMEOUT_SEC = 0.3

# Web UI "auto drive, no trigger held" toggle (2026-07-25): a second,
# independent deadman input that ORs with the real R2 signal above rather
# than replacing it — /nav_enable stays the physical-trigger source exactly
# as before (still works if this override is never touched), and
# /nav_enable_override is published by web_ui_bridge_node reflecting the
# on-screen toggle state. Kept as a SEPARATE topic rather than having the
# web UI also publish onto /nav_enable itself: two independent publishers on
# one topic would race (drive_mux only ever sees whichever arrived most
# recently), silently flickering the gate open/closed depending on
# interleaving. A dedicated topic + explicit OR avoids that entirely.
NAV_ENABLE_OVERRIDE_STALE_TIMEOUT_SEC = 0.5


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class DriveMuxNode(Node):

    def __init__(self):
        super().__init__('drive_mux_node')

        self.declare_parameter('input_stale_timeout_sec', INPUT_STALE_TIMEOUT_SEC)
        self.declare_parameter('semi_max_linear_x', MUX_SEMI_MAX_LINEAR_X)
        self.declare_parameter('semi_max_angular_z', MUX_SEMI_MAX_ANGULAR_Z)
        self.declare_parameter('auto_source_topic', DEFAULT_AUTO_SOURCE_TOPIC)
        self.input_stale_timeout_sec = self.get_parameter('input_stale_timeout_sec').value
        self.semi_max_linear_x = self.get_parameter('semi_max_linear_x').value
        self.semi_max_angular_z = self.get_parameter('semi_max_angular_z').value
        self.auto_source_topic = self.get_parameter('auto_source_topic').value

        self.drive_source = DEFAULT_DRIVE_SOURCE

        self._manual = Twist()
        self._manual_t = 0.0
        self._bias = Twist()
        self._bias_t = 0.0
        self._auto = Twist()   # smoothed Nav2 output (was target_pid /cmd_vel_pid)
        self._auto_t = 0.0
        self._nav_enable = False
        self._nav_enable_t = 0.0
        self._nav_enable_override = False
        self._nav_enable_override_t = 0.0

        self.create_subscription(
            String, TOPIC_DRIVE_SOURCE_STATE, self._drive_source_cb, latched_qos())
        self.create_subscription(Twist, '/manual_cmd_vel', self._manual_cb, 10)
        self.create_subscription(Twist, '/bias_cmd_vel', self._bias_cb, 10)
        self.create_subscription(Twist, self.auto_source_topic, self._auto_cb, 10)
        self.create_subscription(Bool, '/nav_enable', self._nav_enable_cb, 10)
        self.create_subscription(
            Bool, '/nav_enable_override', self._nav_enable_override_cb, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_OUT_TOPIC, 10)
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_tick)

        self.get_logger().info(
            f'drive_mux_node started: input_stale_timeout={self.input_stale_timeout_sec}s, '
            f'semi clamp=({self.semi_max_linear_x} m/s, {self.semi_max_angular_z} rad/s), '
            f'auto/semi source={self.auto_source_topic}, output={CMD_VEL_OUT_TOPIC}, '
            'auto/semi gated on /nav_enable (hold R2) OR /nav_enable_override (web UI toggle)')

    def _drive_source_cb(self, msg: String):
        self.drive_source = msg.data.strip().lower()

    def _manual_cb(self, msg: Twist):
        self._manual = msg
        self._manual_t = time.monotonic()

    def _bias_cb(self, msg: Twist):
        self._bias = msg
        self._bias_t = time.monotonic()

    def _auto_cb(self, msg: Twist):
        self._auto = msg
        self._auto_t = time.monotonic()

    def _nav_enable_cb(self, msg: Bool):
        self._nav_enable = bool(msg.data)
        self._nav_enable_t = time.monotonic()

    def _nav_enable_override_cb(self, msg: Bool):
        self._nav_enable_override = bool(msg.data)
        self._nav_enable_override_t = time.monotonic()

    def _fresh(self, twist, received_at):
        if time.monotonic() - received_at > self.input_stale_timeout_sec:
            return Twist()  # zero
        return twist

    def _nav_enabled(self):
        r2_held = (
            time.monotonic() - self._nav_enable_t <= NAV_ENABLE_STALE_TIMEOUT_SEC
            and self._nav_enable
        )
        override_on = (
            time.monotonic() - self._nav_enable_override_t <= NAV_ENABLE_OVERRIDE_STALE_TIMEOUT_SEC
            and self._nav_enable_override
        )
        return r2_held or override_on

    def _publish_tick(self):
        manual = self._fresh(self._manual, self._manual_t)
        bias = self._fresh(self._bias, self._bias_t)
        auto = self._fresh(self._auto, self._auto_t)

        out = Twist()

        if self.drive_source == 'manual':
            out = manual
        elif self.drive_source in ('auto', 'semi'):
            if not self._nav_enabled():
                # Deadman not held: intent_to_goal/DWB keep planning (goal +
                # route stay visible), but nothing reaches the motors.
                out = Twist()
            elif self.drive_source == 'auto':
                out = auto
            else:  # semi
                # Same blend as Phase 6 (controller bias on top of the
                # autonomous command), just fed by the smoothed Nav2 output
                # instead of raw target_pid.
                out.linear.x = clamp(
                    auto.linear.x + bias.linear.x,
                    -self.semi_max_linear_x, self.semi_max_linear_x)
                out.angular.z = clamp(
                    auto.angular.z + bias.angular.z,
                    -self.semi_max_angular_z, self.semi_max_angular_z)
        else:
            # Unknown/unset drive_source -> safest is no motion.
            out = Twist()

        self.cmd_vel_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DriveMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
