import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String

from robot_behavior.mode_constants import (
    DEFAULT_DRIVE_SOURCE,
    TOPIC_DRIVE_SOURCE_CMD,
    TOPIC_DRIVE_SOURCE_STATE,
    latched_qos,
)

WATCHDOG_RATE_HZ = 10.0
JOY_STALE_TIMEOUT_SEC = 0.5   # controller considered disconnected past this
TARGET_LOSS_LOG_TIMEOUT_SEC = 3.0  # informational log threshold, see note below


class SafetyWatchdogNode(Node):
    """
    Two independent safety behaviors:

    1. Controller-disconnect fallback (drive_source in {manual, semi} only):
       if /joy goes stale, hand off to 'auto' when a target is currently
       detected (purposeful autonomous behavior beats a dead stop mid-turn),
       otherwise force 'manual' (drive_mux's own input-staleness handling
       then zeroes the also-stale /manual_cmd_vel -> full stop). This is
       edge-triggered: it fires once on entering the stale condition and
       does NOT auto-revert when the controller reconnects — the operator
       must explicitly re-select manual/semi afterward, so control isn't
       silently handed back the instant Bluetooth reconnects (sticks may
       not be neutral).

    2. Target-loss logging (drive_source in {auto, semi}): target_pid
       already stops driving immediately (zero Twist) the instant
       /target_detected goes false — there's no reason to keep moving
       blind even briefly. target_pid did not exist before this phase (no
       prior "3s" logic was found anywhere on disk), so there is no legacy
       behavior being preserved here; this 3s log is the closest reasonable
       interpretation of "confirm this still works" for a behavior that is
       actually new. It is purely informational — it does not change the
       robot's motion, which is already stopped by target_pid.
    """

    def __init__(self):
        super().__init__('safety_watchdog_node')

        self.declare_parameter('joy_stale_timeout_sec', JOY_STALE_TIMEOUT_SEC)
        self.declare_parameter('target_loss_log_timeout_sec', TARGET_LOSS_LOG_TIMEOUT_SEC)
        self.joy_stale_timeout_sec = self.get_parameter('joy_stale_timeout_sec').value
        self.target_loss_log_timeout_sec = self.get_parameter('target_loss_log_timeout_sec').value

        self.drive_source = DEFAULT_DRIVE_SOURCE
        self.target_detected = False

        self._last_joy_time = None  # None == never received == stale
        self._fallback_active = False

        self._target_lost_since = None
        self._target_lost_logged = False

        self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.create_subscription(
            String, TOPIC_DRIVE_SOURCE_STATE, self._drive_source_cb, latched_qos())
        self.create_subscription(Bool, '/target_detected', self._target_detected_cb, 10)

        self.drive_source_cmd_pub = self.create_publisher(String, TOPIC_DRIVE_SOURCE_CMD, 10)

        self.create_timer(1.0 / WATCHDOG_RATE_HZ, self._tick)

        self.get_logger().info(
            f'safety_watchdog_node started: joy_stale_timeout={self.joy_stale_timeout_sec}s, '
            f'target_loss_log_timeout={self.target_loss_log_timeout_sec}s')

    def _joy_cb(self, msg: Joy):
        self._last_joy_time = time.monotonic()

    def _drive_source_cb(self, msg: String):
        self.drive_source = msg.data.strip().lower()

    def _target_detected_cb(self, msg: Bool):
        self.target_detected = bool(msg.data)

    def _tick(self):
        self._check_controller_fallback()
        self._check_target_loss_log()

    def _check_controller_fallback(self):
        now = time.monotonic()
        joy_stale = (
            self._last_joy_time is None
            or (now - self._last_joy_time) > self.joy_stale_timeout_sec
        )
        needs_fallback = self.drive_source in ('manual', 'semi') and joy_stale

        if needs_fallback and not self._fallback_active:
            self._fallback_active = True
            fallback_msg = String()
            if self.target_detected:
                fallback_msg.data = 'auto'
                self.get_logger().warn(
                    'Controller stale (/joy) while drive_source='
                    f'{self.drive_source} — target detected, forcing drive_source=auto')
            else:
                fallback_msg.data = 'manual'
                self.get_logger().warn(
                    'Controller stale (/joy) while drive_source='
                    f'{self.drive_source} — no target, forcing drive_source=manual '
                    '(drive_mux will zero the equally-stale manual_cmd_vel -> full stop)')
            self.drive_source_cmd_pub.publish(fallback_msg)
        elif not needs_fallback:
            self._fallback_active = False

    def _check_target_loss_log(self):
        if self.target_detected:
            self._target_lost_since = None
            self._target_lost_logged = False
            return

        if self.drive_source not in ('auto', 'semi'):
            return

        now = time.monotonic()
        if self._target_lost_since is None:
            self._target_lost_since = now
            return

        lost_for = now - self._target_lost_since
        if not self._target_lost_logged and lost_for >= self.target_loss_log_timeout_sec:
            self._target_lost_logged = True
            self.get_logger().warn(
                f'Target lost for >= {self.target_loss_log_timeout_sec:.0f}s while '
                f'drive_source={self.drive_source} (target_pid already holding at zero Twist)')


def main(args=None):
    rclpy.init(args=args)
    node = SafetyWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
