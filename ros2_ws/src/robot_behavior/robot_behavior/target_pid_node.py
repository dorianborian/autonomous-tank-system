import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from robot_behavior.mode_constants import (
    DEFAULT_TRACK_MODE,
    TOPIC_TRACK_MODE_STATE,
    latched_qos,
)

CONTROL_RATE_HZ = 20.0

# --- Tunable constants -----------------------------------------------------
# Bearing PID gains (bearing is normalized [-1, 1], output is angular.z rad/s)
PID_KP_BEARING = 1.5
PID_KI_BEARING = 0.0
PID_KD_BEARING = 0.3
MAX_ANGULAR_Z = 1.0  # rad/s clamp on PID output
# Bearing is normalized [-1, 1]; ignore anything smaller than this rather than
# constantly micro-correcting. Root cause found live: differential-drive
# turning isn't a perfect in-place pivot (motor speed mismatch + the
# min-effective-speed floor turn even a tiny bearing error into a real,
# somewhat-off-center turn), so it was translating the robot slightly on
# every "just centering the bearing" correction — which then read as a
# distance-zone change and triggered more motion, compounding into the
# reported wiggling near the dead zone. Ignoring small bearing error removes
# the main source of that unwanted turning.
BEARING_DEADBAND = 0.08

# Distance control (follow mode), three zones per live-testing feedback:
#   distance < REVERSE_THRESHOLD_M           -> back away
#   REVERSE_THRESHOLD_M..CHASE_MIN_M         -> dead zone, hold position
#   CHASE_MIN_M..CHASE_MAX_M (and beyond)    -> chase forward
# Speed ramps linearly to MAX_LINEAR_X at each zone's far edge (0 at
# REVERSE_THRESHOLD_M for reverse, MAX_LINEAR_X at CHASE_MAX_M for chase;
# chasing stays saturated at MAX_LINEAR_X beyond CHASE_MAX_M rather than
# hard-capping distance).
REVERSE_THRESHOLD_M = 0.4
CHASE_MIN_M = 0.6
# Full chase speed is reached here (not at some far "max tracking range") —
# live-tested: a 0.4-2.0m ramp made near-edge speeds too small to feel/move
# at all. Tightened so it's already at full speed by 1.0m; chasing stays
# saturated at MAX_LINEAR_X for any distance beyond this.
CHASE_MAX_M = 1.0
MAX_LINEAR_X = 0.4  # m/s clamp on PID output (well under serial_driver's 1.0 m/s full scale)

# Hysteresis margin (meters) on the zone boundaries. Without this, detection
# noise plus the min-effective-speed floor overshoot (serial_driver boosts
# any small nonzero command up to its own speed floor) made the robot cross
# a zone boundary, react, overshoot back across it, and repeat — visible as
# constant wiggling right at the reverse/dead-zone/chase edges instead of
# holding still. Leaving a zone now requires crossing DISTANCE_HYSTERESIS_M
# further past the boundary than entering it did.
DISTANCE_HYSTERESIS_M = 0.08
# -----------------------------------------------------------------------


class TargetPidNode(Node):

    def __init__(self):
        super().__init__('target_pid_node')

        self.declare_parameter('kp_bearing', PID_KP_BEARING)
        self.declare_parameter('ki_bearing', PID_KI_BEARING)
        self.declare_parameter('kd_bearing', PID_KD_BEARING)
        self.declare_parameter('max_angular_z', MAX_ANGULAR_Z)
        self.declare_parameter('bearing_deadband', BEARING_DEADBAND)
        self.declare_parameter('reverse_threshold_m', REVERSE_THRESHOLD_M)
        self.declare_parameter('chase_min_m', CHASE_MIN_M)
        self.declare_parameter('chase_max_m', CHASE_MAX_M)
        self.declare_parameter('max_linear_x', MAX_LINEAR_X)
        self.declare_parameter('distance_hysteresis_m', DISTANCE_HYSTERESIS_M)

        self.kp_bearing = self.get_parameter('kp_bearing').value
        self.ki_bearing = self.get_parameter('ki_bearing').value
        self.kd_bearing = self.get_parameter('kd_bearing').value
        self.max_angular_z = self.get_parameter('max_angular_z').value
        self.bearing_deadband = self.get_parameter('bearing_deadband').value
        self.reverse_threshold_m = self.get_parameter('reverse_threshold_m').value
        self.chase_min_m = self.get_parameter('chase_min_m').value
        self.chase_max_m = self.get_parameter('chase_max_m').value
        self.max_linear_x = self.get_parameter('max_linear_x').value
        self.distance_hysteresis_m = self.get_parameter('distance_hysteresis_m').value

        self._distance_zone = 'dead'  # 'reverse' | 'dead' | 'chase'

        self.track_mode = DEFAULT_TRACK_MODE
        self.target_detected = False
        self.target_bearing = 0.0
        self.target_distance = 0.0

        self._bearing_integral = 0.0
        self._bearing_prev_error = 0.0
        self._last_tick_time = time.monotonic()

        self.create_subscription(
            String, TOPIC_TRACK_MODE_STATE, self._track_mode_cb, latched_qos())
        self.create_subscription(Bool, '/target_detected', self._detected_cb, 10)
        self.create_subscription(Float32, '/target_bearing', self._bearing_cb, 10)
        self.create_subscription(Float32, '/target_distance', self._distance_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_pid', 10)

        self.create_timer(1.0 / CONTROL_RATE_HZ, self._control_tick)

        self.get_logger().info(
            f'target_pid_node started: control_rate={CONTROL_RATE_HZ}Hz, '
            f'reverse<{self.reverse_threshold_m}m, dead zone to {self.chase_min_m}m, '
            f'chase {self.chase_min_m}-{self.chase_max_m}m')

    def _track_mode_cb(self, msg: String):
        self.track_mode = msg.data.strip().lower()

    def _detected_cb(self, msg: Bool):
        self.target_detected = bool(msg.data)
        if not self.target_detected:
            # Avoid integral windup while there's nothing to track.
            self._bearing_integral = 0.0
            self._bearing_prev_error = 0.0
            self._distance_zone = 'dead'

    def _bearing_cb(self, msg: Float32):
        self.target_bearing = float(msg.data)

    def _distance_cb(self, msg: Float32):
        self.target_distance = float(msg.data)

    def _next_distance_zone(self, distance, current_zone, hysteresis):
        """Schmitt-trigger zone selection: leaving a zone requires crossing
        `hysteresis` further past its boundary than entering it did."""
        if current_zone == 'reverse':
            if distance >= self.reverse_threshold_m + hysteresis:
                current_zone = 'dead'
        elif current_zone == 'chase':
            if distance <= self.chase_min_m - hysteresis:
                current_zone = 'dead'
        else:  # 'dead'
            if distance < self.reverse_threshold_m - hysteresis:
                current_zone = 'reverse'
            elif distance > self.chase_min_m + hysteresis:
                current_zone = 'chase'
        return current_zone

    def _control_tick(self):
        now = time.monotonic()
        dt = max(1e-3, now - self._last_tick_time)
        self._last_tick_time = now

        twist = Twist()

        if self.target_detected:
            # Bearing PID -> angular.z. Positive bearing means target is to
            # the right of center, and the robot needs to turn right to
            # face it. serial_driver's differential-drive mixing
            # (v_right = linear + angular*TRACK_WIDTH/2) makes positive
            # angular.z turn the robot LEFT (right wheel speeds up), matching
            # REP-103 (+z = CCW). So a positive bearing error must produce a
            # NEGATIVE angular.z command — the PID output is negated below.
            error = self.target_bearing
            if abs(error) < self.bearing_deadband:
                # Close enough to centered — don't turn at all. Turning isn't
                # a perfect in-place pivot on this chassis, so correcting
                # tiny bearing errors was itself moving the robot enough to
                # perturb the distance reading (see BEARING_DEADBAND note).
                angular_z = 0.0
                self._bearing_integral = 0.0
                self._bearing_prev_error = 0.0
            else:
                self._bearing_integral += error * dt
                derivative = (error - self._bearing_prev_error) / dt
                self._bearing_prev_error = error

                angular_z = -(
                    self.kp_bearing * error
                    + self.ki_bearing * self._bearing_integral
                    + self.kd_bearing * derivative
                )
                angular_z = clamp(angular_z, -self.max_angular_z, self.max_angular_z)

            # Distance control -> linear.x, three zones (see constants above),
            # with hysteresis so noise/overshoot right at a boundary doesn't
            # cause rapid zone flip-flopping (see DISTANCE_HYSTERESIS_M).
            d = self.target_distance
            h = self.distance_hysteresis_m
            self._distance_zone = self._next_distance_zone(d, self._distance_zone, h)

            if self._distance_zone == 'reverse':
                # Ramps to -max_linear_x at d=0.
                linear_x = -self.max_linear_x * (self.reverse_threshold_m - d) / self.reverse_threshold_m
            elif self._distance_zone == 'dead':
                linear_x = 0.0
            else:  # 'chase'
                # Ramps to +max_linear_x at d=chase_max_m, saturates beyond it.
                span = self.chase_max_m - self.chase_min_m
                linear_x = self.max_linear_x * (d - self.chase_min_m) / span
            linear_x = clamp(linear_x, -self.max_linear_x, self.max_linear_x)

            twist.linear.x = linear_x
            twist.angular.z = angular_z

            if self.track_mode == 'flee':
                # Flee inverts the follow control law so a nearby rear threat
                # pushes the robot forward, away from it. But we do NOT let a
                # FAR threat pull the robot back toward it: after inversion,
                # follow's "chase forward when far" becomes "reverse toward
                # threat when far" (negative linear.x here) — per design
                # decision that is suppressed to a stop, so a distant threat
                # just means hold, never re-approach. Angular still inverts so
                # the robot steers its heading away from the threat's bearing.
                twist.linear.x = -twist.linear.x
                twist.angular.z = -twist.angular.z
                if twist.linear.x < 0.0:
                    twist.linear.x = 0.0
        # else: target not detected -> publish zero Twist (stay at defaults).

        self.cmd_pub.publish(twist)


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def main(args=None):
    rclpy.init(args=args)
    node = TargetPidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
