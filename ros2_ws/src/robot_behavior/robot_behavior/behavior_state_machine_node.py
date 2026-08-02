"""Behavior state machine -- persistence + recovery for the autonomous
(auto/semi) driving path. This is an INTERNAL state within the existing
drive_source x track_mode model (mode_constants.py); it does NOT add a new
user-selectable mode dimension.

--------------------------------------------------------------------------
WHY THIS EXISTS
--------------------------------------------------------------------------
Before this refactor, a target loss ended the behavior almost immediately
(a couple of dropped frames -> lost -> intent_to_goal stops commanding
Nav2). This node adds real persistence (coast through brief dropouts, active
search when actually lost) and a costmap-aware, semi-blind-safe recovery
maneuver for the "robot rammed something and just sits there spinning its
wheels" failure mode.

--------------------------------------------------------------------------
STATES
--------------------------------------------------------------------------
TRACKING   -- target_state.detected is True (fresh measurement, per
              target_state_estimator's coast_window_sec). Passes through
              intent_to_goal/DWB's own output (/nav2/cmd_vel_smoothed)
              unchanged -- this node does not drive during TRACKING.
COASTING   -- covered by the SAME target_state.detected flag: the estimator
              already keeps `detected` True through its own coast window, so
              intent_to_goal keeps planning against the (predicted) filtered
              position with NO knowledge that this is a coast rather than a
              live detection. This state label exists for telemetry only
              (see _label_for_age); the actual behavior (full normal
              tracking, no interruption) is identical to TRACKING by
              construction, which is the point -- "brief dropouts must not
              interrupt driving."
SEARCHING  -- beyond the coast window. Active recovery to reacquire, with a
              CRITICAL asymmetry between follow and flee (see
              _searching_twist).
RECOVERING -- stuck detected (see _check_stuck). Executes a costmap-checked
              backup + reorient maneuver (see _tick_recovering).
LOST       -- search exhausted (age beyond the mode's search timeout) or
              recovery retries exhausted. Safe stop, hold, keep passively
              looking (trackers keep running regardless; reacquisition at
              any point returns straight to TRACKING).

--------------------------------------------------------------------------
OUTPUT ARBITRATION
--------------------------------------------------------------------------
This node is the auto_source_topic drive_mux actually consumes (see
nav2_bringup.launch.py's drive_mux auto_source_topic parameter -- was
/nav2/cmd_vel_smoothed directly, now this node's output topic). It:
  - passes /nav2/cmd_vel_smoothed straight through during TRACKING/COASTING
    (intent_to_goal/DWB already handles that driving),
  - substitutes its OWN computed Twist during SEARCHING/RECOVERING,
  - publishes zero during LOST.
This keeps intent_to_goal/DWB completely unmodified -- they just naturally
stop commanding anything once target_state_estimator's /target_detected
goes false (unchanged interface), and this node fills the gap.

--------------------------------------------------------------------------
MODE INTERACTION (ground rule: do not touch drive_source x track_mode)
--------------------------------------------------------------------------
manual: this node runs no search/recovery logic AT ALL and outputs zero --
        moot for motors anyway (drive_mux ignores the auto source entirely
        in manual), but the state machine itself must not evaluate/transition
        in manual per the spec, not just "not be applied."
semi:   search/recovery may run, but a fresh, non-deadzone /bias_cmd_vel
        (the human stick nudge) immediately zeroes this node's own output
        so drive_mux's semi blend (auto + bias) becomes pure human input --
        see _human_override_active.
auto:   full state machine active.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                        QoSReliabilityPolicy)
from std_msgs.msg import Float32, String

from robot_behavior import costmap_planner
from robot_behavior_msgs.msg import BehaviorState, TargetState
from robot_behavior.mode_constants import (
    DEFAULT_DRIVE_SOURCE,
    DEFAULT_TRACK_MODE,
    TOPIC_DRIVE_SOURCE_STATE,
    TOPIC_TRACK_MODE_STATE,
    latched_qos,
)

# --- Tunable constants ---------------------------------------------------
TICK_RATE_HZ = 10.0

BEHAVIOR_CMD_VEL_TOPIC = '/behavior/cmd_vel'
NAV2_CMD_VEL_TOPIC = '/nav2/cmd_vel_smoothed'

# Per-mode SEARCHING timeout, measured from when age_sec first exceeds the
# estimator's coast_window_sec (~0.5s) -- i.e. total time-since-loss before
# LOST is coast_window_sec + this. Flee gets a materially longer timeout
# than follow per the CRITICAL asymmetry this whole node exists to fix: a
# fleeing robot should keep running on its last heading rather than stopping
# to look, so it should be given more time to plausibly re-enter frame
# before giving up and stopping.
FOLLOW_SEARCH_TIMEOUT_SEC = 4.5
FLEE_SEARCH_TIMEOUT_SEC = 9.5

# Follow SEARCHING: rotate toward the last known bearing for this long, then
# switch to a slow oscillating scan-sweep for the remainder of the search
# window. Values chosen to visibly separate the two phases without a closed-
# loop yaw controller (this node deliberately keeps search motion open-loop
# and simple -- TUNABLE).
FOLLOW_SEARCH_ROTATE_TIME_SEC = 2.0
FOLLOW_SEARCH_ROTATE_SPEED_RPS = 0.5
FOLLOW_SEARCH_SCAN_SWEEP_SPEED_RPS = 0.3
FOLLOW_SEARCH_SCAN_PERIOD_SEC = 4.0

# Flee SEARCHING: keep moving. This is the specific behavior the whole
# refactor is built around -- see module docstring and the task's "door
# facing the person" failure description.
FLEE_SEARCH_LINEAR_MPS = 0.2

# Stuck detection: commanded velocity meaningfully nonzero, but odometry
# shows little/no actual movement, sustained for this many consecutive ticks
# (at TICK_RATE_HZ -> ~1.0s window). A consecutive-tick counter is used
# instead of a literal rolling buffer for simplicity; equivalent in effect.
STUCK_CONSECUTIVE_TICKS = 10
STUCK_CMD_LINEAR_MPS = 0.10
STUCK_CMD_ANGULAR_RPS = 0.30
STUCK_ODOM_LINEAR_MPS = 0.03
STUCK_ODOM_ANGULAR_RPS = 0.10
# Forward-costmap check: if driving forward and the costmap shows no clear
# cell in a short probe ahead, count that as an additional (immediate,
# same-tick) stuck vote alongside the odometry-based one.
STUCK_FORWARD_PROBE_M = 0.35

# Recovery maneuver. Backing up is SEMI-BLIND (ODIN faces forward, rear
# webcam has no depth) -- kept short/slow and gated on the local costmap
# showing the area behind as clear (the rolling costmap retains recently-
# observed obstacles even out of current FOV, which is what makes this
# checkable at all). TUNABLE, flagged per the task's explicit ask.
BACKUP_DISTANCE_M = 0.25
BACKUP_SPEED_MPS = 0.12
BACKUP_CLEARANCE_MARGIN_M = 0.15  # extra margin checked behind, beyond BACKUP_DISTANCE_M
BACKUP_MAX_DURATION_SEC = 4.0     # safety timeout if odom displacement tracking stalls
ROTATE_PROBE_M = 0.6              # how far ahead each candidate heading is sampled
ROTATE_SPEED_RPS = 0.5
ROTATE_TOLERANCE_RAD = 0.15
ROTATE_MAX_DURATION_SEC = 6.0
RECOVERY_RETRY_CAP = 3
# If a recovery finishes and stuck is detected again within this long, it
# counts toward the retry cap above rather than resetting it -- otherwise a
# robot wedged against something would recover-fail-recover-fail forever.
RECOVERY_COOLDOWN_SEC = 2.0

# Semi-mode human override: any fresh /bias_cmd_vel above this magnitude
# immediately zeroes this node's own output during SEARCHING/RECOVERING so
# the human's stick, not an in-progress maneuver, drives the robot.
BIAS_OVERRIDE_LINEAR_MPS = 0.05
BIAS_OVERRIDE_ANGULAR_RPS = 0.15
BIAS_STALE_SEC = 0.3
# -------------------------------------------------------------------------

STATE_TRACKING = 'TRACKING'
STATE_COASTING = 'COASTING'
STATE_SEARCHING = 'SEARCHING'
STATE_RECOVERING = 'RECOVERING'
STATE_LOST = 'LOST'


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    """Smallest signed difference a - b, wrapped to [-pi, pi]."""
    d = (a - b + math.pi) % (2.0 * math.pi) - math.pi
    return d


class BehaviorStateMachineNode(Node):

    def __init__(self):
        super().__init__('behavior_state_machine_node')

        self.declare_parameter('tick_rate_hz', TICK_RATE_HZ)
        self.declare_parameter('follow_search_timeout_sec', FOLLOW_SEARCH_TIMEOUT_SEC)
        self.declare_parameter('flee_search_timeout_sec', FLEE_SEARCH_TIMEOUT_SEC)
        self.declare_parameter('backup_distance_m', BACKUP_DISTANCE_M)
        self.declare_parameter('backup_speed_mps', BACKUP_SPEED_MPS)
        self.declare_parameter('recovery_retry_cap', RECOVERY_RETRY_CAP)

        self.tick_rate_hz = self.get_parameter('tick_rate_hz').value
        self.follow_search_timeout_sec = self.get_parameter('follow_search_timeout_sec').value
        self.flee_search_timeout_sec = self.get_parameter('flee_search_timeout_sec').value
        self.backup_distance_m = self.get_parameter('backup_distance_m').value
        self.backup_speed_mps = self.get_parameter('backup_speed_mps').value
        self.recovery_retry_cap = self.get_parameter('recovery_retry_cap').value

        self.drive_source = DEFAULT_DRIVE_SOURCE
        self.track_mode = DEFAULT_TRACK_MODE

        self.target_state = None       # latest TargetState
        self.last_target_bearing = 0.0  # latest legacy /target_bearing, for "last known bearing"
        self.nav2_cmd = Twist()
        self.odom = None               # latest nav_msgs/Odometry
        self.costmap = None            # latest OccupancyGrid
        self.bias_cmd = Twist()
        self.bias_cmd_t = 0.0

        self.state = STATE_LOST
        self._searching_entered_mono = None
        self._searching_exit_bearing = 0.0

        self._stuck_tick_count = 0
        self._pre_recovery_state = STATE_TRACKING
        self._recovery_phase = None   # 'check' | 'backing_up' | 'rotating'
        self._recovery_start_pose = None  # (x, y) odom, at backup start
        self._recovery_start_mono = None
        self._recovery_target_heading = None
        self._recovery_attempts = 0
        self._last_recovery_end_mono = None

        self.create_subscription(
            String, TOPIC_DRIVE_SOURCE_STATE, self._drive_source_cb, latched_qos())
        self.create_subscription(
            String, TOPIC_TRACK_MODE_STATE, self._track_mode_cb, latched_qos())
        self.create_subscription(TargetState, '/target_state', self._target_state_cb, 10)
        self.create_subscription(Float32, '/target_bearing', self._target_bearing_cb, 10)
        self.create_subscription(Twist, NAV2_CMD_VEL_TOPIC, self._nav2_cmd_cb, 10)
        self.create_subscription(Odometry, '/odin1/odometry', self._odom_cb, 10)
        self.create_subscription(Twist, '/bias_cmd_vel', self._bias_cb, 10)
        costmap_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self._costmap_cb, costmap_qos)

        self.cmd_pub = self.create_publisher(Twist, BEHAVIOR_CMD_VEL_TOPIC, 10)
        self.state_pub = self.create_publisher(BehaviorState, '/behavior_state', latched_qos())

        self.create_timer(1.0 / self.tick_rate_hz, self._tick)

        self.get_logger().info(
            f'behavior_state_machine_node started: tick_rate={self.tick_rate_hz}Hz, '
            f'follow_search_timeout={self.follow_search_timeout_sec}s, '
            f'flee_search_timeout={self.flee_search_timeout_sec}s, '
            f'backup={self.backup_distance_m}m @ {self.backup_speed_mps}m/s, '
            f'retry_cap={self.recovery_retry_cap}')

    # --- callbacks ---
    def _drive_source_cb(self, msg): self.drive_source = msg.data.strip().lower()
    def _track_mode_cb(self, msg): self.track_mode = msg.data.strip().lower()
    def _target_state_cb(self, msg): self.target_state = msg
    def _target_bearing_cb(self, msg): self.last_target_bearing = float(msg.data)
    def _nav2_cmd_cb(self, msg): self.nav2_cmd = msg
    def _odom_cb(self, msg): self.odom = msg
    def _costmap_cb(self, msg): self.costmap = msg

    def _bias_cb(self, msg):
        self.bias_cmd = msg
        self.bias_cmd_t = time.monotonic()

    def _human_override_active(self):
        if self.drive_source != 'semi':
            return False
        if time.monotonic() - self.bias_cmd_t > BIAS_STALE_SEC:
            return False
        return (abs(self.bias_cmd.linear.x) > BIAS_OVERRIDE_LINEAR_MPS
                or abs(self.bias_cmd.angular.z) > BIAS_OVERRIDE_ANGULAR_RPS)

    def _robot_pose(self):
        if self.odom is None:
            return None
        p = self.odom.pose.pose.position
        yaw = yaw_from_quat(self.odom.pose.pose.orientation)
        return p.x, p.y, yaw

    # --- main loop ---
    def _tick(self):
        if self.drive_source == 'manual':
            # No autonomous recovery/search evaluation at all in manual --
            # not just "not applied to motors" (drive_mux already ignores
            # this node's output there), the state machine itself must not
            # run. Publish zero and leave state as whatever it last was.
            self.cmd_pub.publish(Twist())
            return

        if self.target_state is None:
            self.cmd_pub.publish(Twist())
            return

        age = self.target_state.age_sec
        search_timeout = (self.flee_search_timeout_sec if self.track_mode == 'flee'
                           else self.follow_search_timeout_sec)

        # RECOVERING takes priority over everything else once entered --
        # driven to completion (or abort) by _tick_recovering before any
        # other state re-evaluation happens.
        if self.state == STATE_RECOVERING:
            self._tick_recovering()
            return

        # Stuck check applies whenever this node is actually issuing/passing
        # through nonzero motion, i.e. TRACKING/COASTING (passthrough) and
        # SEARCHING (own commands) -- not while already LOST (stopped) and
        # not mid-recovery (handled above).
        if self._check_stuck():
            self._enter_recovering()
            self._tick_recovering()
            return

        # Normal TRACKING/COASTING/SEARCHING/LOST evaluation from target age.
        if self.target_state.detected:
            new_state = STATE_TRACKING if age < 1e-3 else STATE_COASTING
        elif age <= search_timeout:
            # target_state_estimator's own coast window has already elapsed
            # (detected==False) by the time we get here -- SEARCHING covers
            # coast_window_sec..search_timeout total, per module docstring.
            new_state = STATE_SEARCHING
        else:
            new_state = STATE_LOST

        if new_state == STATE_SEARCHING and self.state != STATE_SEARCHING:
            self._searching_entered_mono = time.monotonic()
            self._searching_exit_bearing = self.last_target_bearing
        self._set_state(new_state, reason=self._reason_for(new_state, age))

        if self.state in (STATE_TRACKING, STATE_COASTING):
            out = self.nav2_cmd
        elif self.state == STATE_SEARCHING:
            out = self._searching_twist()
        else:  # LOST
            out = Twist()

        if self._human_override_active():
            out = Twist()

        self.cmd_pub.publish(out)

    def _reason_for(self, state, age):
        if state == STATE_TRACKING:
            return 'fresh detection'
        if state == STATE_COASTING:
            return f'coasting on prediction, age={age:.2f}s'
        if state == STATE_SEARCHING:
            return (f'{self.track_mode}: searching, last bearing='
                    f'{self._searching_exit_bearing:+.2f}, age={age:.2f}s')
        return f'search exhausted, age={age:.2f}s'

    def _set_state(self, new_state, reason=''):
        if new_state != self.state:
            self.get_logger().info(f'behavior_state: {self.state} -> {new_state} ({reason})')
        self.state = new_state
        msg = BehaviorState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = new_state
        msg.reason = reason
        self.state_pub.publish(msg)

    # --- SEARCHING ---
    def _searching_twist(self):
        out = Twist()
        if self.track_mode == 'flee':
            # CRITICAL asymmetry (see module docstring): keep moving away on
            # the last heading rather than stopping to look. Reduced speed,
            # no turning.
            out.linear.x = FLEE_SEARCH_LINEAR_MPS
            out.angular.z = 0.0
            return out

        # Follow: stop, rotate toward the last known bearing, then scan-sweep.
        elapsed = time.monotonic() - self._searching_entered_mono
        out.linear.x = 0.0
        bearing_sign = 1.0 if self._searching_exit_bearing >= 0.0 else -1.0
        if elapsed < FOLLOW_SEARCH_ROTATE_TIME_SEC:
            # Same sign convention as target_pid: positive bearing (target to
            # the right) needs NEGATIVE angular.z to turn toward it.
            out.angular.z = -bearing_sign * FOLLOW_SEARCH_ROTATE_SPEED_RPS
        else:
            sweep_t = elapsed - FOLLOW_SEARCH_ROTATE_TIME_SEC
            phase = math.sin(2.0 * math.pi * sweep_t / FOLLOW_SEARCH_SCAN_PERIOD_SEC)
            out.angular.z = FOLLOW_SEARCH_SCAN_SWEEP_SPEED_RPS * (1.0 if phase >= 0.0 else -1.0)
        return out

    # --- stuck detection ---
    def _check_stuck(self):
        if self.state == STATE_LOST:
            return False
        if self._last_recovery_end_mono is not None and (
                time.monotonic() - self._last_recovery_end_mono) < RECOVERY_COOLDOWN_SEC:
            return False  # cooldown after a just-completed recovery

        commanded = self.nav2_cmd if self.state in (STATE_TRACKING, STATE_COASTING) else self._searching_twist()
        cmd_moving = (abs(commanded.linear.x) > STUCK_CMD_LINEAR_MPS
                      or abs(commanded.angular.z) > STUCK_CMD_ANGULAR_RPS)

        odom_still = True
        if self.odom is not None:
            lin = abs(self.odom.twist.twist.linear.x)
            ang = abs(self.odom.twist.twist.angular.z)
            odom_still = lin < STUCK_ODOM_LINEAR_MPS and ang < STUCK_ODOM_ANGULAR_RPS

        forward_blocked = False
        if self.costmap is not None and commanded.linear.x > STUCK_CMD_LINEAR_MPS:
            pose = self._robot_pose()
            if pose is not None:
                rx, ry, yaw = pose
                cost = costmap_planner.heading_clearance(
                    self.costmap, (rx, ry), yaw, STUCK_FORWARD_PROBE_M)
                forward_blocked = cost >= costmap_planner.LETHAL_THRESHOLD

        stuck_this_tick = cmd_moving and (odom_still or forward_blocked)
        if stuck_this_tick:
            self._stuck_tick_count += 1
        else:
            self._stuck_tick_count = 0

        return self._stuck_tick_count >= STUCK_CONSECUTIVE_TICKS

    # --- RECOVERING ---
    def _enter_recovering(self):
        self._pre_recovery_state = self.state
        self._recovery_phase = 'check'
        self._stuck_tick_count = 0
        self.get_logger().warn('Stuck detected -- entering RECOVERING')

    def _tick_recovering(self):
        if self._human_override_active():
            # Semi mode, human took the stick -- abort immediately, hand
            # control back with zero autonomous output on top of it.
            self.get_logger().info('RECOVERING aborted: human stick override')
            self._finish_recovery(success=False)
            self.cmd_pub.publish(Twist())
            return

        pose = self._robot_pose()
        if pose is None:
            self.cmd_pub.publish(Twist())
            return
        rx, ry, yaw = pose

        if self._recovery_phase == 'check':
            self._set_state(STATE_RECOVERING, reason='checking costmap behind robot')
            if self.costmap is None:
                # No costmap yet -- refuse to back up blind at all.
                self.get_logger().warn('RECOVERING: no costmap available, refusing backup')
                self._finish_recovery(success=False)
                self.cmd_pub.publish(Twist())
                return
            behind_heading = yaw + math.pi
            check_dist = self.backup_distance_m + BACKUP_CLEARANCE_MARGIN_M
            probe_points = [
                (rx + math.cos(behind_heading) * check_dist * f,
                 ry + math.sin(behind_heading) * check_dist * f)
                for f in (0.33, 0.66, 1.0)
            ]
            if not costmap_planner.region_is_clear(self.costmap, probe_points):
                self.get_logger().warn(
                    'RECOVERING: costmap shows an obstacle behind the robot -- refusing backup')
                self._finish_recovery(success=False)
                self.cmd_pub.publish(Twist())
                return
            self._recovery_phase = 'backing_up'
            self._recovery_start_pose = (rx, ry)
            self._recovery_start_mono = time.monotonic()

        if self._recovery_phase == 'backing_up':
            self._set_state(STATE_RECOVERING, reason='backing up')
            sx, sy = self._recovery_start_pose
            traveled = math.hypot(rx - sx, ry - sy)
            elapsed = time.monotonic() - self._recovery_start_mono
            if traveled >= self.backup_distance_m or elapsed >= BACKUP_MAX_DURATION_SEC:
                self._recovery_phase = 'rotating'
                self._recovery_target_heading = costmap_planner.best_clear_heading(
                    self.costmap, (rx, ry), ROTATE_PROBE_M)
                self._recovery_start_mono = time.monotonic()
            else:
                out = Twist()
                out.linear.x = -self.backup_speed_mps
                self.cmd_pub.publish(out)
                return

        if self._recovery_phase == 'rotating':
            self._set_state(STATE_RECOVERING, reason='reorienting to a clear heading')
            elapsed = time.monotonic() - self._recovery_start_mono
            target = self._recovery_target_heading
            done_rotating = (
                target is None
                or abs(angle_diff(target, yaw)) <= ROTATE_TOLERANCE_RAD
                or elapsed >= ROTATE_MAX_DURATION_SEC
            )
            if done_rotating:
                self._finish_recovery(success=True)
                self.cmd_pub.publish(Twist())
                return
            out = Twist()
            out.angular.z = ROTATE_SPEED_RPS if angle_diff(target, yaw) > 0 else -ROTATE_SPEED_RPS
            self.cmd_pub.publish(out)
            return

    def _finish_recovery(self, success):
        self._recovery_phase = None
        self._last_recovery_end_mono = time.monotonic()
        if success:
            self._recovery_attempts = 0
        else:
            self._recovery_attempts += 1
        if self._recovery_attempts >= self.recovery_retry_cap:
            self.get_logger().warn(
                f'RECOVERING failed {self._recovery_attempts} times -- giving up, going LOST')
            self._recovery_attempts = 0
            self._set_state(STATE_LOST, reason='recovery retry cap exceeded')
        else:
            self._set_state(self._pre_recovery_state, reason='recovery maneuver complete')


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorStateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
