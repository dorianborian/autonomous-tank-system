"""Target state estimator -- sits between the raw per-frame trackers
(odin_yolo_tracker_node / rear_yolo_tracker_node) and everything downstream
(target_pid, intent_to_goal), replacing target_selector's role as well as
the trackers' own crude miss-hold/persistence logic.

--------------------------------------------------------------------------
THE BUG THIS FIXES
--------------------------------------------------------------------------
The old pipeline was: YOLO bearing/distance (robot-relative) -> transform to
odom using the robot's CURRENT pose -> set as goal. Every term in that chain
depends on the robot's own pose estimate *at the moment the message happens
to be processed*, not at the moment the image was actually captured. Odometry
drift, TF-lookup/processing latency, and motor asymmetry all get injected
directly into where the system believes the person is standing -- turn ->
new (wrong) bearing -> goal moves -> replan -> turn more. That is the
observed circling behavior.

This node instead tracks the person in the WORLD-FIXED odom frame with a
constant-velocity motion model (alpha-beta filter -- see FRONT_ALPHA/BETA
below for why this was chosen over a full Kalman filter). A measurement
CORRECTS the estimate; it does not replace it. If the robot moves and the
person did not, the odom-frame estimate stays put.

--------------------------------------------------------------------------
TIMESTAMP DISCIPLINE
--------------------------------------------------------------------------
Each raw TargetDetection carries header.stamp = the SOURCE IMAGE's capture
time (fixed in the tracker nodes as part of this refactor -- see their
`detection_pub` additions). When a detection arrives, this node looks up
odom<-robot_base_frame AT THAT STAMP (tf2 time-travel lookup) to compute the
world-frame point the detection implies, instead of using whatever pose the
robot has *right now*. At ~2.5 m/s with ~150ms of pipeline latency the robot
has moved ~0.4m -- fusing against a stale/current pose is a real error
source, not a rounding detail.

CONFIRMED LIVE (2026-07-26): /odin1/image, /odin1/odometry, /odin1/cloud_slam,
/odin1/cloud_raw, AND the odom->odin1_base_link /tf transform are ALL stamped
in the same non-wall-clock domain -- offset from system time by
~1,785,099,631 seconds (~56.6 years, not "tens of minutes" as nav2_params.yaml's
collision_monitor note estimated -- same conclusion, bigger number), measured
twice ~3s apart with the offset agreeing to within 0.5s both times. This is
exactly why collision_monitor was removed in Phase 7.1 (it compared message
stamps directly against node now() and permanently distrusted "stale" data
that was actually fresh). A direct `now() - header.stamp` subtraction on any
ODIN-derived message is nonsense in this domain.

The important part for THIS node: image, odometry, and TF are all in that
SAME domain together, which is exactly what makes a tf2 time-travel lookup
at header.stamp valid -- it's a same-domain lookup, so the TF buffer and the
detection message agree on "when," even though neither agrees with the
system clock. (Rear webcam /image_raw's domain was NOT verified live --
found a separate, pre-existing issue where usb_cam_node_exe holds /dev/video0
but currently has 0 publishers on /image_raw; standard usb_cam behavior
stamps with the ROS node clock i.e. system time, which would put it in a
DIFFERENT domain than the front camera. Doesn't affect correctness here since
stamps are never mixed across sources, but flag if fixing the rear camera
feed and worth a live check at that point.)

This node is deliberately built to NEVER do that now()-vs-header.stamp
subtraction:
  - The tf2 TIME-TRAVEL LOOKUP uses header.stamp directly against the TF
    buffer, which is self-consistent as long as the TF broadcaster (the same
    ODIN driver) stamps its transforms in that same domain -- a same-domain
    lookup is correct regardless of what that domain's absolute offset from
    ROS time is.
  - All AGE / CONFIDENCE / dt bookkeeping (age_sec, the alpha-beta predict
    step, detected-vs-coasting) is instead computed from LOCAL monotonic
    time at MESSAGE ARRIVAL, never from header.stamp deltas. This sacrifices
    a little precision (arrival time != capture time by the pipeline
    latency, ~tens of ms) but is immune to the clock-domain hazard above,
    while the actual position fix (the point of this whole refactor) still
    comes from the correctly-timed TF lookup.
  - If the time-travel lookup itself throws (extrapolation/connectivity
    error -- e.g. TF buffer doesn't go back far enough, or the domain
    assumption above is wrong), this node falls back to the latest
    available transform and logs a throttled warning. A live test should
    watch for that warning firing often; if it does, the capture-time TF
    lookup is silently degrading to current-pose behavior and the clock
    domain assumption needs to be revisited (report this if seen).

--------------------------------------------------------------------------
MEASUREMENT TRUST (front vs rear)
--------------------------------------------------------------------------
Front (ODIN) distance comes from a real depth-sensor lookup -- trustworthy.
Rear (webcam) distance is a bbox-height heuristic -- materially noisier.
Bearing (pixel offset -> angle) is comparably reliable on both cameras (pure
geometry, not a heuristic), so the difference is modeled as an overall
correction-strength difference per source rather than per-axis: REAR_ALPHA/
REAR_BETA are set well below FRONT_ALPHA/BETA so a rear measurement nudges
the filter gently instead of snapping to a noisy bbox-derived point.

--------------------------------------------------------------------------
INTERFACE COMPATIBILITY
--------------------------------------------------------------------------
Publishes the same /target_detected (Bool) /target_bearing (Float32)
/target_distance (Float32) topic set target_selector used to produce,
derived from the FILTERED estimate re-projected through the robot's CURRENT
pose (this is fine -- re-projecting a stable world point through a moving
current pose is exactly what "bearing to a fixed point" should do; the bug
was fusing the MEASUREMENT that way, not re-deriving a relative bearing for
display/control). target_pid and intent_to_goal need no changes.
target_selector_node.py is no longer used (superseded, its mode-selection
job is absorbed here) but is left in the package as a documented fallback,
same pattern already used for target_pid vs Nav2 and the old ArUco trackers.

Also publishes the richer /target_state (robot_behavior_msgs/TargetState)
for behavior_state_machine_node (Part B): filtered world-frame pose +
velocity + confidence + age-since-last-real-measurement.
"""

import time
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

import tf2_ros

from robot_behavior_msgs.msg import TargetDetection, TargetState
from robot_behavior.intent_to_goal_node import (
    FOLLOW_BEARING_TAN_RATIO,
    FLEE_BEARING_TAN_RATIO,
    FLEE_LATERAL_SIGN,
)
from robot_behavior.mode_constants import (
    DEFAULT_TRACK_MODE,
    TOPIC_TRACK_MODE_STATE,
    latched_qos,
)

# --- Tunable constants ---------------------------------------------------
ESTIMATOR_RATE_HZ = 30.0  # requirement: 30Hz or fastest reliably achievable

GLOBAL_FRAME = 'odom'
ROBOT_BASE_FRAME = 'odin1_base_link'  # same assumption intent_to_goal makes
                                       # for the rear camera too -- no
                                       # separate rear-camera TF frame exists.

# Correction strength (alpha = position gain, beta = velocity gain per
# second of the correction's own dt) per source -- see module docstring for
# why bearing isn't split out separately from distance here.
FRONT_ALPHA = 0.85
FRONT_BETA = 0.6
REAR_ALPHA = 0.35
REAR_BETA = 0.15

# Legacy target_detected stays True through TRACKING *and* COASTING (fresh
# measurement or brief dropout) -- this is what makes "coast through a 0.3s
# occlusion without interrupting motion" work for target_pid/intent_to_goal
# with NO changes on their side. Matches Part B's COASTING window; kept as
# its own parameter here (not imported from behavior_state_machine) since
# this node must not depend on that one.
COAST_WINDOW_SEC = 0.5

# Confidence decays linearly from 1.0 at age=COAST_WINDOW_SEC to 0.0 at this
# age, then clamps at 0. Generously beyond COAST_WINDOW_SEC on purpose --
# confidence is a smooth telemetry signal for behavior_state_machine to use
# as it likes, not itself a hard state-transition boundary (that logic lives
# in Part B, using age_sec directly against its own per-mode timeouts).
CONFIDENCE_FLOOR_SEC = 5.0

TF_LOOKUP_TIMEOUT_SEC = 0.15
# Floor on the correction dt (see _fuse). Raised from an earlier 1e-3: that
# floor was found live to still let the velocity-correction term
# (beta/dt * residual) spike to tens of m/s whenever two corrections landed
# only a few ms apart, and since velocity is never damped, that spike then
# free-runs the position estimate forever via the predict step once
# detections stop -- observed live growing to ~1e132 within seconds of a bad
# correction. 0.02s (50Hz-equivalent) is comfortably below any real tracker's
# frame interval (~33-50ms), so it only clips genuine near-simultaneous
# double-fires, not normal operation.
MIN_CORRECTION_DT = 0.02
# Defense in depth alongside the dt floor above: no real person moves faster
# than this, so any correction implying more is treated as a bad
# measurement/residual, not real motion, and clamped rather than integrated.
MAX_TARGET_SPEED_MPS = 3.0
# Divergence guard: if the estimate ever ends up further than this from the
# robot (nonsense for an indoor/short-range tracker) or non-finite, something
# upstream produced a bad correction -- reset rather than let a corrupted
# state free-run indefinitely (see MIN_CORRECTION_DT comment).
MAX_PLAUSIBLE_RANGE_M = 30.0
# -------------------------------------------------------------------------


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class TargetStateEstimatorNode(Node):

    def __init__(self):
        super().__init__('target_state_estimator_node')

        self.declare_parameter('estimator_rate_hz', ESTIMATOR_RATE_HZ)
        self.declare_parameter('coast_window_sec', COAST_WINDOW_SEC)
        self.declare_parameter('confidence_floor_sec', CONFIDENCE_FLOOR_SEC)
        self.declare_parameter('front_alpha', FRONT_ALPHA)
        self.declare_parameter('front_beta', FRONT_BETA)
        self.declare_parameter('rear_alpha', REAR_ALPHA)
        self.declare_parameter('rear_beta', REAR_BETA)
        self.declare_parameter('global_frame', GLOBAL_FRAME)
        self.declare_parameter('robot_base_frame', ROBOT_BASE_FRAME)

        self.estimator_rate_hz = self.get_parameter('estimator_rate_hz').value
        self.coast_window_sec = self.get_parameter('coast_window_sec').value
        self.confidence_floor_sec = self.get_parameter('confidence_floor_sec').value
        self.front_alpha = self.get_parameter('front_alpha').value
        self.front_beta = self.get_parameter('front_beta').value
        self.rear_alpha = self.get_parameter('rear_alpha').value
        self.rear_beta = self.get_parameter('rear_beta').value
        self.global_frame = self.get_parameter('global_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value

        self.track_mode = DEFAULT_TRACK_MODE

        # Filter state (odom frame). Uninitialized until the first real
        # measurement -- see _fuse().
        self._initialized = False
        self._x = 0.0
        self._y = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._last_predict_mono = time.monotonic()
        self._last_measurement_mono = None  # None == never measured
        self._last_correction_mono = None   # None == no prior correction to dt against

        # Achieved-rate reporting (final-summary requirement).
        self._tick_count = 0
        self._rate_report_start = time.monotonic()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            String, TOPIC_TRACK_MODE_STATE, self._track_mode_cb, latched_qos())
        self.create_subscription(
            TargetDetection, '/odin_target_detection_raw', self._front_cb, 10)
        self.create_subscription(
            TargetDetection, '/rear_target_detection_raw', self._rear_cb, 10)

        # Interface-compatible legacy outputs (target_selector's old job).
        self.detected_pub = self.create_publisher(Bool, '/target_detected', 10)
        self.bearing_pub = self.create_publisher(Float32, '/target_bearing', 10)
        self.distance_pub = self.create_publisher(Float32, '/target_distance', 10)
        # Rich state for behavior_state_machine_node (Part B).
        self.state_pub = self.create_publisher(TargetState, '/target_state', 10)

        self.create_timer(1.0 / self.estimator_rate_hz, self._tick)
        self.create_timer(5.0, self._report_rate)

        self.get_logger().info(
            f'target_state_estimator_node started: rate={self.estimator_rate_hz}Hz, '
            f'coast_window={self.coast_window_sec}s, '
            f'front(alpha={self.front_alpha},beta={self.front_beta}), '
            f'rear(alpha={self.rear_alpha},beta={self.rear_beta})')

    # --- mode ---
    def _track_mode_cb(self, msg: String):
        self.track_mode = msg.data.strip().lower()

    # --- raw detection callbacks ---
    def _front_cb(self, msg: TargetDetection):
        if self.track_mode != 'follow' or not msg.detected:
            return
        self._fuse(msg, FOLLOW_BEARING_TAN_RATIO, self.front_alpha, self.front_beta,
                   is_flee=False)

    def _rear_cb(self, msg: TargetDetection):
        if self.track_mode != 'flee' or not msg.detected:
            return
        self._fuse(msg, FLEE_BEARING_TAN_RATIO, self.rear_alpha, self.rear_beta,
                   is_flee=True)

    def _lookup_pose_at(self, stamp):
        """Return (x, y, yaw) of robot_base_frame in global_frame at `stamp`
        (a builtin_interfaces Time), or None if TF truly isn't available at
        all (falls back to the LATEST transform on an extrapolation/
        connectivity error rather than dropping the measurement -- see
        module docstring's clock-domain caveat)."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_base_frame, stamp,
                timeout=Duration(seconds=TF_LOOKUP_TIMEOUT_SEC))
        except (tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.global_frame, self.robot_base_frame, rclpy.time.Time(),
                    timeout=Duration(seconds=TF_LOOKUP_TIMEOUT_SEC))
                self.get_logger().warn(
                    'Capture-time TF lookup failed, fell back to latest transform '
                    '-- if this fires often, the capture-time fix is not actually '
                    'taking effect (see module docstring clock-domain caveat).',
                    throttle_duration_sec=5.0)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as exc:
                self.get_logger().warn(f'TF unavailable: {exc}', throttle_duration_sec=2.0)
                return None
        except tf2_ros.LookupException as exc:
            self.get_logger().warn(f'TF unavailable: {exc}', throttle_duration_sec=2.0)
            return None

        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return rx, ry, yaw

    def _measurement_world_point(self, msg: TargetDetection, ratio, is_flee, rx, ry, yaw):
        """Project a raw (bearing, distance) detection into a world-frame
        (x, y) point, using the robot pose AT CAPTURE TIME (rx, ry, yaw --
        already looked up by the caller). Mirrors intent_to_goal_node's own
        follow/flee base-frame projections so the two stay geometrically
        consistent."""
        theta = math.atan(msg.bearing * ratio)
        if not is_flee:
            tx_base = msg.distance * math.cos(theta)
            ty_base = -msg.distance * math.sin(theta)
        else:
            # Threat is BEHIND the robot (rear camera); mirrors
            # intent_to_goal's away-direction convention (see that module's
            # flee comment) but for the threat's OWN position, not the
            # away-goal: away ~= (cos, -flee_sign*sin), so the threat itself
            # sits at the negation of that direction, scaled by distance.
            tx_base = -msg.distance * math.cos(theta)
            ty_base = FLEE_LATERAL_SIGN * msg.distance * math.sin(theta)

        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        wx = rx + cos_y * tx_base - sin_y * ty_base
        wy = ry + sin_y * tx_base + cos_y * ty_base
        return wx, wy

    def _fuse(self, msg: TargetDetection, ratio, alpha, beta, is_flee):
        pose = self._lookup_pose_at(msg.header.stamp)
        if pose is None:
            return  # no usable TF at all yet; drop this measurement
        rx, ry, yaw = pose
        wx, wy = self._measurement_world_point(msg, ratio, is_flee, rx, ry, yaw)

        now_mono = time.monotonic()
        self._predict(now_mono)  # bring state up to "now" before correcting

        if not self._initialized:
            # First-ever measurement: snap directly rather than blending
            # against an arbitrary (0, 0) origin, which would otherwise read
            # as a huge, fake initial velocity.
            self._x, self._y = wx, wy
            self._vx = self._vy = 0.0
            self._initialized = True
        else:
            # dt against the dedicated correction clock (_last_correction_mono),
            # NOT _last_predict_mono -- that one is also touched by the 30Hz
            # timer's own _predict() calls (including the one right above),
            # so using it here could see a near-zero dt whenever a detection
            # callback happens to run moments after a timer tick, blowing up
            # beta/dt. See MIN_CORRECTION_DT comment for what that looked
            # like live (estimate diverging to ~1e132 within seconds).
            dt = max(MIN_CORRECTION_DT, now_mono - self._last_correction_mono)
            res_x = wx - self._x
            res_y = wy - self._y
            self._x += alpha * res_x
            self._y += alpha * res_y
            self._vx = clamp(self._vx + (beta / dt) * res_x, -MAX_TARGET_SPEED_MPS, MAX_TARGET_SPEED_MPS)
            self._vy = clamp(self._vy + (beta / dt) * res_y, -MAX_TARGET_SPEED_MPS, MAX_TARGET_SPEED_MPS)

        self._last_measurement_mono = now_mono
        self._last_correction_mono = now_mono
        self._check_divergence()

    def _predict(self, now_mono):
        dt = max(0.0, now_mono - self._last_predict_mono)
        if self._initialized:
            self._x += self._vx * dt
            self._y += self._vy * dt
        self._last_predict_mono = now_mono

    def _check_divergence(self):
        """Defense in depth alongside the velocity clamp in _fuse(): reset
        the filter rather than let a corrupted state free-run forever if it
        ever goes non-finite or implausibly far from the robot (indoor/
        short-range tracker -- there is no legitimate reading this far out).
        Runs after every correction AND every predict tick, since a bad
        velocity alone (pre-clamp bug, or a future change that reintroduces
        one) would otherwise keep growing position indefinitely with no
        further corrections to catch it."""
        if not self._initialized:
            return
        bad = (not math.isfinite(self._x) or not math.isfinite(self._y)
               or not math.isfinite(self._vx) or not math.isfinite(self._vy)
               or math.hypot(self._x, self._y) > MAX_PLAUSIBLE_RANGE_M)
        if bad:
            self.get_logger().error(
                f'target_state_estimator diverged (x={self._x}, y={self._y}, '
                f'vx={self._vx}, vy={self._vy}) -- resetting filter, will '
                're-initialize on the next real measurement.')
            self._initialized = False
            self._x = self._y = self._vx = self._vy = 0.0
            self._last_measurement_mono = None
            self._last_correction_mono = None

    # --- main loop ---
    def _tick(self):
        self._tick_count += 1
        now_mono = time.monotonic()
        self._predict(now_mono)
        self._check_divergence()

        if self._last_measurement_mono is None:
            age_sec = float('inf')
        else:
            age_sec = now_mono - self._last_measurement_mono

        if age_sec <= self.coast_window_sec:
            confidence = 1.0
        elif age_sec >= self.confidence_floor_sec:
            confidence = 0.0
        else:
            span = self.confidence_floor_sec - self.coast_window_sec
            confidence = 1.0 - (age_sec - self.coast_window_sec) / span

        detected = self._initialized and age_sec <= self.coast_window_sec

        # Re-project the filtered world point through the robot's CURRENT
        # pose for the legacy bearing/distance outputs -- this is the
        # "re-derive a relative bearing for display/control from a stable
        # world point" step, not the measurement-fusion step, so using the
        # current (not capture-time) pose here is correct.
        bearing = 0.0
        distance = 0.0
        if self._initialized:
            pose_now = self._lookup_pose_at(rclpy.time.Time())
            if pose_now is not None:
                rx, ry, yaw = pose_now
                dx, dy = self._x - rx, self._y - ry
                distance = math.hypot(dx, dy)
                if distance > 1e-3:
                    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
                    tx_base = cos_y * dx + sin_y * dy
                    ty_base = -sin_y * dx + cos_y * dy
                    ratio = FLEE_BEARING_TAN_RATIO if self.track_mode == 'flee' else FOLLOW_BEARING_TAN_RATIO
                    if self.track_mode == 'flee':
                        # Invert the flee projection in _measurement_world_point.
                        theta = math.atan2(ty_base / FLEE_LATERAL_SIGN, -tx_base)
                    else:
                        theta = math.atan2(-ty_base, tx_base)
                    bearing = clamp(math.tan(theta) / ratio, -1.0, 1.0)

        self.detected_pub.publish(Bool(data=bool(detected)))
        self.bearing_pub.publish(Float32(data=float(bearing)))
        self.distance_pub.publish(Float32(data=float(distance)))

        state_msg = TargetState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.header.frame_id = self.global_frame
        state_msg.detected = bool(detected)
        state_msg.x = float(self._x)
        state_msg.y = float(self._y)
        state_msg.vx = float(self._vx)
        state_msg.vy = float(self._vy)
        state_msg.confidence = float(confidence)
        state_msg.age_sec = float(age_sec) if age_sec != float('inf') else 1e9
        state_msg.track_mode = self.track_mode
        self.state_pub.publish(state_msg)

    def _report_rate(self):
        elapsed = time.monotonic() - self._rate_report_start
        if elapsed > 0:
            achieved = self._tick_count / elapsed
            self.get_logger().info(f'target_state_estimator achieved rate: {achieved:.1f}Hz '
                                    f'(target {self.estimator_rate_hz}Hz)')
        self._tick_count = 0
        self._rate_report_start = time.monotonic()


def main(args=None):
    rclpy.init(args=args)
    node = TargetStateEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
