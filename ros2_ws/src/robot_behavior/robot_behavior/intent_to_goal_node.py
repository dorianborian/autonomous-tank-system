"""Phase 7 intent_to_goal bridge.

Turns the mode-aware target intent (from target_selector: /target_detected,
/target_bearing, /target_distance) into a Nav2 *path* that controller_server
(DWB) follows with local obstacle avoidance, instead of the old direct-velocity
target_pid. This is the node that lets Nav2 route AROUND obstacles between the
robot and the marker (the whole point of Phase 7).

--------------------------------------------------------------------------
MOVING-TARGET GOAL MECHANISM (the real design decision, Phase 7 step 6)
--------------------------------------------------------------------------
Nav2's NavigateToPose action assumes a one-shot, static goal reached via a
global planner. Our target MOVES continuously and we have NO global costmap.
So instead:

  * We talk directly to controller_server's FollowPath action.
  * On a timer (UPDATE_RATE_HZ, deliberately modest) we recompute the goal
    from the LATEST target reading + current TF, build a Path (robot ->
    goal) in the odom frame, and send it as a *new* FollowPath goal.
  * Sending a new FollowPath goal NATIVELY PREEMPTS the previous one — the
    action server aborts the old goal and accepts the newest. This is why we
    do NOT cancel-then-resend: preemption is the clean built-in pattern for a
    continuously updated goal, and the robot never stops between updates.
  * The Path itself is built by _build_path() using costmap_planner.astar_path
    — a real obstacle-aware grid search over the local costmap (see the
    JOYSTICK GOAL AUTHORING section below for why this replaced an earlier
    naive straight line: DWB alone couldn't route around a real obstacle
    given only a straight reference path through it). A real global planner
    still isn't needed — the local costmap + this A* search is enough for
    local follow/flee/goal-seeking.
  * The downstream velocity_smoother absorbs the tiny discontinuity when the
    path is swapped, so path swaps don't show up as jerks.

UPDATE_RATE_HZ is kept below the costmap/cloud update rate (~5Hz) so we don't
spam preemptions faster than the world model actually changes.

--------------------------------------------------------------------------
follow vs flee goal geometry
--------------------------------------------------------------------------
follow (front ODIN camera):
  Target is AHEAD. Goal = a point STANDOFF_M short of the target along the
  robot->target ray (so the robot settles ~STANDOFF_M from the marker). If the
  target is already closer than STANDOFF_M the goal collapses to the robot's
  current pose => hold. NOTE: unlike Phase 6's target_pid, we do NOT reverse
  when the target is too close. Reasons, flagged deliberately: (a) DWB is
  configured forward-only (min_vel_x=0) which is safer, and (b) the ODIN sensor
  only sees FORWARD, so backing up would be blind to obstacles. "Too close"
  therefore means hold, not retreat.

flee (rear webcam):
  Threat is BEHIND. Goal = a point FLEE_DISTANCE_M ahead of the robot in the
  direction directly AWAY from the threat, recomputed every tick so it keeps
  receding as both robot and threat move => continuous forward flight, never
  "arrives". The rear camera is mirror-flipped relative to the robot body, so
  the lateral sign of the away-direction is exposed as FLEE_LATERAL_SIGN and
  MUST be confirmed in live test 8d (flip it if the robot steers toward the
  threat instead of away).

--------------------------------------------------------------------------
bearing units
--------------------------------------------------------------------------
/target_bearing is NORMALIZED [-1, 1] (image-plane horizontal offset), NOT
radians. Real angle off the optical axis: theta = atan(bearing * ratio) where
ratio = (calibrated_image_width/2) / calibrated_focal_px. For the ODIN front
cam (1600px wide, 734.92px focal) ratio ~= 1.089 (=> ~+/-47deg at the edges).
The rear webcam has different optics; its ratio is a separate tunable.

ASSUMPTION (flagged): the camera optical axis is treated as aligned with the
robot base +x (forward). There is no measured camera->base extrinsic; if follow
consistently drives at a fixed angular offset, that extrinsic is the cause.

--------------------------------------------------------------------------
JOYSTICK GOAL AUTHORING (2026-07-14 redesign — lock/delete, real A* routing)
--------------------------------------------------------------------------
This replaced an earlier "continuously re-project a goal from the current
joystick Twist every tick" design (Phase 7.1) after two real problems showed
up in testing:

  1. The reference path handed to controller_server was a NAIVE STRAIGHT
     LINE from robot to goal. DWB (controller_server's local controller) is a
     trajectory SCORER/tracker, not a path-finder — its PathAlign/PathDist/
     GoalDist critics penalize departing from the given reference path, so
     when that line ran through a real obstacle, DWB had no way to invent a
     wide detour and just stalled at the obstacle edge ("terminates at the
     box"). Fixed by costmap_planner.astar_path(): a real obstacle-aware grid
     search over the local costmap, replacing the straight line with an
     actually-routed path.
  2. Recomputing the goal every tick from the joystick Twist + CURRENT robot
     pose meant the goal silently rotated/drifted with the robot's own
     heading. The instant DWB nudged the robot off-axis to avoid something,
     the NEXT tick re-aimed from that new (deviated) heading — so the goal
     never returned to the original line, it just kept chasing wherever the
     robot happened to be pointed. Fixed by decoupling "aiming" from
     "executing": the joystick now only moves a CANDIDATE preview goal
     (recomputed every tick, visualized, but never sent to controller_server)
     until explicitly LOCKED — a locked goal is a fixed, stable pose in odom
     that does not move again until reached, deleted, or a new one is locked.

Flow (semi mode, no marker detected — marker-based follow/flee is unchanged,
see below):
  - Unlocked: the joystick projects a candidate goal each tick (same
    unicycle-model projection as before), published to /candidate_goal_debug
    for visualization ONLY. Nothing is sent to controller_server — no motion
    planning happens while just aiming.
  - X (LOCK_BUTTON, verified real hardware = buttons[1], see
    ps_controller_bridge_node.py) freezes the current candidate as
    self._locked_goal_pose — a fixed odom pose, published to
    /intent_goal_debug (latched). From here every tick re-runs the A*
    planner from the robot's CURRENT pose to that FIXED goal and sends the
    resulting path via FollowPath (still using goal-preemption — a fresh
    path every tick is a new FollowPath goal, same pattern as before — but
    now the goal POINT itself never moves).
  - O (DELETE_BUTTON, verified real hardware = buttons[2]) clears the lock
    and cancels any active FollowPath.
  - Reaching the locked goal (within GOAL_REACHED_TOLERANCE_M) auto-clears
    the lock, same as an explicit delete.
  - The R2 deadman gate (drive_mux, /nav_enable) is UNCHANGED and still the
    only thing standing between a locked+planned goal and actual motor
    output — planning/routing happens regardless of R2, exactly so the goal
    and route stay visible on the costmap viewer at all times.

AUTO mode is UNCHANGED (still pure marker-only autonomy, no human input,
no lock/candidate concept — marker position is independently, continuously
measured each tick, so it never had the self-referential drift problem).
MANUAL mode is UNCHANGED (raw joystick straight through drive_mux, no Nav2
involvement).

nav2_collision_monitor was tried as an additional downstream safety net and
REMOVED same session, 2026-07-14 — see drive_mux_node.py CMD_VEL_OUT_TOPIC
comment. This node's DWB-routing behavior is unaffected by that removal;
e-stop and a physical kill switch are the safety net now.
"""

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration

import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import FollowPath
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                        QoSReliabilityPolicy)
from std_msgs.msg import Bool, Float32, String

from robot_behavior import costmap_planner
from robot_behavior.mode_constants import (
    DEFAULT_DRIVE_SOURCE,
    DEFAULT_TRACK_MODE,
    TOPIC_DRIVE_SOURCE_STATE,
    TOPIC_TRACK_MODE_STATE,
    latched_qos,
)

# --- Tunable constants -----------------------------------------------------
UPDATE_RATE_HZ = 4.0          # goal refresh rate; kept < costmap ~5Hz on purpose

GLOBAL_FRAME = 'odom'         # local costmap global_frame (Phase 7 step 1)
ROBOT_BASE_FRAME = 'odin1_base_link'

# bearing[-1,1] -> angle: theta = atan(bearing * ratio). ratio = (W/2)/focal.
FOLLOW_BEARING_TAN_RATIO = 800.0 / 734.92   # ODIN front cam ~1.089
FLEE_BEARING_TAN_RATIO = 1.0                 # rear webcam optics differ; TUNABLE

STANDOFF_M = 1.0              # follow: settle this far from the marker
FLEE_DISTANCE_M = 2.0         # flee: project the goal this far ahead each tick
FLEE_LATERAL_SIGN = 1.0       # rear-camera mirror; confirm/flip in test 8d

# Reported 2026-07-25: flee spins in place instead of progressing forward
# while evading. Root cause: nav2_params.yaml's DWB critic weights are a
# single SHARED config for follow AND flee, tuned around RotateToGoal.scale
# =32.0 -- DWB's RotateToGoal critic explicitly biases toward pure-rotation
# trajectories whenever heading error to the current goal is large, i.e.
# "rotate fully in place, THEN translate". That is fine for follow's mostly-
# static standoff goal, but flee's goal direction is recomputed from a live,
# continuously-moving bearing every tick (UPDATE_RATE_HZ) -- the target
# heading keeps shifting before the robot ever finishes rotating into it, so
# the robot never gets to the "aligned enough to translate" state and just
# keeps re-rotating: the observed spin. Fix: drop RotateToGoal's influence
# to ~0 specifically while track_mode=='flee' (SetParameters against
# controller_server, see _apply_rotate_to_goal_scale), so DWB falls back to
# PathAlign/PathDist/GoalAlign/GoalDist -- which still favor turning toward
# the goal, just blended smoothly with translation instead of gating it --
# letting the robot curve while it drives, which is also what continuously
# keeps the person centered in the rear camera's dead zone rather than
# requiring a full stop-and-realign. follow's value is unchanged from
# nav2_params.yaml's static default so follow's tuning/behavior is
# unaffected by this.
FOLLOW_ROTATE_TO_GOAL_SCALE = 32.0
FLEE_ROTATE_TO_GOAL_SCALE = 0.0

PATH_POINT_SPACING_M = 0.1    # interpolation spacing for the straight-line path
MIN_GOAL_DISTANCE_M = 0.05    # below this, treat follow goal as "hold in place"

# Joystick-intent goal projection (semi mode, no marker — see module docstring).
JOYSTICK_LOOKAHEAD_SEC = 2.0        # "if this command held for T sec, where do you end up"
JOYSTICK_MIN_LINEAR_MPS = 0.08      # stick linear below this = neutral, no goal
# NOTE on scale: ps_controller_bridge multiplies the raw [-1,1] stick by
# max_angular_z=4.0 rad/s. A threshold in rad/s therefore has to be judged
# against THAT range, not against [-1,1] — 0.10 rad/s (the original value
# here) is only 2.5% of full deflection, so ordinary human imprecision
# pushing "straight forward" (a few % of unintended cross-axis stick
# movement) already cleared it, spawning an arc instead of a straight goal,
# and idle stick jitter could intermittently cross it too (spurious goals at
# rest). Raised to a value that actually requires a deliberate turn — TUNABLE,
# raise further if arcs still appear on straight pushes, lower if real gentle
# turns stop registering.
JOYSTICK_MIN_ANGULAR_RPS = 0.50     # ~12.5% of the 4.0 rad/s max — see note above
JOYSTICK_GOAL_MIN_DISTANCE_M = 0.30  # don't bother sending a goal closer than this —
                                     # not enough room for DWB to plan a meaningful curve
JOYSTICK_GOAL_MAX_DISTANCE_M = 1.8   # cap projected distance so the goal stays well
                                     # inside the local costmap's rolling window (4m,
                                     # robot centered => 2m half-width, see nav2_params.yaml)
JOYSTICK_CMD_STALE_SEC = 0.3         # matches drive_mux's own staleness handling
# Exponential moving average applied to the raw joystick Twist as it arrives
# (in _manual_cb, every message — not just at the 4Hz goal-tick), so a single
# noisy/spiky sample landing exactly on a tick can't spuriously cross the
# neutral or "intentional turn" thresholds above. Lower alpha = more smoothing
# (slower to respond); TUNABLE — raise toward 1.0 if the goal feels laggy
# behind real stick movement, lower if it's still twitchy.
JOYSTICK_EMA_ALPHA = 0.3

# Goal-locking (see module docstring). Reaching the locked goal within this
# radius auto-clears the lock, same as pressing O. Slightly looser than
# controller_server's own xy_goal_tolerance (0.20m, nav2_params.yaml) since
# this is just "stop re-planning and free up the lock for a new goal", not a
# precision stopping requirement — DWB/the goal_checker handle the actual
# stop.
GOAL_REACHED_TOLERANCE_M = 0.25
# -----------------------------------------------------------------------


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class IntentToGoalNode(Node):

    def __init__(self):
        super().__init__('intent_to_goal_node')

        self.declare_parameter('update_rate_hz', UPDATE_RATE_HZ)
        self.declare_parameter('standoff_m', STANDOFF_M)
        self.declare_parameter('flee_distance_m', FLEE_DISTANCE_M)
        self.declare_parameter('flee_lateral_sign', FLEE_LATERAL_SIGN)
        self.declare_parameter('follow_bearing_tan_ratio', FOLLOW_BEARING_TAN_RATIO)
        self.declare_parameter('flee_bearing_tan_ratio', FLEE_BEARING_TAN_RATIO)
        self.declare_parameter('global_frame', GLOBAL_FRAME)
        self.declare_parameter('robot_base_frame', ROBOT_BASE_FRAME)
        self.declare_parameter('follow_rotate_to_goal_scale', FOLLOW_ROTATE_TO_GOAL_SCALE)
        self.declare_parameter('flee_rotate_to_goal_scale', FLEE_ROTATE_TO_GOAL_SCALE)

        self.update_rate_hz = self.get_parameter('update_rate_hz').value
        self.standoff_m = self.get_parameter('standoff_m').value
        self.flee_distance_m = self.get_parameter('flee_distance_m').value
        self.flee_lateral_sign = self.get_parameter('flee_lateral_sign').value
        self.follow_ratio = self.get_parameter('follow_bearing_tan_ratio').value
        self.flee_ratio = self.get_parameter('flee_bearing_tan_ratio').value
        self.global_frame = self.get_parameter('global_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.follow_rotate_to_goal_scale = self.get_parameter('follow_rotate_to_goal_scale').value
        self.flee_rotate_to_goal_scale = self.get_parameter('flee_rotate_to_goal_scale').value

        # SetParameters client for controller_server's DWB critic scale --
        # see FLEE_ROTATE_TO_GOAL_SCALE comment above. Applied on every
        # track_mode change (_track_mode_cb) AND on a slow timer (self-heals
        # if controller_server was restarted after this node, reverting to
        # nav2_params.yaml's static default in the meantime).
        from rcl_interfaces.srv import SetParameters
        self._controller_param_client = self.create_client(
            SetParameters, '/controller_server/set_parameters')
        self.create_timer(2.0, self._apply_rotate_to_goal_scale)

        # Mode + target state
        self.drive_source = DEFAULT_DRIVE_SOURCE
        self.track_mode = DEFAULT_TRACK_MODE
        self.target_detected = False
        self.target_bearing = 0.0
        self.target_distance = 0.0

        # Joystick intent (semi mode, no marker — see module docstring).
        # _manual_smooth_{lin,ang} are EMA-filtered versions of the raw stick
        # Twist, updated on every /manual_cmd_vel message (see _manual_cb).
        self._manual_t = 0.0
        self._manual_smooth_lin = 0.0
        self._manual_smooth_ang = 0.0

        # Goal-locking state (see module docstring). candidate = unlocked
        # preview, recomputed every tick from the joystick, visualization
        # only. locked = fixed odom pose, frozen on X, cleared on O/reached.
        self._candidate_goal_pose = None
        self._locked_goal_pose = None

        # Latest local costmap, for A* planning (costmap_planner.astar_path).
        self._costmap = None

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Active FollowPath goal handle (for preemption / cancel bookkeeping)
        self._goal_handle = None
        self._active = False   # are we currently commanding Nav2?

        # Subscriptions (mode topics latched, per the Phase 6 QoS rule)
        self.create_subscription(
            String, TOPIC_DRIVE_SOURCE_STATE, self._drive_source_cb, latched_qos())
        self.create_subscription(
            String, TOPIC_TRACK_MODE_STATE, self._track_mode_cb, latched_qos())
        self.create_subscription(Bool, '/target_detected', self._detected_cb, 10)
        self.create_subscription(Float32, '/target_bearing', self._bearing_cb, 10)
        self.create_subscription(Float32, '/target_distance', self._distance_cb, 10)
        self.create_subscription(Twist, '/manual_cmd_vel', self._manual_cb, 10)
        self.create_subscription(String, '/goal_lock_cmd', self._goal_lock_cmd_cb, 10)
        costmap_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self._costmap_cb, costmap_qos)

        # FollowPath action client -> controller_server (DWB)
        self._follow_path_client = ActionClient(self, FollowPath, '/follow_path')

        # Debug: publish goal poses so they can be seen on the costmap viewer
        # and by the test monitor. Latched so a late viewer still shows the
        # last value. /intent_goal_debug = the ACTIVE goal being executed
        # (marker-based, or a locked joystick goal). /candidate_goal_debug =
        # the unlocked joystick preview (not yet being executed).
        self._goal_debug_pub = self.create_publisher(
            PoseStamped, '/intent_goal_debug', latched_qos())
        self._candidate_debug_pub = self.create_publisher(
            PoseStamped, '/candidate_goal_debug', latched_qos())

        self.create_timer(1.0 / self.update_rate_hz, self._tick)

        self.get_logger().info(
            'intent_to_goal_node started: '
            f'standoff={self.standoff_m}m, flee_distance={self.flee_distance_m}m, '
            f'joystick_lookahead={JOYSTICK_LOOKAHEAD_SEC}s, update_rate={self.update_rate_hz}Hz. '
            'auto=marker-only, semi=marker-if-detected-else-joystick '
            '(aim+X to lock, O to delete), manual=never. Paths are A*-planned '
            'over the local costmap (costmap_planner), not naive straight lines.')

    # --- callbacks ---
    def _drive_source_cb(self, msg):
        self.drive_source = msg.data.strip().lower()

    def _track_mode_cb(self, msg):
        self.track_mode = msg.data.strip().lower()
        self._apply_rotate_to_goal_scale()

    def _apply_rotate_to_goal_scale(self):
        """Push the mode-appropriate FollowPath.RotateToGoal.scale to
        controller_server -- see FLEE_ROTATE_TO_GOAL_SCALE module comment.
        Idempotent/cheap; called on every track_mode change for
        responsiveness and on a slow timer to self-heal if
        controller_server wasn't up yet or got restarted."""
        if not self._controller_param_client.service_is_ready():
            return
        target = (self.flee_rotate_to_goal_scale if self.track_mode == 'flee'
                  else self.follow_rotate_to_goal_scale)
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
        from rcl_interfaces.srv import SetParameters
        req = SetParameters.Request()
        req.parameters = [Parameter(
            name='FollowPath.RotateToGoal.scale',
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(target)))]
        self._controller_param_client.call_async(req)

    def _detected_cb(self, msg):
        self.target_detected = bool(msg.data)

    def _bearing_cb(self, msg):
        self.target_bearing = float(msg.data)

    def _distance_cb(self, msg):
        self.target_distance = float(msg.data)

    def _manual_cb(self, msg):
        # EMA filter every incoming sample (not just at the 4Hz goal tick) so
        # a single noisy/spiky reading can't spuriously cross the neutral or
        # intentional-turn thresholds used in _compute_joystick_goal_pose.
        a = JOYSTICK_EMA_ALPHA
        self._manual_smooth_lin = a * msg.linear.x + (1.0 - a) * self._manual_smooth_lin
        self._manual_smooth_ang = a * msg.angular.z + (1.0 - a) * self._manual_smooth_ang
        self._manual_t = time.monotonic()

    def _costmap_cb(self, msg):
        self._costmap = msg

    def _goal_lock_cmd_cb(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == 'lock':
            if self.drive_source != 'semi' or self.target_detected:
                self.get_logger().info(
                    'Lock (X) ignored: only applies in semi mode with no marker '
                    f'detected (drive_source={self.drive_source}, '
                    f'target_detected={self.target_detected}).')
                return
            if self._candidate_goal_pose is None:
                self.get_logger().info('Lock (X) ignored: no candidate goal '
                                       '(stick is neutral, nothing to lock).')
                return
            self._locked_goal_pose = self._candidate_goal_pose
            self._goal_debug_pub.publish(self._locked_goal_pose)
            self.get_logger().info(
                f'Goal LOCKED at odom ({self._locked_goal_pose.pose.position.x:.2f}, '
                f'{self._locked_goal_pose.pose.position.y:.2f}) — hold R2 to drive there.')
        elif cmd == 'delete':
            if self._locked_goal_pose is not None:
                self.get_logger().info('Goal DELETED (O).')
            self._locked_goal_pose = None
            if self._active:
                self._stop()

    # --- main loop ---
    def _tick(self):
        # auto: pure marker autonomy, unchanged, no human input by design.
        # semi + marker detected: unchanged marker-based follow/flee (goal
        #   recomputed every tick, but from an independently-measured marker
        #   position, not the robot's own heading — no drift problem there).
        # semi + no marker: joystick GOAL AUTHORING (candidate/lock/delete —
        #   see module docstring). Handled separately below since it has its
        #   own state machine, not a single "compute a goal, send it" step.
        # manual: never touches Nav2 (unchanged, raw joystick straight
        #   through drive_mux).
        if self.drive_source == 'auto':
            use_marker = self.target_detected
        elif self.drive_source == 'semi':
            use_marker = self.target_detected
        else:
            use_marker = False
            self._candidate_goal_pose = None  # not aiming, clear stale preview

        if self.drive_source == 'semi' and not use_marker:
            self._tick_joystick_goal_authoring()
            return

        if not use_marker:
            # Nothing to command Nav2 with right now -> make sure it isn't
            # still driving from a stale goal. Cancelling lets controller_server
            # settle to zero, the smoother decays, drive_mux passes a stop.
            if self._active:
                self._stop()
            return

        if self.target_distance <= 0.0:
            return  # no usable range this frame; hold whatever was last sent
        goal_pose = self._compute_goal_pose()
        if goal_pose is None:
            if self._active:
                self._stop()
            return
        self._send_path_to_goal(goal_pose)

    def _tick_joystick_goal_authoring(self):
        """semi mode, no marker: candidate/lock/delete state machine (see
        module docstring). Called every tick instead of the generic
        compute-goal-then-send flow above, since aiming (unlocked) and
        executing (locked) are genuinely different behaviors, not just
        different goal sources."""
        if self._locked_goal_pose is None:
            # Unlocked: update the preview only, never send anything to
            # controller_server — no motion planning while just aiming.
            self._candidate_goal_pose = self._compute_joystick_goal_pose()
            if self._candidate_goal_pose is not None:
                self._candidate_debug_pub.publish(self._candidate_goal_pose)
            if self._active:
                self._stop()  # was executing before, e.g. goal just got deleted
            return

        # Locked: plan an obstacle-aware route from the CURRENT pose to the
        # FIXED locked goal every tick, and send it. The goal point itself
        # never moves — only the resulting path adapts as the robot/costmap
        # change, exactly what fixes the "goal drifts with heading" problem.
        pose_now = self._get_robot_pose()
        if pose_now is None:
            return  # TF not ready; hold, try again next tick
        rx, ry, robot_yaw = pose_now

        gx = self._locked_goal_pose.pose.position.x
        gy = self._locked_goal_pose.pose.position.y
        if math.hypot(gx - rx, gy - ry) < GOAL_REACHED_TOLERANCE_M:
            self.get_logger().info('Locked goal reached — clearing.')
            self._locked_goal_pose = None
            self._stop()
            return

        # _send_path_to_goal -> _build_path reads self._robot_xy/_robot_yaw
        # (the same stashed-pose pattern _base_point_to_goal_pose uses for
        # the marker case) — set them explicitly here since the locked goal
        # itself isn't recomputed from the robot's current pose this tick.
        self._robot_xy = (rx, ry)
        self._robot_yaw = robot_yaw
        self._send_path_to_goal(self._locked_goal_pose)

    def _get_robot_pose(self):
        """Return (x, y, yaw) of the robot in the global (odom) frame, or None
        if TF isn't available yet. Shared by marker- and joystick-sourced
        goal computation."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_base_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            self.get_logger().warn(f'TF {self.global_frame}<-{self.robot_base_frame} '
                                   f'unavailable: {exc}', throttle_duration_sec=2.0)
            return None
        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        q = tf.transform.rotation
        robot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return rx, ry, robot_yaw

    def _base_point_to_goal_pose(self, gx_base, gy_base, rx, ry, robot_yaw):
        """Transform a point in the robot base frame to a PoseStamped goal in
        odom, facing the direction from the robot to that point."""
        cos_y, sin_y = math.cos(robot_yaw), math.sin(robot_yaw)
        gx = rx + cos_y * gx_base - sin_y * gy_base
        gy = ry + sin_y * gx_base + cos_y * gy_base
        goal_yaw = math.atan2(gy - ry, gx - rx) if (abs(gx - rx) + abs(gy - ry)) > 1e-3 else robot_yaw

        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = gx
        pose.pose.position.y = gy
        qx, qy, qz, qw = yaw_to_quat(goal_yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self._robot_xy = (rx, ry)
        self._robot_yaw = robot_yaw
        return pose

    def _compute_joystick_goal_pose(self):
        """Semi mode, no marker: project the current joystick Twist forward
        over JOYSTICK_LOOKAHEAD_SEC (unicycle model) to get a goal point, so
        DWB can plan an obstacle-aware path toward "roughly where the stick is
        pointing" instead of a raw, un-avoidant velocity command. Returns None
        if the stick is neutral or TF isn't ready (caller then holds/stops).
        Uses the EMA-smoothed stick values (see _manual_cb), not the raw
        instantaneous sample — avoids spurious goals/arcs from single-sample
        jitter or off-axis imprecision while pushing "straight" forward."""
        if (time.monotonic() - self._manual_t) > JOYSTICK_CMD_STALE_SEC:
            return None  # no recent joystick data
        lin = self._manual_smooth_lin
        ang = self._manual_smooth_ang
        if abs(lin) < JOYSTICK_MIN_LINEAR_MPS and abs(ang) < JOYSTICK_MIN_ANGULAR_RPS:
            return None  # stick neutral -> no goal, let auto/bias settle to zero

        pose_now = self._get_robot_pose()
        if pose_now is None:
            return None
        rx, ry, robot_yaw = pose_now

        T = JOYSTICK_LOOKAHEAD_SEC
        if abs(lin) < JOYSTICK_MIN_LINEAR_MPS:
            # Pure rotation command: no meaningful forward goal (in-place turns
            # don't need obstacle-aware path planning the way translation
            # does) -> no goal, semi's direct bias blend handles the turn.
            return None
        if abs(ang) < JOYSTICK_MIN_ANGULAR_RPS:
            # Below the "intentional turn" threshold -> treat as straight-line
            # forward, ignoring the residual angular component entirely. Using
            # the SAME threshold as the neutral gate (not a near-zero epsilon)
            # is what actually fixes "pushing forward doesn't go straight" —
            # real analog sticks essentially never read EXACTLY zero angular
            # while being pushed forward.
            gx_base, gy_base = lin * T, 0.0
        else:
            # Unicycle-model arc: where do you end up after T seconds of
            # constant (lin, ang)?
            theta = ang * T
            radius = lin / ang
            gx_base = radius * math.sin(theta)
            gy_base = radius * (1.0 - math.cos(theta))

        dist = math.hypot(gx_base, gy_base)
        if dist < 1e-6:
            return None
        # Clamp projected distance into [MIN, MAX] while preserving direction —
        # keeps the goal meaningful for DWB to plan toward, and inside the
        # local costmap's rolling window.
        clamped_dist = max(JOYSTICK_GOAL_MIN_DISTANCE_M,
                            min(dist, JOYSTICK_GOAL_MAX_DISTANCE_M))
        scale = clamped_dist / dist
        gx_base *= scale
        gy_base *= scale

        return self._base_point_to_goal_pose(gx_base, gy_base, rx, ry, robot_yaw)

    def _compute_goal_pose(self):
        """Return a PoseStamped goal in the global (odom) frame for the
        marker-based (follow/flee) intent source, or None if TF isn't ready."""
        pose_now = self._get_robot_pose()
        if pose_now is None:
            return None
        rx, ry, robot_yaw = pose_now

        d = self.target_distance

        if self.track_mode == 'flee':
            # Threat behind (rear cam faces -x_base). Away direction ~ forward.
            theta = math.atan(self.target_bearing * self.flee_ratio)
            # Away-from-threat unit vector in base frame: forward + mirrored lateral.
            ax = math.cos(theta)
            ay = -self.flee_lateral_sign * math.sin(theta)
            gx_base = self.flee_distance_m * ax
            gy_base = self.flee_distance_m * ay
        else:  # follow
            theta = math.atan(self.target_bearing * self.follow_ratio)
            # Target point in base frame (forward +x, right = -y).
            tx_base = d * math.cos(theta)
            ty_base = -d * math.sin(theta)
            standoff_d = d - self.standoff_m
            if standoff_d < MIN_GOAL_DISTANCE_M:
                # Too close: hold current pose (no reverse - see module docstring).
                gx_base, gy_base = 0.0, 0.0
            else:
                scale = standoff_d / d
                gx_base = tx_base * scale
                gy_base = ty_base * scale

        return self._base_point_to_goal_pose(gx_base, gy_base, rx, ry, robot_yaw)

    def _straight_line_path(self, rx, ry, goal_pose):
        """Naive straight-line Path — ONLY used as a startup-transient
        fallback when no costmap has arrived yet (see _build_path). Not
        obstacle-aware; this is what caused the original "terminates at the
        box" problem when it was the only path source, so it must never be
        used once a real costmap is available."""
        gx = goal_pose.pose.position.x
        gy = goal_pose.pose.position.y
        dist = math.hypot(gx - rx, gy - ry)
        seg_yaw = math.atan2(gy - ry, gx - rx) if dist > 1e-3 else self._robot_yaw
        qx, qy, qz, qw = yaw_to_quat(seg_yaw)

        path = Path()
        path.header.frame_id = self.global_frame
        path.header.stamp = self.get_clock().now().to_msg()

        n = max(2, int(dist / PATH_POINT_SPACING_M) + 1)
        for i in range(n):
            frac = i / (n - 1)
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x = rx + frac * (gx - rx)
            p.pose.position.y = ry + frac * (gy - ry)
            p.pose.orientation.x = qx
            p.pose.orientation.y = qy
            p.pose.orientation.z = qz
            p.pose.orientation.w = qw
            path.poses.append(p)
        path.poses[-1].pose.orientation = goal_pose.pose.orientation
        return path

    def _build_path(self, goal_pose):
        """Obstacle-aware Path (robot -> goal) via A* over the local costmap
        (costmap_planner.astar_path) — replaces the old naive straight line,
        which is what let DWB stall at an obstacle instead of routing around
        it (see module docstring). Returns None if the costmap has arrived
        but A* genuinely can't find a route (goal unreachable right now) —
        callers must NOT fall back to a straight line in that case, only when
        self._costmap is None (still starting up)."""
        rx, ry = self._robot_xy
        gx = goal_pose.pose.position.x
        gy = goal_pose.pose.position.y

        if self._costmap is None:
            self.get_logger().warn('No costmap yet — using straight-line path '
                                   'as a startup fallback (NOT obstacle-aware).',
                                   throttle_duration_sec=2.0)
            return self._straight_line_path(rx, ry, goal_pose)

        waypoints = costmap_planner.astar_path(self._costmap, (rx, ry), (gx, gy))
        if waypoints is None:
            self.get_logger().warn(
                'A* found no route to the goal — holding rather than driving blind.',
                throttle_duration_sec=2.0)
            return None

        path = Path()
        path.header.frame_id = self.global_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for i, (wx, wy) in enumerate(waypoints):
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x = wx
            p.pose.position.y = wy
            if i + 1 < len(waypoints):
                nx, ny = waypoints[i + 1]
                seg_yaw = math.atan2(ny - wy, nx - wx)
            else:
                seg_yaw = None  # filled in below with the goal's own heading
            if seg_yaw is not None:
                qx, qy, qz, qw = yaw_to_quat(seg_yaw)
                p.pose.orientation.x = qx
                p.pose.orientation.y = qy
                p.pose.orientation.z = qz
                p.pose.orientation.w = qw
            path.poses.append(p)
        path.poses[-1].pose.orientation = goal_pose.pose.orientation
        return path

    def _send_path_to_goal(self, goal_pose):
        if not self._follow_path_client.server_is_ready():
            # controller_server not up / not active yet.
            self.get_logger().warn('FollowPath server not ready',
                                   throttle_duration_sec=2.0)
            return

        path = self._build_path(goal_pose)
        if path is None:
            # A* explicitly couldn't find a route — do not send anything and
            # do not drive blind. Let the caller's existing goal (if any)
            # keep being tracked at the controller level, but stop issuing
            # new preempting goals until a route exists again.
            if self._active:
                self._stop()
            return

        self._goal_debug_pub.publish(goal_pose)   # for rviz + test monitor
        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = 'FollowPath'
        goal_msg.goal_checker_id = 'goal_checker'

        # Sending a new goal PREEMPTS the previous one (see module docstring).
        send_future = self._follow_path_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._on_goal_accepted)
        self._active = True

    def _on_goal_accepted(self, future):
        try:
            self._goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'FollowPath goal send failed: {exc}',
                                   throttle_duration_sec=2.0)

    def _stop(self):
        """Stop commanding Nav2: cancel the active FollowPath so the controller
        settles to zero (smoother decays, drive_mux passes a stop through)."""
        self._active = False
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
            self._goal_handle = None
        self.get_logger().info('intent_to_goal: stopped commanding Nav2 '
                               '(manual mode or target lost).')


def main(args=None):
    rclpy.init(args=args)
    node = IntentToGoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
