"""rclpy node backing the Phase 8 web UI: owns every ROS subscription/
publisher the FastAPI app needs, and exposes a thread-safe snapshot of
state for the HTTP/WebSocket handlers to read. Runs its own executor in a
background thread (see web_ui_node.py) so it never blocks uvicorn's event
loop and vice versa.
"""
import math
import subprocess
import threading
import time

import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Image, Joy, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32, Float32MultiArray, String

from robot_behavior.mode_constants import (
    DEFAULT_DRIVE_SOURCE,
    DEFAULT_TRACK_MODE,
    TOPIC_DRIVE_SOURCE_CMD,
    TOPIC_DRIVE_SOURCE_STATE,
    TOPIC_TRACK_MODE_CMD,
    TOPIC_TRACK_MODE_STATE,
    latched_qos,
)

# --- Node health -------------------------------------------------------
# "Health" combines two independent signals per node:
#   1. presence — is this node name in the live ROS graph right now
#      (rclpy's own get_node_names(), no subprocess needed)?
#   2. activity — for nodes that publish something on a predictable
#      cadence, has that topic produced a message recently?
# A node can be present but stalled (e.g. hung in a callback) — presence
# alone would falsely report it healthy, which is why the second check
# exists wherever we have a topic to watch. Nodes with no natural periodic
# output (e.g. mode_manager, which only publishes latched state on change)
# are judged on presence alone — there is nothing else honest to check.
#
# This is a real system, not the full 14 nodes named in the design brief's
# example status text — the number in "N/14 nodes ok" is illustrative, not
# a hard target; the grid reports however many nodes are actually expected.
STALE_SEC = 2.0

EXPECTED_NODES = [
    # (display name, ros graph node name, liveness topic, msg type, or None)
    ('mode_manager', 'mode_manager_node', None, None),
    ('target_selector', 'target_selector_node', '/target_detected', Bool),
    ('drive_mux', 'drive_mux_node', '/cmd_vel', Twist),
    ('safety_watchdog', 'safety_watchdog_node', None, None),
    ('ps_controller_bridge', 'ps_controller_bridge_node', '/manual_cmd_vel', Twist),
    ('odin_yolo_tracker', 'odin_yolo_tracker_node', '/odin_target_detected', Bool),
    ('rear_yolo_tracker', 'rear_yolo_tracker_node', '/rear_target_detected', Bool),
    ('intent_to_goal', 'intent_to_goal_node', None, None),
    ('controller_server', 'controller_server', None, None),
    ('velocity_smoother', 'velocity_smoother', '/nav2/cmd_vel_smoothed', Twist),
    ('lifecycle_manager_navigation', 'lifecycle_manager_navigation', None, None),
    ('serial_driver', 'serial_driver', None, None),
    ('odin_driver (host_sdk_sample)', 'host_sdk_sample', '/odin1/image', Image),
    ('web_ui_bridge', 'web_ui_bridge_node', None, None),
]

# target_pid is Phase 6/7's documented fallback (see nav2_bringup.launch.py
# module comment) — normally NOT running under the current Nav2 stack. Kept
# out of EXPECTED_NODES so its absence doesn't read as a failure; if you
# revert to the Phase 6 direct-velocity path, it simply won't appear here.

# Live camera + door frame topics reused as-is (see odin_yolo_tracker_node.py
# / rear_yolo_tracker_node.py subscriptions) — same source the trackers use,
# so the video the operator sees is exactly what YOLO is seeing.
FRONT_IMAGE_TOPIC = '/odin1/image'
REAR_IMAGE_TOPIC = '/image_raw'

# costmap_to_image_node (existing Phase 7 diagnostic node, see that file) is
# a drop-in reuse for the costmap panel — same pattern as the two cameras
# above, just launched (it wasn't part of any bringup file before) and
# streamed the same way. NOT reimplemented here.
COSTMAP_IMAGE_TOPIC = '/local_costmap/costmap_image'

# Point cloud panel reuses the EXACT source already chosen for the local
# costmap's obstacle layer (see nav2_params.yaml: observation_sources
# odin_cloud -> /odin1/cloud_slam) rather than the raw sensor cloud —
# gravity-aligned and floor-filtered, so the live view matches what the
# costmap (and therefore the robot's obstacle avoidance) is actually seeing,
# not a noisier raw feed that would visually disagree with it.
POINTCLOUD_TOPIC = '/odin1/cloud_slam'
POINTCLOUD_MAX_POINTS = 300  # cap for WebSocket payload size / canvas draw cost
POINTCLOUD_GLOBAL_FRAME = 'odom'
POINTCLOUD_BASE_FRAME = 'odin1_base_link'  # matches costmap_to_image_node's default

# Fallback if target_pid_node isn't running to answer a live parameter query
# (see _refresh_bearing_deadband). Keep in sync with target_pid_node.py's
# BEARING_DEADBAND constant if that value is ever retuned.
FALLBACK_BEARING_DEADBAND = 0.08

# Fallback if serial_driver isn't running yet to answer a live query. Keep in
# sync with serial_driver.py's own declared default.
FALLBACK_SPEED_SCALE = 0.15

# Per-wheel, per-direction minimum-effective-speed floors (settings menu,
# 2026-07-25) -- fallbacks match serial_driver.py's own declared defaults.
MOTOR_MIN_SPEED_PARAMS = {
    'left_min_forward_speed': 14.0,
    'left_min_reverse_speed': 14.0,
    'right_min_forward_speed': 10.0,
    'right_min_reverse_speed': 10.0,
}

# Per-camera turn-direction invert toggle (settings menu, 2026-07-25) --
# fallbacks mirror the trackers' own BEARING_SIGN defaults (both un-negated
# -- the actual follow-mode flip turned out to be a wheel-mixing sign bug in
# serial_driver.py, not a camera mirroring issue; see that file's comment).
# Kept as a live per-camera toggle regardless since it's a legitimate,
# independent thing to ever need.
FALLBACK_FRONT_BEARING_SIGN = 1.0
FALLBACK_REAR_BEARING_SIGN = 1.0

EVENT_LOG_MAXLEN = 40


class SharedState:
    """Plain thread-safe snapshot store. One lock, small dicts — this is
    read/written many times a second from two different threads (rclpy
    executor thread + uvicorn's asyncio loop via run_in_executor), so it
    stays intentionally simple rather than reaching for asyncio primitives
    that only one of those two threads could use anyway.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self.drive_source = DEFAULT_DRIVE_SOURCE
        self.track_mode = DEFAULT_TRACK_MODE
        self.bearing_deadband = FALLBACK_BEARING_DEADBAND
        self.speed_scale = FALLBACK_SPEED_SCALE
        self.motor_min_speeds = dict(MOTOR_MIN_SPEED_PARAMS)
        self.auto_enable_override = False  # "auto drive, no trigger held" web UI toggle
        self.front_bearing_sign = FALLBACK_FRONT_BEARING_SIGN
        self.rear_bearing_sign = FALLBACK_REAR_BEARING_SIGN

        self.front = {'detected': False, 'bearing': 0.0, 'distance': 0.0, 'bbox': []}
        self.rear = {'detected': False, 'bearing': 0.0, 'distance': 0.0, 'bbox': []}

        self.door_state = 'unknown'  # 'open' | 'closed' | 'opening' | 'closing' | 'unknown'
        self.door_angle_deg = 0.0    # ESTIMATED, see _door_cmd_cb docstring
        self._door_swing_started = None
        self._door_target_open = None

        self.cpu_percent = None
        self.gpu_percent = None
        self.ram_percent = None
        self.temp_c = None

        self.node_health = []  # list of {name, ok, level} refreshed periodically
        self.event_log = []    # list of {t, text}

        self.latest_front_jpeg = None
        self.latest_rear_jpeg = None
        self.latest_costmap_jpeg = None

        # Robot-relative top-down points [[x, y], ...] in meters, +x forward
        # +y left (REP-103, same convention used throughout this codebase —
        # see target_pid_node.py's angular.z sign comment). Empty until both
        # a cloud message and a valid TF lookup have landed.
        self.pointcloud_xy = []

    def snapshot_fast(self):
        with self._lock:
            return {
                'front': dict(self.front),
                'rear': dict(self.rear),
                'pointcloud': list(self.pointcloud_xy),
            }

    def snapshot_slow(self):
        with self._lock:
            return {
                'drive_source': self.drive_source,
                'track_mode': self.track_mode,
                'bearing_deadband': self.bearing_deadband,
                'speed_scale': self.speed_scale,
                'motor_min_speeds': dict(self.motor_min_speeds),
                'auto_enable_override': self.auto_enable_override,
                'front_bearing_sign': self.front_bearing_sign,
                'rear_bearing_sign': self.rear_bearing_sign,
                'door_state': self.door_state,
                'door_angle_deg': round(self.door_angle_deg, 1),
                'cpu_percent': self.cpu_percent,
                'gpu_percent': self.gpu_percent,
                'ram_percent': self.ram_percent,
                'temp_c': self.temp_c,
                'node_health': list(self.node_health),
                'event_log': list(self.event_log[-12:]),
            }

    def log_event(self, text):
        with self._lock:
            self.event_log.append({'t': time.time(), 'text': text})
            if len(self.event_log) > EVENT_LOG_MAXLEN:
                self.event_log = self.event_log[-EVENT_LOG_MAXLEN:]


# Assumed full swing time for the door-angle ESTIMATE below (no real
# feedback exists — see docstring on _door_cmd_cb). Tune to match the real
# servo's actual travel time if that's ever measured; purely cosmetic
# otherwise, does not affect any control decision.
DOOR_SWING_SEC = 2.0
DOOR_FULL_ANGLE_DEG = 90.0


class WebUiBridgeNode(Node):
    """Subscribes to everything the web UI needs and republishes browser
    gamepad state + button commands back into the existing ROS graph,
    without modifying any node this phase was told to reuse.
    """

    def __init__(self, state: SharedState):
        super().__init__('web_ui_bridge_node')
        self.state = state
        self.bridge = CvBridge()

        # --- Mode state (latched — MUST match mode_manager's QoS, see
        # mode_constants.latched_qos docstring: a volatile subscriber here
        # would silently show a stale/default mode after this node restarts
        # until the next actual mode change, the exact bug already found and
        # fixed once in this project). ---
        self.create_subscription(
            String, TOPIC_DRIVE_SOURCE_STATE, self._drive_source_cb, latched_qos())
        self.create_subscription(
            String, TOPIC_TRACK_MODE_STATE, self._track_mode_cb, latched_qos())

        # --- Per-camera tracker output (own tracker's raw topics, not the
        # target_selector-muxed /target_*, so BOTH camera overlays are live
        # regardless of which one is currently driving — see CONTEXT: "only
        # show it when target_detected is true for that camera's active
        # tracker", read as "the tracker actually watching that camera",
        # not the current follow/flee selection). ---
        self.create_subscription(Bool, '/odin_target_detected', self._front_detected_cb, 10)
        self.create_subscription(Float32, '/odin_target_bearing', self._front_bearing_cb, 10)
        self.create_subscription(Float32, '/odin_target_distance', self._front_distance_cb, 10)
        self.create_subscription(Float32MultiArray, '/odin_target_bbox', self._front_bbox_cb, 10)

        self.create_subscription(Bool, '/rear_target_detected', self._rear_detected_cb, 10)
        self.create_subscription(Float32, '/rear_target_bearing', self._rear_bearing_cb, 10)
        self.create_subscription(Float32, '/rear_target_distance', self._rear_distance_cb, 10)
        self.create_subscription(Float32MultiArray, '/rear_target_bbox', self._rear_bbox_cb, 10)

        # --- Door: no physical angle feedback exists (serial_driver only
        # WRITES to the ESP32 over a timeout-only serial link, see
        # serial_driver.py — there is no read-back channel at all). We
        # track the last commanded open/close and ESTIMATE swing progress
        # against a fixed assumed duration purely for the UI diagram; this
        # is clearly not ground truth and is documented as such in the
        # telemetry payload. ---
        self.create_subscription(String, '/door_cmd', self._door_cmd_cb, 10)

        # --- Camera frames -> latest JPEG for MJPEG streaming ---
        self.create_subscription(Image, FRONT_IMAGE_TOPIC, self._front_image_cb, 1)
        self.create_subscription(Image, REAR_IMAGE_TOPIC, self._rear_image_cb, 1)

        # --- Costmap panel: pure reuse of the existing costmap_to_image_node
        # (see that file) — this bridge only streams its output, same as the
        # two cameras above. ---
        self.create_subscription(Image, COSTMAP_IMAGE_TOPIC, self._costmap_image_cb, 1)

        # --- Point cloud panel: same source already feeding the local
        # costmap's obstacle layer (see POINTCLOUD_TOPIC comment above).
        # Reuses the identical TF-lookup pattern costmap_to_image_node
        # already uses for its own robot marker (odom -> odin1_base_link). ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(PointCloud2, POINTCLOUD_TOPIC, self._cloud_cb, 1)

        # --- Outbound: browser -> ROS ---
        # /joy replaces joy_node as the Twist-scaling input source for
        # ps_controller_bridge (see web_ui_node.py's gamepad translation —
        # ps_controller_bridge itself is UNCHANGED, per the "replace the
        # input source only" instruction).
        self.joy_pub = self.create_publisher(Joy, '/joy', 10)
        self.drive_source_cmd_pub = self.create_publisher(String, TOPIC_DRIVE_SOURCE_CMD, 10)
        self.track_mode_cmd_pub = self.create_publisher(String, TOPIC_TRACK_MODE_CMD, 10)
        self.door_cmd_pub = self.create_publisher(String, '/door_cmd', 10)
        self.manual_cmd_pub = self.create_publisher(Twist, '/manual_cmd_vel', 10)
        # "Auto drive, no trigger held" toggle -- see drive_mux_node.py's
        # NAV_ENABLE_OVERRIDE comment for why this is a dedicated topic
        # OR'd with the real R2 signal rather than also publishing onto
        # /nav_enable directly (would race with ps_controller_bridge's own
        # publisher on that topic).
        self.nav_enable_override_pub = self.create_publisher(Bool, '/nav_enable_override', 10)
        self.create_timer(0.2, self._republish_nav_enable_override)

        # See publish_joy/_republish_joy below for why this exists (fixes a
        # real "randomly drops into auto mode" bug, 2026-07-25).
        self._last_gamepad_axes = [0.0, 0.0]
        self._last_gamepad_buttons = [0] * 11
        self._last_gamepad_recv_time = None
        self._ws_client_count = 0
        self.create_timer(1.0 / self.GAMEPAD_REPUBLISH_HZ, self._republish_joy)

        # Node-health + tegrastats polling on ROS timers (background thread's
        # own executor, does not touch uvicorn's loop).
        self.create_timer(1.0, self._refresh_node_health)
        self.create_timer(5.0, self._refresh_bearing_deadband)
        self.create_timer(5.0, self._refresh_speed_scale)
        self.create_timer(5.0, self._refresh_motor_min_speeds)
        self.create_timer(5.0, self._refresh_bearing_signs)

        self._topic_last_seen = {}
        self._liveness_subs = {}

        from rcl_interfaces.srv import GetParameters, SetParameters
        self._param_client = self.create_client(
            GetParameters, '/target_pid_node/get_parameters')
        # speed_scale (serial_driver's max-power fraction, [0.0-1.0]) is the
        # single knob that caps ALL motor output regardless of drive_source
        # (manual/semi/auto) -- see serial_driver.py's own doc comment on
        # that parameter -- which is why the web UI's max-speed control
        # targets this one parameter rather than the per-mode scale
        # constants (ps_controller_bridge's MAX_LINEAR_X, target_pid's own
        # clamp, etc). Required a small fix to serial_driver.py itself: that
        # parameter was declared but never actually wired to a live update
        # (see its _on_set_parameters — 2026-07-24).
        self._speed_get_client = self.create_client(
            GetParameters, '/serial_driver/get_parameters')
        self._speed_set_client = self.create_client(
            SetParameters, '/serial_driver/set_parameters')

        # Per-camera turn-direction invert toggle (settings menu) -- same
        # live GetParameters/SetParameters pattern as speed_scale above,
        # targeting each YOLO tracker's own 'bearing_sign' parameter.
        self._front_tracker_get_client = self.create_client(
            GetParameters, '/odin_yolo_tracker_node/get_parameters')
        self._front_tracker_set_client = self.create_client(
            SetParameters, '/odin_yolo_tracker_node/set_parameters')
        self._rear_tracker_get_client = self.create_client(
            GetParameters, '/rear_yolo_tracker_node/get_parameters')
        self._rear_tracker_set_client = self.create_client(
            SetParameters, '/rear_yolo_tracker_node/set_parameters')

        # Settings menu "reset costmap" button -- standard Nav2 service
        # (nav2_costmap_2d exposes this on every costmap automatically, not
        # something added for this phase). Clears ALL layers (obstacle +
        # inflation), same as `ros2 service call
        # /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap`.
        from nav2_msgs.srv import ClearEntireCostmap
        self._clear_costmap_client = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')

        self._tegrastats_proc = None
        self._start_tegrastats()

        self.get_logger().info(
            f'web_ui_bridge_node started: front={FRONT_IMAGE_TOPIC}, rear={REAR_IMAGE_TOPIC}, '
            f'publishing synthetic /joy for ps_controller_bridge')

    # -- mode state --
    def _drive_source_cb(self, msg: String):
        with self.state._lock:
            self.state.drive_source = msg.data.strip().lower()

    def _track_mode_cb(self, msg: String):
        with self.state._lock:
            self.state.track_mode = msg.data.strip().lower()

    # -- front tracker --
    def _front_detected_cb(self, msg: Bool):
        with self.state._lock:
            self.state.front['detected'] = bool(msg.data)

    def _front_bearing_cb(self, msg: Float32):
        with self.state._lock:
            self.state.front['bearing'] = float(msg.data)

    def _front_distance_cb(self, msg: Float32):
        with self.state._lock:
            self.state.front['distance'] = float(msg.data)

    def _front_bbox_cb(self, msg: Float32MultiArray):
        with self.state._lock:
            self.state.front['bbox'] = list(msg.data)

    # -- rear tracker --
    def _rear_detected_cb(self, msg: Bool):
        with self.state._lock:
            self.state.rear['detected'] = bool(msg.data)

    def _rear_bearing_cb(self, msg: Float32):
        with self.state._lock:
            self.state.rear['bearing'] = float(msg.data)

    def _rear_distance_cb(self, msg: Float32):
        with self.state._lock:
            self.state.rear['distance'] = float(msg.data)

    def _rear_bbox_cb(self, msg: Float32MultiArray):
        with self.state._lock:
            self.state.rear['bbox'] = list(msg.data)

    # -- door (estimate only, see class docstring above) --
    def _door_cmd_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd not in ('open', 'close'):
            return
        with self.state._lock:
            self.state._door_swing_started = time.monotonic()
            self.state._door_target_open = (cmd == 'open')
            self.state.door_state = 'opening' if cmd == 'open' else 'closing'
        self.state.log_event(f'door_cmd -> {cmd}')

    def _tick_door_estimate(self):
        with self.state._lock:
            started = self.state._door_swing_started
            target_open = self.state._door_target_open
        if started is None:
            return
        elapsed = time.monotonic() - started
        frac = min(1.0, elapsed / DOOR_SWING_SEC)
        angle = frac * DOOR_FULL_ANGLE_DEG if target_open else (1.0 - frac) * DOOR_FULL_ANGLE_DEG
        with self.state._lock:
            self.state.door_angle_deg = angle
            if frac >= 1.0:
                self.state.door_state = 'open' if target_open else 'closed'
            else:
                self.state.door_state = 'opening' if target_open else 'closing'

    # -- camera frames --
    def _front_image_cb(self, msg: Image):
        jpeg = self._encode_jpeg(msg)
        if jpeg is not None:
            with self.state._lock:
                self.state.latest_front_jpeg = jpeg

    def _rear_image_cb(self, msg: Image):
        jpeg = self._encode_jpeg(msg)
        if jpeg is not None:
            with self.state._lock:
                self.state.latest_rear_jpeg = jpeg

    def _costmap_image_cb(self, msg: Image):
        jpeg = self._encode_jpeg(msg)
        if jpeg is not None:
            with self.state._lock:
                self.state.latest_costmap_jpeg = jpeg

    def _cloud_cb(self, msg: PointCloud2):
        try:
            tf = self.tf_buffer.lookup_transform(
                POINTCLOUD_GLOBAL_FRAME, POINTCLOUD_BASE_FRAME,
                rclpy.time.Time(), timeout=Duration(seconds=0.05))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return  # no TF yet (e.g. ODIN not localized) -- leave last-known points in place

        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)

        pts = point_cloud2.read_points(
            msg, field_names=('x', 'y'), skip_nans=True)
        pts = np.array([[p[0], p[1]] for p in pts], dtype=np.float64)
        if pts.shape[0] == 0:
            with self.state._lock:
                self.state.pointcloud_xy = []
            return

        if pts.shape[0] > POINTCLOUD_MAX_POINTS:
            stride = pts.shape[0] // POINTCLOUD_MAX_POINTS
            pts = pts[::stride]

        # odom -> robot-relative: translate then rotate by -yaw (standard 2D
        # frame change, same math costmap_to_image_node uses for its heading
        # arrow, just applied to a point set instead of a single angle).
        dx = pts[:, 0] - rx
        dy = pts[:, 1] - ry
        bx = dx * cos_y + dy * sin_y
        by = -dx * sin_y + dy * cos_y

        with self.state._lock:
            self.state.pointcloud_xy = np.stack([bx, by], axis=1).round(2).tolist()

    def _encode_jpeg(self, msg: Image):
        import cv2
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            return buf.tobytes() if ok else None
        except Exception as e:
            self.get_logger().warn(f'JPEG encode failed: {e}', throttle_duration_sec=5.0)
            return None

    # -- browser -> ROS commands --
    #
    # BUG FIX (2026-07-25): the robot was randomly self-switching from
    # manual to auto mid-drive with no button pressed. Root cause:
    # safety_watchdog_node's controller-disconnect fallback (see that file)
    # forces drive_source to 'auto' (if a target is detected) the instant
    # /joy goes quiet for > joy_stale_timeout_sec (0.5s) — a check designed
    # around joy_node's real hardware Joy messages arriving at a rock-solid
    # cadence, where a gap really did mean the physical controller
    # disconnected. It was never re-examined against THIS phase's actual
    # source: a browser's requestAnimationFrame loop relayed over a
    # WebSocket, which routinely stalls for a few hundred ms under
    # perfectly normal conditions (tab backgrounding/throttling, a Wi-Fi
    # blip, GC pause) while the operator is still very much in control. Each
    # such stall was silently satisfying safety_watchdog's "controller is
    # gone" condition and triggering its designed (but here, wrong) response.
    #
    # Fix: decouple "how often the browser's gamepad frame actually arrives"
    # from "/joy publish cadence". publish_joy() below now only CACHES the
    # latest browser frame; a steady timer (_republish_joy) does the actual
    # publishing at a fixed rate for as long as at least one browser tab is
    # connected (on_ws_connect/on_ws_disconnect track that), using the
    # cached frame if it's reasonably fresh or a neutral (zeroed) Joy
    # message if the browser has gone quiet for a bit longer than one frame
    # should. /joy only stops publishing altogether — the actual condition
    # safety_watchdog's fallback exists to catch — when the WebSocket itself
    # closes, i.e. a REAL disconnect (tab closed, browser crashed, network
    # truly down), not routine scheduling jitter. Same "genuine disconnect
    # must stop the topic, not just drift stale" principle already used
    # elsewhere in this project (see drive_mux_node.py's
    # INPUT_STALE_TIMEOUT_SEC comment on joy_node's autorepeat_rate bug).
    GAMEPAD_DATA_STALE_SEC = 0.3
    GAMEPAD_REPUBLISH_HZ = 20.0

    def publish_joy(self, axes, buttons):
        self._last_gamepad_axes = [float(a) for a in axes]
        self._last_gamepad_buttons = [int(b) for b in buttons]
        self._last_gamepad_recv_time = time.monotonic()

    def on_ws_connect(self):
        self._ws_client_count += 1

    def on_ws_disconnect(self):
        self._ws_client_count = max(0, self._ws_client_count - 1)

    def _republish_joy(self):
        if self._ws_client_count <= 0:
            return  # no browser connected -- let /joy go stale, same as a real hardware disconnect

        fresh = (
            self._last_gamepad_recv_time is not None
            and (time.monotonic() - self._last_gamepad_recv_time) < self.GAMEPAD_DATA_STALE_SEC
        )
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        if fresh:
            msg.axes = list(self._last_gamepad_axes)
            msg.buttons = list(self._last_gamepad_buttons)
        else:
            # Browser is connected but hasn't sent a fresh frame in a
            # while -- neutral, not "keep doing whatever it last said",
            # same reasoning as drive_mux's own input staleness handling.
            msg.axes = [0.0, 0.0]
            # Length must match web_ui_node.py's OUT_BUTTONS_LEN (highest
            # OUT_BUTTON_* index + 1) -- ps_controller_bridge indexes into
            # this array directly and a too-short one would just read as
            # "not pressed" for the missing tail, but keep it in sync.
            msg.buttons = [0] * 11
        self.joy_pub.publish(msg)

    def command_drive_source(self, value):
        self.drive_source_cmd_pub.publish(String(data=value))
        self.state.log_event(f'UI: drive_source -> {value}')

    def command_track_mode(self, value):
        self.track_mode_cmd_pub.publish(String(data=value))
        self.state.log_event(f'UI: track_mode -> {value}')

    def command_door(self, value):
        self.door_cmd_pub.publish(String(data=value))
        self.state.log_event(f'UI: door -> {value}')

    def command_estop(self):
        # Mirror ps_controller_bridge's own estop behavior exactly (force
        # manual + zero Twist) — this is the UI button path, independent of
        # any gamepad button, so it must work even if the Deck is unplugged.
        self.drive_source_cmd_pub.publish(String(data='manual'))
        self.manual_cmd_pub.publish(Twist())
        self.state.log_event('UI: E-STOP pressed')

    def command_reset_costmap(self):
        if not self._clear_costmap_client.service_is_ready():
            self.get_logger().warn('Reset costmap requested but controller_server is not up')
            self.state.log_event('UI: reset costmap -> FAILED (controller_server not up)')
            return
        from nav2_msgs.srv import ClearEntireCostmap
        future = self._clear_costmap_client.call_async(ClearEntireCostmap.Request())
        future.add_done_callback(self._on_clear_costmap_response)
        self.state.log_event('UI: reset costmap requested')

    def _on_clear_costmap_response(self, future):
        try:
            future.result()
        except Exception as e:
            self.get_logger().warn(f'Reset costmap call failed: {e}')
            self.state.log_event('UI: reset costmap -> FAILED')
            return
        self.state.log_event('UI: reset costmap -> done')

    def set_auto_enable_override(self, value):
        value = bool(value)
        with self.state._lock:
            self.state.auto_enable_override = value
        self.state.log_event(f'UI: auto-drive (no trigger) -> {"ON" if value else "off"}')

    def _republish_nav_enable_override(self):
        with self.state._lock:
            value = self.state.auto_enable_override
        self.nav_enable_override_pub.publish(Bool(data=value))

    # -- node health --
    def _refresh_node_health(self):
        try:
            live_names = set(self.get_node_names())
        except Exception:
            live_names = set()

        now = time.monotonic()
        results = []
        for display, node_name, topic, msg_type in EXPECTED_NODES:
            present = node_name in live_names
            if not present:
                results.append({'name': display, 'level': 'down'})
                continue
            if topic is None:
                results.append({'name': display, 'level': 'ok'})
                continue
            self._ensure_liveness_sub(topic, msg_type)
            last_t = self._topic_last_seen.get(topic)
            if last_t is None:
                results.append({'name': display, 'level': 'warn'})
            elif now - last_t > STALE_SEC:
                results.append({'name': display, 'level': 'warn'})
            else:
                results.append({'name': display, 'level': 'ok'})

        with self.state._lock:
            self.state.node_health = results
        self._tick_door_estimate()

    def _ensure_liveness_sub(self, topic, msg_type):
        if topic in self._liveness_subs:
            return

        def _cb(_msg, t=topic):
            self._topic_last_seen[t] = time.monotonic()

        self._liveness_subs[topic] = self.create_subscription(msg_type, topic, _cb, 10)

    # -- bearing deadband: query target_pid_node's OWN live parameter so the
    # frontend's deadzone overlay always matches the real control value in
    # use, rather than duplicating a second hardcoded copy that could drift
    # out of sync if target_pid is ever retuned. Falls back to the
    # documented constant above if target_pid_node isn't running (e.g. the
    # current Nav2-based stack, where it's a fallback-only node). --
    def _refresh_bearing_deadband(self):
        # Non-blocking: this timer callback runs INSIDE the same executor
        # that would have to service the response, so a blocking
        # spin_until_future_complete call here would deadlock against
        # itself. add_done_callback lets the executor pick the response up
        # on a later spin iteration instead.
        if 'target_pid_node' not in self.get_node_names():
            return
        if not self._param_client.service_is_ready():
            return
        from rcl_interfaces.srv import GetParameters
        req = GetParameters.Request()
        req.names = ['bearing_deadband']
        future = self._param_client.call_async(req)
        future.add_done_callback(self._on_bearing_deadband_response)

    def _on_bearing_deadband_response(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().warn(f'bearing_deadband param query failed: {e}', throttle_duration_sec=30.0)
            return
        if result is not None and result.values:
            with self.state._lock:
                self.state.bearing_deadband = result.values[0].double_value

    # -- max speed: mirrors the bearing_deadband pattern above (live query,
    # non-blocking, graceful no-op if serial_driver isn't up yet) so the
    # slider always reflects the REAL current cap -- including if it was
    # last changed via `ros2 param set` from the CLI, or reset to
    # serial_driver's own default by a node restart, not just what the web
    # UI itself last sent. --
    def _refresh_speed_scale(self):
        if not self._speed_get_client.service_is_ready():
            return
        from rcl_interfaces.srv import GetParameters
        req = GetParameters.Request()
        req.names = ['speed_scale']
        future = self._speed_get_client.call_async(req)
        future.add_done_callback(self._on_speed_scale_response)

    def _on_speed_scale_response(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().warn(f'speed_scale param query failed: {e}', throttle_duration_sec=30.0)
            return
        if result is not None and result.values:
            with self.state._lock:
                self.state.speed_scale = result.values[0].double_value

    def set_speed_scale(self, value):
        value = max(0.0, min(1.0, float(value)))
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
        from rcl_interfaces.srv import SetParameters
        if not self._speed_set_client.service_is_ready():
            self.get_logger().warn('speed_scale set requested but serial_driver is not up')
            return
        req = SetParameters.Request()
        req.parameters = [Parameter(
            name='speed_scale',
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value))]
        self._speed_set_client.call_async(req)
        # Optimistic local update -- the periodic refresh above will correct
        # this within 5s if the call actually failed, same fail-safe
        # philosophy as everything else in this bridge.
        with self.state._lock:
            self.state.speed_scale = value
        self.state.log_event(f'UI: max speed -> {value*100:.0f}%')

    # -- per-wheel min speed floors (settings menu): same live-query pattern
    # as speed_scale above, just batched into one GetParameters call for all
    # 4 names at once rather than 4 separate round trips. Reuses the SAME
    # /serial_driver parameter clients speed_scale already set up -- these
    # aren't speed_scale-specific, they're generic /serial_driver param
    # clients. --
    def _refresh_motor_min_speeds(self):
        if not self._speed_get_client.service_is_ready():
            return
        from rcl_interfaces.srv import GetParameters
        req = GetParameters.Request()
        req.names = list(MOTOR_MIN_SPEED_PARAMS.keys())
        future = self._speed_get_client.call_async(req)
        future.add_done_callback(self._on_motor_min_speeds_response)

    def _on_motor_min_speeds_response(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().warn(f'motor min-speed param query failed: {e}', throttle_duration_sec=30.0)
            return
        if result is None or not result.values:
            return
        names = list(MOTOR_MIN_SPEED_PARAMS.keys())
        with self.state._lock:
            for name, value in zip(names, result.values):
                self.state.motor_min_speeds[name] = value.double_value

    def set_motor_min_speed(self, name, value):
        if name not in MOTOR_MIN_SPEED_PARAMS:
            self.get_logger().warn(f'Rejected unknown motor min-speed param: {name}')
            return
        value = max(0.0, min(100.0, float(value)))  # same [-100,100] scale as serial_driver's bytes
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
        from rcl_interfaces.srv import SetParameters
        if not self._speed_set_client.service_is_ready():
            self.get_logger().warn(f'{name} set requested but serial_driver is not up')
            return
        req = SetParameters.Request()
        req.parameters = [Parameter(
            name=name,
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value))]
        self._speed_set_client.call_async(req)
        with self.state._lock:
            self.state.motor_min_speeds[name] = value
        self.state.log_event(f'UI: {name} -> {value:.1f}')

    # -- per-camera turn-direction invert (settings menu, debug): same live
    # GetParameters/SetParameters pattern as speed_scale, one tracker node
    # each. Exposed as a boolean "inverted" in the UI (True -> -1.0, False
    # -> +1.0) rather than the raw signed value, since "invert" is the
    # literal, camera-agnostic thing an operator is deciding live -- see
    # odin_yolo_tracker_node.py / rear_yolo_tracker_node.py BEARING_SIGN. --
    def _refresh_bearing_signs(self):
        from rcl_interfaces.srv import GetParameters
        if self._front_tracker_get_client.service_is_ready():
            req = GetParameters.Request()
            req.names = ['bearing_sign']
            future = self._front_tracker_get_client.call_async(req)
            future.add_done_callback(self._on_front_bearing_sign_response)
        if self._rear_tracker_get_client.service_is_ready():
            req = GetParameters.Request()
            req.names = ['bearing_sign']
            future = self._rear_tracker_get_client.call_async(req)
            future.add_done_callback(self._on_rear_bearing_sign_response)

    def _on_front_bearing_sign_response(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().warn(f'front bearing_sign param query failed: {e}', throttle_duration_sec=30.0)
            return
        if result is not None and result.values:
            with self.state._lock:
                self.state.front_bearing_sign = result.values[0].double_value

    def _on_rear_bearing_sign_response(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().warn(f'rear bearing_sign param query failed: {e}', throttle_duration_sec=30.0)
            return
        if result is not None and result.values:
            with self.state._lock:
                self.state.rear_bearing_sign = result.values[0].double_value

    def set_bearing_sign(self, camera, inverted):
        if camera not in ('front', 'rear'):
            self.get_logger().warn(f'Rejected unknown camera for bearing_sign: {camera}')
            return
        value = -1.0 if inverted else 1.0
        client = self._front_tracker_set_client if camera == 'front' else self._rear_tracker_set_client
        if not client.service_is_ready():
            self.get_logger().warn(f'bearing_sign set requested for {camera} but tracker node is not up')
            return
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
        from rcl_interfaces.srv import SetParameters
        req = SetParameters.Request()
        req.parameters = [Parameter(
            name='bearing_sign',
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value))]
        client.call_async(req)
        with self.state._lock:
            if camera == 'front':
                self.state.front_bearing_sign = value
            else:
                self.state.rear_bearing_sign = value
        self.state.log_event(f'UI: {camera} camera turn direction -> {"inverted" if inverted else "normal"}')

    def _start_tegrastats(self):
        try:
            self._tegrastats_proc = subprocess.Popen(
                ['tegrastats', '--interval', '1000'],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        except FileNotFoundError:
            self.get_logger().warn('tegrastats not found — CPU/GPU telemetry disabled')
            return
        threading.Thread(target=self._read_tegrastats_loop, daemon=True).start()

    def _read_tegrastats_loop(self):
        # Sample tegrastats line, e.g.:
        # RAM 4210/7860MB ... CPU [12%@1907,8%@1907,...] ... GR3D_FREQ 34%
        import re
        proc = self._tegrastats_proc
        for line in proc.stdout:
            try:
                ram_m = re.search(r'RAM (\d+)/(\d+)MB', line)
                cpu_m = re.findall(r'(\d+)%@\d+', line)
                gpu_m = re.search(r'GR3D_FREQ (\d+)%', line)
                temp_m = re.search(r'CPU@(\d+(?:\.\d+)?)C', line)
                with self.state._lock:
                    if ram_m:
                        used, total = int(ram_m.group(1)), int(ram_m.group(2))
                        self.state.ram_percent = round(100.0 * used / total, 1)
                    if cpu_m:
                        vals = [int(v) for v in cpu_m]
                        self.state.cpu_percent = round(sum(vals) / len(vals), 1)
                    if gpu_m:
                        self.state.gpu_percent = int(gpu_m.group(1))
                    if temp_m:
                        self.state.temp_c = float(temp_m.group(1))
            except Exception:
                continue

    def destroy_node(self):
        if self._tegrastats_proc is not None:
            self._tegrastats_proc.terminate()
        super().destroy_node()
