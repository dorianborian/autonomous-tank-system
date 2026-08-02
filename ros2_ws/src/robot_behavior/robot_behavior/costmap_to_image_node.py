"""Phase 7 diagnostic: renders the Nav2 local costmap (OccupancyGrid) as a
color image on /local_costmap/costmap_image, so it can be viewed in a
headless browser via web_video_server (no rviz/display needed).

Color key:
  black       = free (cost 0)
  red->yellow = inflated cost (1-98), brighter = closer to lethal
  bright red  = lethal (cost >= 99)
  dark gray   = unknown (-1)
  green dot   = robot's current cell (grid center; costmap is a rolling
                window centered on the robot, so this is always mid-image)
  blue arrow  = robot heading (drawn from TF, forward direction)
  white ring  = collision_monitor Stop zone (0.30m radius); turns SOLID RED
                when collision_monitor is actively commanding a stop
  gray ring   = collision_monitor Slowdown zone (0.60m radius); turns SOLID
                YELLOW when collision_monitor is actively slowing the robot
  magenta X   = ACTIVE goal (/intent_goal_debug) — marker-based, or a LOCKED
                joystick goal that's actually being executed/routed to
  yellow O    = CANDIDATE joystick goal (/candidate_goal_debug) — where the
                stick is currently aiming, NOT locked in yet, no motion
                planning happening for it. Press X to lock (turns into the
                magenta X above), O to clear.
  cyan line   = /received_global_plan — the A*-planned route intent_to_goal
                requested (goal -> obstacle-aware path, from costmap_planner)
  lime line   = /local_plan — DWB's actual tracked trajectory following that
                route. Both should curve around a lethal blob together; if
                cyan curves but lime doesn't follow, DWB is struggling to
                track the plan (a separate, real problem worth flagging).

This node only reads /local_costmap/costmap, /collision_monitor_state,
/intent_goal_debug, /candidate_goal_debug, /received_global_plan,
/local_plan, and TF — it does not publish anything that can affect driving.
"""

import math
import time

import cv2
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                        QoSReliabilityPolicy)
from sensor_msgs.msg import Image

UPSCALE = 4  # 80x80 costmap cells -> 320x320 image, easier to see in-browser

# Must match the Stop/Slowdown circle radii in nav2_params.yaml's
# collision_monitor block. Kept as a separate constant here (not read from
# the params file) since this is a debug overlay, not a control path —
# TUNABLE, keep in sync by hand if the real zone radii change.
STOP_RADIUS_M = 0.30
SLOWDOWN_RADIUS_M = 0.60
CM_STATE_STALE_SEC = 1.0
PLAN_STALE_SEC = 1.0   # goal/plan considered "not currently active" past this age

# Path/goal visibility (2026-07-25, explicit request): these were previously
# drawn with _draw_line, a hand-rolled single-pixel-per-step line (see that
# method) -- always exactly 1px wide regardless of image size, which reads
# as barely-there on a 320x320 image shown in a compact web UI panel.
# Switched to cv2.line/cv2.circle with real thickness + anti-aliasing below;
# these also clip out-of-bounds coordinates automatically, so the manual
# bounds math _draw_line needed is no longer necessary for these markers.
PLAN_LINE_THICKNESS = 3
GOAL_MARKER_SIZE = 12       # X half-size, px (was 8)
GOAL_MARKER_THICKNESS = 3
CANDIDATE_CIRCLE_RADIUS = 10  # px (was 7)
CANDIDATE_CIRCLE_THICKNESS = 3  # (was 2)


class CostmapToImageNode(Node):

    def __init__(self):
        super().__init__('costmap_to_image_node')

        self.declare_parameter('global_frame', 'odom')
        self.declare_parameter('robot_base_frame', 'odin1_base_link')
        self.global_frame = self.get_parameter('global_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        costmap_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self._costmap_cb, costmap_qos)
        self.create_subscription(
            CollisionMonitorState, '/collision_monitor_state', self._cm_state_cb, 10)

        goal_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PoseStamped, '/intent_goal_debug', self._goal_cb, goal_qos)
        self.create_subscription(
            PoseStamped, '/candidate_goal_debug', self._candidate_cb, goal_qos)
        self.create_subscription(
            Path, '/received_global_plan', self._global_plan_cb, 10)
        self.create_subscription(
            Path, '/local_plan', self._local_plan_cb, 10)

        self.cm_action = CollisionMonitorState.DO_NOTHING
        self._cm_state_t = None
        self.goal_pose = None
        self._goal_t = None
        self.candidate_pose = None
        self._candidate_t = None
        self.global_plan = None
        self._global_plan_t = None
        self.local_plan = None
        self._local_plan_t = None

        self.image_pub = self.create_publisher(Image, '/local_costmap/costmap_image', 1)

        self.get_logger().info(
            'costmap_to_image_node started: publishing /local_costmap/costmap_image '
            f'(x{UPSCALE} upscaled). View via web_video_server, e.g. '
            'http://<jetson-ip>:8080/stream?topic=/local_costmap/costmap_image')

    def _costmap_cb(self, msg: OccupancyGrid):
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int16).reshape((h, w))

        img = np.zeros((h, w, 3), dtype=np.uint8)
        unknown = data < 0
        free = data == 0
        lethal = data >= 99
        inflated = (data > 0) & (data < 99)

        img[unknown] = (60, 60, 60)          # dark gray
        img[free] = (0, 0, 0)                # black
        # Inflated: dim-red -> orange/yellow heat ramp BY ACTUAL COST, not a flat
        # "any nonzero = full red" fill. A prior version pegged R=255 for every
        # inflated cell regardless of cost, which made even a barely-inflated
        # cost-1 cell look as alarming as cost-98 — misleading when eyeballing
        # whether a gap is really tight or just barely cost-touched. Now R also
        # ramps with cost (dim at low cost, bright at high cost).
        infl_cost = data[inflated].astype(np.float32) / 98.0
        img[inflated, 2] = (80 + infl_cost * 175).astype(np.uint8)   # R: 80->255
        img[inflated, 1] = (infl_cost * 200).astype(np.uint8)        # G: 0->200
        img[lethal] = (0, 0, 255)            # bright red (BGR: B=0 G=0 R=255)

        # OccupancyGrid row 0 = origin (bottom-left); flip vertically so
        # "up" in the image is +y, matching a conventional top-down map view.
        img = np.flipud(img)

        img = np.kron(img, np.ones((UPSCALE, UPSCALE, 1), dtype=np.uint8))

        self._draw_robot_marker(img, msg)
        self._draw_plan(img, msg, self.global_plan, self._global_plan_t, (255, 255, 0))  # cyan
        self._draw_plan(img, msg, self.local_plan, self._local_plan_t, (0, 255, 0))       # lime
        self._draw_candidate_goal(img, msg)
        self._draw_goal(img, msg)

        out = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        out.header = msg.header
        self.image_pub.publish(out)

    def _cm_state_cb(self, msg: CollisionMonitorState):
        self.cm_action = msg.action_type
        self._cm_state_t = time.monotonic()

    def _goal_cb(self, msg: PoseStamped):
        self.goal_pose = msg
        self._goal_t = time.monotonic()

    def _candidate_cb(self, msg: PoseStamped):
        self.candidate_pose = msg
        self._candidate_t = time.monotonic()

    def _global_plan_cb(self, msg: Path):
        self.global_plan = msg
        self._global_plan_t = time.monotonic()

    def _local_plan_cb(self, msg: Path):
        self.local_plan = msg
        self._local_plan_t = time.monotonic()

    def _is_fresh(self, t):
        return t is not None and (time.monotonic() - t) < PLAN_STALE_SEC

    def _odom_xy_to_pixel(self, x, y, costmap_msg):
        """Convert an (x, y) point in odom to a pixel coord in the rendered
        image. Shared by the robot marker, goal marker, and plan lines so
        they all use one consistent transform."""
        res = costmap_msg.info.resolution
        ox = costmap_msg.info.origin.position.x
        oy = costmap_msg.info.origin.position.y
        gx = int((x - ox) / res) * UPSCALE
        gy_from_bottom = int((y - oy) / res)
        gy = (costmap_msg.info.height - 1 - gy_from_bottom) * UPSCALE
        return gx, gy

    def _draw_goal(self, img, costmap_msg):
        if not self._is_fresh(self._goal_t):
            return
        h_img, w_img = img.shape[0], img.shape[1]
        gx, gy = self._odom_xy_to_pixel(
            self.goal_pose.pose.position.x, self.goal_pose.pose.position.y, costmap_msg)
        if not (0 <= gx < w_img and 0 <= gy < h_img):
            return  # goal projected outside the local window — off-screen, not an error
        s = GOAL_MARKER_SIZE
        color = (255, 0, 255)  # magenta (BGR)
        cv2.line(img, (gx - s, gy - s), (gx + s, gy + s), color,
                 GOAL_MARKER_THICKNESS, cv2.LINE_AA)
        cv2.line(img, (gx - s, gy + s), (gx + s, gy - s), color,
                 GOAL_MARKER_THICKNESS, cv2.LINE_AA)

    def _draw_candidate_goal(self, img, costmap_msg):
        if not self._is_fresh(self._candidate_t):
            return
        h_img, w_img = img.shape[0], img.shape[1]
        gx, gy = self._odom_xy_to_pixel(
            self.candidate_pose.pose.position.x, self.candidate_pose.pose.position.y,
            costmap_msg)
        if not (0 <= gx < w_img and 0 <= gy < h_img):
            return
        cv2.circle(img, (gx, gy), CANDIDATE_CIRCLE_RADIUS, (0, 255, 255),
                   CANDIDATE_CIRCLE_THICKNESS, cv2.LINE_AA)  # hollow yellow circle (BGR)

    def _draw_plan(self, img, costmap_msg, plan, plan_t, color):
        if plan is None or not self._is_fresh(plan_t) or len(plan.poses) < 2:
            return
        pts = [self._odom_xy_to_pixel(p.pose.position.x, p.pose.position.y, costmap_msg)
               for p in plan.poses]
        # cv2.line clips out-of-bounds coordinates on its own -- no need for
        # the manual in-bounds pre-check the old _draw_line-based version had.
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            cv2.line(img, (x0, y0), (x1, y1), color, PLAN_LINE_THICKNESS, cv2.LINE_AA)

    def _cm_action_is_active(self, action):
        if self._cm_state_t is None:
            return False
        if (time.monotonic() - self._cm_state_t) > CM_STATE_STALE_SEC:
            return False
        return self.cm_action == action

    def _draw_robot_marker(self, img, msg: OccupancyGrid):
        h_img, w_img = img.shape[0], img.shape[1]
        res = msg.info.resolution

        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_base_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        gx, gy = self._odom_xy_to_pixel(rx, ry, msg)
        cx = min(max(gx, 0), w_img - 1)
        cy = min(max(gy, 0), h_img - 1)

        # collision_monitor zones, drawn as rings (or filled when actively
        # triggering) so it's visually obvious when a stop/slowdown fires.
        px_per_m = UPSCALE / res
        stop_r_px = int(STOP_RADIUS_M * px_per_m)
        slow_r_px = int(SLOWDOWN_RADIUS_M * px_per_m)

        slow_active = self._cm_action_is_active(CollisionMonitorState.SLOWDOWN)
        stop_active = self._cm_action_is_active(CollisionMonitorState.STOP)

        cv2.circle(img, (cx, cy), slow_r_px, (0, 255, 255) if slow_active else (90, 90, 90),
                   -1 if slow_active else 1)
        cv2.circle(img, (cx, cy), stop_r_px, (0, 0, 255) if stop_active else (220, 220, 220),
                   -1 if stop_active else 1)

        r = 6
        img[max(cy - r, 0):min(cy + r, h_img), max(cx - r, 0):min(cx + r, w_img)] = (0, 255, 0)

        # Heading arrow (image y grows downward; odom +y maps to image "up",
        # i.e. decreasing row index — see flipud above).
        ax = int(cx + 20 * math.cos(yaw))
        ay = int(cy - 20 * math.sin(yaw))
        self._draw_line(img, cx, cy, ax, ay, (255, 128, 0))

    @staticmethod
    def _draw_line(img, x0, y0, x1, y1, color):
        h_img, w_img = img.shape[0], img.shape[1]
        steps = max(abs(x1 - x0), abs(y1 - y0), 1)
        for i in range(steps + 1):
            t = i / steps
            x = int(x0 + t * (x1 - x0))
            y = int(y0 + t * (y1 - y0))
            if 0 <= x < w_img and 0 <= y < h_img:
                img[y, x] = color


def main(args=None):
    rclpy.init(args=args)
    node = CostmapToImageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
