import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32, Float32MultiArray
from ultralytics import YOLO

from robot_behavior_msgs.msg import TargetDetection

# Same calibrated fisheye-camera pinhole approximation odin_target_tracker_node
# used for ArUco (see that file's module comment): real focal length from
# calib.yaml, least accurate for boxes near the frame edge since the lens is
# actually FishPoly, not pinhole. Accepted here for the same reason: person
# detections steering the robot are usually near-center anyway.
CALIB_A11 = 7.3492194936202099e+02
CALIB_A12 = -8.4727063323393436e-01
CALIB_A22 = 7.3491032666831233e+02
CALIB_U0 = 7.5436190439525842e+02
CALIB_V0 = 6.5201821461453027e+02
CALIB_IMAGE_WIDTH_PX = 1600

# calib.yaml Tcl_0: transforms a point from the dToF/lidar frame into the RGB
# camera frame (row-major 4x4, p_cam = Tcl_0 @ p_lidar_homogeneous).
TCL_0 = np.array([
    [0.00022, -0.99999, -0.00343, 0.03832],
    [0.00547, 0.00343, -0.99998, 0.00488],
    [0.99999, 0.0002, 0.00547, -0.00971],
    [0, 0, 0, 1],
], dtype=np.float64)

DEFAULT_ENGINE_PATH = os.path.expanduser('~/behavior_ws/models/yolo11n.engine')

# The TensorRT engine has a fixed 640x640 input (confirmed via
# predictor.imgsz); Ultralytics letterboxes whatever we feed it down to that
# regardless. The ODIN publishes at 1600x1296, so passing the raw frame in
# means Ultralytics' internal cv2.resize does a full 1600x1296 -> 640x640
# letterbox every frame. Pre-shrinking to the engine's max dimension first
# makes that resize operate on far fewer source pixels (a much cheaper
# resize from ~640x518 -> 640x640 letterbox instead of 1600x1296 -> 640x640).
# The GPU-side inference cost is identical either way (same fixed 640x640
# tensor); this only saves the CPU-side preprocessing.
INFER_MAX_DIM = 640

# Reported 2026-07-25: standing to the LEFT of the robot and activating
# follow sent it RIGHT. Originally suspected to be a mirrored front-camera
# image (this constant was briefly set to -1.0 to compensate) -- RULED OUT
# by live-capturing a real front-camera frame and confirming visible text in
# the scene reads correctly, not reversed. The actual root cause was
# downstream, in serial_driver.py's differential-drive wheel mixing (see
# that file's 2026-07-25 comment) -- confirmed by physically commanding a
# known angular.z and watching /odin1/odometry yaw respond in the wrong
# direction. Left at the un-negated default; kept as a live parameter (wired
# to a web UI settings toggle) since it's a legitimate, independent thing to
# ever need (e.g. a genuinely mirrored replacement camera), just not what
# was wrong here.
BEARING_SIGN = 1.0


class OdinYoloTrackerNode(Node):
    """Drop-in replacement for odin_target_tracker_node: same topic names/types
    (/odin_target_detected, /odin_target_bearing, /odin_target_distance), but
    detects the largest person via YOLO (TensorRT engine) instead of an ArUco
    marker, and looks up real depth from the ODIN's sparse dToF point cloud
    instead of estimating distance from marker size.
    """

    def __init__(self):
        super().__init__('odin_yolo_tracker_node')

        self.declare_parameter('engine_path', DEFAULT_ENGINE_PATH)
        self.declare_parameter('conf_threshold', 0.5)
        # Half-width of the "no correction needed" zone, as a fraction of
        # image width. 0.15 of the ODIN's 1600px-wide frame is +/-240px --
        # generous enough to absorb normal walking sway without chosen so
        # small it re-triggers on every step, small enough to still react
        # once someone has clearly moved off-center.
        self.declare_parameter('deadzone_ratio', 0.15)
        # Consecutive missed frames before we declare the target lost and
        # start the persistence/search countdown, rather than reacting to a
        # single dropped detection.
        self.declare_parameter('loss_frame_threshold', 2)
        self.declare_parameter('persistence_timeout_sec', 5.0)
        # Best-guess distance published while "lost but searching". Deliberately
        # NOT the farthest chase-zone value: target_pid's chase zone ramps
        # linear.x from 0 at chase_min_m to max at chase_max_m, so publishing
        # a mid-chase-zone distance yields a real but reduced forward speed
        # while searching, without touching target_pid at all.
        self.declare_parameter('persistence_distance_m', 0.75)
        # Magnitude of the bearing nudge published while searching (full +/-1.0
        # felt too aggressive live; this still visibly steers toward the exit
        # side without a full-lock turn while there's no one there to confirm).
        self.declare_parameter('persistence_bearing_magnitude', 0.6)
        # Radius (px, in the calibrated 1600-wide frame) to widen the point
        # search if none of the sparse dToF points land inside the bbox itself.
        self.declare_parameter('depth_search_radius_px', 40)
        self.declare_parameter('bearing_sign', BEARING_SIGN)

        self.engine_path = self.get_parameter('engine_path').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.deadzone_ratio = self.get_parameter('deadzone_ratio').value
        self.loss_frame_threshold = self.get_parameter('loss_frame_threshold').value
        self.persistence_timeout_sec = self.get_parameter('persistence_timeout_sec').value
        self.persistence_distance_m = self.get_parameter('persistence_distance_m').value
        self.persistence_bearing_magnitude = self.get_parameter('persistence_bearing_magnitude').value
        self.depth_search_radius_px = self.get_parameter('depth_search_radius_px').value
        self.bearing_sign = self.get_parameter('bearing_sign').value
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.bridge = CvBridge()
        self.get_logger().info(f'Loading YOLO engine: {self.engine_path}')
        self.model = YOLO(self.engine_path, task='detect')

        self.latest_cloud_points_cam = None  # Nx3 array in camera frame, or None

        self.miss_count = 0
        self.searching = False
        self.search_started_at = None
        self.exit_bearing_sign = 0.0  # -1 left, 0 straight, +1 right
        self.last_valid_distance = self.persistence_distance_m

        self.image_sub = self.create_subscription(
            Image, '/odin1/image', self.image_callback, 10)
        self.cloud_sub = self.create_subscription(
            PointCloud2, '/odin1/cloud_raw', self.cloud_callback, 10)

        self.detected_pub = self.create_publisher(Bool, '/odin_target_detected', 10)
        self.bearing_pub = self.create_publisher(Float32, '/odin_target_bearing', 10)
        self.distance_pub = self.create_publisher(Float32, '/odin_target_distance', 10)
        # Web UI bbox overlay: [x1, y1, x2, y2] normalized to [0.0, 1.0] of frame
        # width/height, only meaningful (and only published) for a REAL YOLO
        # detection this frame -- not during the miss-hold or persistence/
        # search windows above, since there is no actual box to draw then.
        self.bbox_pub = self.create_publisher(Float32MultiArray, '/odin_target_bbox', 10)
        self.debug_image_pub = self.create_publisher(Image, '/odin_yolo_debug_image', 10)
        # Refactor (target_state_estimator): a SEPARATE stamped raw-detection
        # topic, additive -- does not touch the legacy Bool/Float32/Float32
        # topics above or their miss-hold/persistence behavior (kept for the
        # bringup.launch.py fallback path, see that file's comments). This
        # topic reports ONLY this frame's real YOLO result (no miss-hold, no
        # search fabrication) with header.stamp = the source image's capture
        # time, so target_state_estimator can do a correct tf2 time-travel
        # TF lookup instead of fusing against a stale/processing-time pose.
        self.detection_pub = self.create_publisher(
            TargetDetection, '/odin_target_detection_raw', 10)

        self.get_logger().info(
            f'odin_yolo_tracker_node started: conf={self.conf_threshold}, '
            f'deadzone_ratio={self.deadzone_ratio}, '
            f'persistence_timeout={self.persistence_timeout_sec}s, '
            'subscribed to /odin1/image + /odin1/cloud_raw')

    def _on_set_parameters(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'bearing_sign':
                self.bearing_sign = p.value
                self.get_logger().info(f'bearing_sign -> {p.value:+.1f}')
        return SetParametersResult(successful=True)

    def cloud_callback(self, msg: PointCloud2):
        # read_points already returns a vectorized structured ndarray -- do
        # NOT iterate it point-by-point in Python (was previously a
        # per-point list comprehension over up to ~49k points/cloud, which
        # blocked the single-threaded executor for tens of ms per message
        # and starved image_callback of scheduling time, keeping the GPU
        # idle regardless of the TensorRT engine being correctly loaded).
        struct_pts = point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if struct_pts.size == 0:
            self.latest_cloud_points_cam = None
            return
        pts = np.stack(
            [struct_pts['x'], struct_pts['y'], struct_pts['z']], axis=-1
        ).astype(np.float64)
        ones = np.ones((pts.shape[0], 1))
        pts_h = np.hstack([pts, ones])
        pts_cam = (TCL_0 @ pts_h.T).T[:, :3]
        # Keep only points in front of the camera.
        pts_cam = pts_cam[pts_cam[:, 2] > 0.05]
        self.latest_cloud_points_cam = pts_cam if pts_cam.shape[0] > 0 else None

    def _lookup_depth(self, x1, y1, x2, y2):
        """Project the cached dToF cloud into the image and return the median
        depth of points landing inside the bbox (widening to a small radius
        around the bbox center if the sparse cloud has none inside it)."""
        pts_cam = self.latest_cloud_points_cam
        if pts_cam is None:
            return None

        z = pts_cam[:, 2]
        u = CALIB_A11 * pts_cam[:, 0] / z + CALIB_A12 * pts_cam[:, 1] / z + CALIB_U0
        v = CALIB_A22 * pts_cam[:, 1] / z + CALIB_V0

        in_box = (u >= x1) & (u <= x2) & (v >= y1) & (v <= y2)
        if np.any(in_box):
            return float(np.median(z[in_box]))

        cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        r = self.depth_search_radius_px
        near = ((u - cx) ** 2 + (v - cy) ** 2) <= r * r
        if np.any(near):
            return float(np.median(z[near]))

        return None

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame_width = frame.shape[1]
        frame_height = frame.shape[0]

        # Downscale (aspect-preserving) only the copy fed to inference --
        # everything below this block (bearing, depth lookup, debug drawing,
        # bbox normalization) keeps operating in the ORIGINAL frame's pixel
        # space, so the calibrated depth projection in _lookup_depth (which
        # assumes CALIB_IMAGE_WIDTH_PX-native coordinates) is unaffected.
        # Box coordinates are rescaled back to native space immediately
        # below, before any other use.
        infer_scale = INFER_MAX_DIM / float(max(frame_width, frame_height))
        infer_frame = cv2.resize(
            frame, (round(frame_width * infer_scale), round(frame_height * infer_scale)))

        results = self.model.predict(
            infer_frame, conf=self.conf_threshold, classes=[0], verbose=False)
        boxes = results[0].boxes

        best_box = None
        best_area = -1.0
        if boxes is not None and len(boxes) > 0:
            xyxy = boxes.xyxy.cpu().numpy() / infer_scale
            for x1, y1, x2, y2 in xyxy:
                area = (x2 - x1) * (y2 - y1)
                if area > best_area:
                    best_area = area
                    best_box = (x1, y1, x2, y2)

        debug_wanted = self.debug_image_pub.get_subscription_count() > 0
        debug_frame = frame
        now = time.monotonic()

        if best_box is not None:
            self.miss_count = 0
            was_searching = self.searching
            self.searching = False
            self.search_started_at = None

            x1, y1, x2, y2 = best_box
            center_x = (x1 + x2) / 2.0

            half_dead = self.deadzone_ratio * frame_width
            offset = center_x - frame_width / 2.0
            if abs(offset) <= half_dead:
                bearing = 0.0
            else:
                bearing = offset / (frame_width / 2.0)
                bearing = max(-1.0, min(1.0, bearing))

            distance = self._lookup_depth(x1, y1, x2, y2)
            if distance is None:
                # Sparse dToF had nothing under this box this frame -- hold
                # the last good reading rather than publishing a bogus zero.
                distance = self.last_valid_distance
            else:
                self.last_valid_distance = distance

            detected = True

            if debug_wanted:
                debug_frame = frame.copy()
                cv2.rectangle(debug_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                label = f'd={distance:.2f}m b={bearing:+.2f}' + (' [reacquired]' if was_searching else '')
                cv2.putText(debug_frame, label, (int(x1), max(15, int(y1) - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            self.miss_count += 1

            if self.miss_count < self.loss_frame_threshold:
                # Not enough consecutive misses yet to declare loss -- hold
                # last state silently (avoid single-frame flicker).
                detected = True
                bearing = 0.0
                distance = self.last_valid_distance
            else:
                if not self.searching:
                    self.searching = True
                    self.search_started_at = now
                    # exit_bearing_sign was set on the last valid detection
                    # below; nothing to do here besides starting the clock.

                elapsed = now - self.search_started_at
                if elapsed <= self.persistence_timeout_sec:
                    detected = True
                    bearing = self.exit_bearing_sign * self.persistence_bearing_magnitude
                    distance = self.persistence_distance_m
                else:
                    detected = False
                    bearing = 0.0
                    distance = 0.0

        # Track exit direction from the last valid detection's position,
        # independent of whether we just started searching this frame --
        # cheap to keep updated every frame a person is actually seen.
        if best_box is not None:
            x1, y1, x2, y2 = best_box
            center_x = (x1 + x2) / 2.0
            if center_x < frame_width / 3.0:
                self.exit_bearing_sign = -1.0
            elif center_x > 2.0 * frame_width / 3.0:
                self.exit_bearing_sign = 1.0
            else:
                self.exit_bearing_sign = 0.0

        # Snapshot the REAL per-frame result (before the miss-hold/persistence
        # branches above may have substituted a held or fabricated value) for
        # the stamped raw-detection topic -- best_box is not None here iff
        # this frame had an actual YOLO detection, in which case bearing/
        # distance above were computed fresh, not held/faked.
        raw_detected = best_box is not None
        raw_bearing = self.bearing_sign * (bearing if raw_detected else 0.0)
        raw_distance = distance if raw_detected else 0.0

        # Applied last, after both the live-detection and persistence/search
        # branches above have settled on a raw image-offset-based bearing --
        # a single point of truth so the sign fix (and the web UI's live
        # invert toggle) covers every source of a published bearing
        # uniformly, including the search-direction nudge.
        bearing = self.bearing_sign * bearing

        self.detected_pub.publish(Bool(data=bool(detected)))
        self.bearing_pub.publish(Float32(data=float(bearing)))
        self.distance_pub.publish(Float32(data=float(distance)))

        raw_msg = TargetDetection()
        raw_msg.header = msg.header  # capture timestamp, not processing time
        raw_msg.detected = bool(raw_detected)
        raw_msg.bearing = float(raw_bearing)
        raw_msg.distance = float(raw_distance)
        self.detection_pub.publish(raw_msg)

        # Only a REAL YOLO box this frame gets a bbox published (empty array
        # otherwise) -- miss-hold and persistence/search have no actual box to
        # draw, so the web UI overlay should not show a stale/fabricated one.
        bbox_msg = Float32MultiArray()
        if best_box is not None:
            x1, y1, x2, y2 = best_box
            bbox_msg.data = [
                float(x1) / frame_width, float(y1) / frame_height,
                float(x2) / frame_width, float(y2) / frame_height,
            ]
        self.bbox_pub.publish(bbox_msg)

        if debug_wanted:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdinYoloTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
