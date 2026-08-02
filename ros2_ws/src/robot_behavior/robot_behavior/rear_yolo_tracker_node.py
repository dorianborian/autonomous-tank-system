import math
import os
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Float32MultiArray
from ultralytics import YOLO

from robot_behavior_msgs.msg import TargetDetection

# Rear webcam has no depth sensor (unlike the ODIN front camera's dToF point
# cloud, see odin_yolo_tracker_node.py). Distance here is a rough estimate
# from bbox height using an assumed average person height and a focal length
# derived from the webcam's assumed FOV -- same assumed-FOV approach the old
# aruco_tracker_node used for its own distance estimate, just driven by person
# height instead of a known marker size. This is materially less accurate than
# the front tracker's real depth lookup: flee mode leans on bearing +
# detection presence being right, and treats distance as a coarse "how close"
# signal, not a precise range.
ASSUMED_HORIZONTAL_FOV_DEG = 120.0
DEFAULT_ENGINE_PATH = os.path.expanduser('~/behavior_ws/models/yolo11n.engine')

# See odin_yolo_tracker_node.py's BEARING_SIGN comment -- same live-flippable
# per-camera sign, wired to the same web UI settings toggle. The front
# camera's flip was confirmed by direct observation (2026-07-25); the rear
# camera's is NOT yet confirmed (intent_to_goal_node's module docstring
# already flags FLEE_LATERAL_SIGN as unconfirmed for the same underlying
# reason -- the rear camera may or may not be mirrored the same way).
# Left at the un-negated default until confirmed live; flip via the
# settings-menu toggle if flee steers the wrong way.
BEARING_SIGN = 1.0


class RearYoloTrackerNode(Node):
    """Drop-in replacement for aruco_tracker_node (rear webcam): same native
    topic names (/target_detected, /target_bearing, /target_distance -- the
    launch file remaps these to /rear_target_* exactly like it did for the
    old aruco node), but detects the largest person via YOLO instead of an
    ArUco marker.
    """

    def __init__(self):
        super().__init__('rear_yolo_tracker_node')

        self.declare_parameter('engine_path', DEFAULT_ENGINE_PATH)
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('assumed_person_height_m', 1.7)
        # See odin_yolo_tracker_node.py for the deadzone_ratio rationale --
        # same fraction-of-width definition so the two trackers behave
        # consistently even though the rear webcam is a much lower resolution.
        self.declare_parameter('deadzone_ratio', 0.15)
        self.declare_parameter('loss_frame_threshold', 2)
        self.declare_parameter('persistence_timeout_sec', 5.0)
        # Chosen deliberately, NOT the 1.0m literal example: target_pid's flee
        # inversion suppresses linear.x to zero for any distance at/above
        # chase_min_m (0.6m) -- see target_pid_node.py's flee comment. A
        # constant 1.0m during persistence would silently zero the escape
        # speed while "searching", defeating the point of this test. Below
        # reverse_threshold_m (0.4m) keeps the existing flee inversion
        # producing a real (if reduced-from-max) escape speed while searching.
        self.declare_parameter('persistence_distance_m', 0.3)
        self.declare_parameter('persistence_bearing_magnitude', 0.6)
        self.declare_parameter('bearing_sign', BEARING_SIGN)

        self.engine_path = self.get_parameter('engine_path').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.assumed_person_height_m = self.get_parameter('assumed_person_height_m').value
        self.deadzone_ratio = self.get_parameter('deadzone_ratio').value
        self.loss_frame_threshold = self.get_parameter('loss_frame_threshold').value
        self.persistence_timeout_sec = self.get_parameter('persistence_timeout_sec').value
        self.persistence_distance_m = self.get_parameter('persistence_distance_m').value
        self.persistence_bearing_magnitude = self.get_parameter('persistence_bearing_magnitude').value
        self.bearing_sign = self.get_parameter('bearing_sign').value
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.bridge = CvBridge()
        self.get_logger().info(f'Loading YOLO engine: {self.engine_path}')
        self.model = YOLO(self.engine_path, task='detect')
        self.focal_px = None  # computed on first frame once we know image width

        self.miss_count = 0
        self.searching = False
        self.search_started_at = None
        self.exit_bearing_sign = 0.0
        self.last_valid_distance = self.persistence_distance_m

        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)

        self.detected_pub = self.create_publisher(Bool, '/target_detected', 10)
        self.bearing_pub = self.create_publisher(Float32, '/target_bearing', 10)
        self.distance_pub = self.create_publisher(Float32, '/target_distance', 10)
        # Web UI bbox overlay -- native name, remapped to /rear_target_bbox in
        # launch exactly like the other /target_* topics (see bringup.launch.py).
        self.bbox_pub = self.create_publisher(Float32MultiArray, '/target_bbox', 10)
        self.debug_image_pub = self.create_publisher(Image, '/rear_yolo_debug_image', 10)
        # See odin_yolo_tracker_node.py -- same additive stamped raw-detection
        # topic (native name, remapped to /rear_target_detection_raw in
        # launch exactly like the other /target_* topics).
        self.detection_pub = self.create_publisher(
            TargetDetection, '/target_detection_raw', 10)

        self.get_logger().info(
            f'rear_yolo_tracker_node started: conf={self.conf_threshold}, '
            f'deadzone_ratio={self.deadzone_ratio}, '
            f'persistence_timeout={self.persistence_timeout_sec}s, '
            'subscribed to /image_raw')

    def _on_set_parameters(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'bearing_sign':
                self.bearing_sign = p.value
                self.get_logger().info(f'bearing_sign -> {p.value:+.1f}')
        return SetParametersResult(successful=True)

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame_width = frame.shape[1]
        frame_height = frame.shape[0]

        if self.focal_px is None:
            hfov_rad = math.radians(ASSUMED_HORIZONTAL_FOV_DEG)
            self.focal_px = (frame_width / 2.0) / math.tan(hfov_rad / 2.0)

        results = self.model.predict(
            frame, conf=self.conf_threshold, classes=[0], verbose=False)
        boxes = results[0].boxes

        best_box = None
        best_area = -1.0
        if boxes is not None and len(boxes) > 0:
            xyxy = boxes.xyxy.cpu().numpy()
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
            box_height_px = max(1.0, y2 - y1)

            half_dead = self.deadzone_ratio * frame_width
            offset = center_x - frame_width / 2.0
            if abs(offset) <= half_dead:
                bearing = 0.0
            else:
                bearing = offset / (frame_width / 2.0)
                bearing = max(-1.0, min(1.0, bearing))

            distance = (self.assumed_person_height_m * self.focal_px) / box_height_px
            self.last_valid_distance = distance
            detected = True

            if debug_wanted:
                debug_frame = frame.copy()
                cv2.rectangle(debug_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                label = f'd~{distance:.2f}m b={bearing:+.2f}' + (' [reacquired]' if was_searching else '')
                cv2.putText(debug_frame, label, (int(x1), max(15, int(y1) - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            self.miss_count += 1

            if self.miss_count < self.loss_frame_threshold:
                detected = True
                bearing = 0.0
                distance = self.last_valid_distance
            else:
                if not self.searching:
                    self.searching = True
                    self.search_started_at = now

                elapsed = now - self.search_started_at
                if elapsed <= self.persistence_timeout_sec:
                    detected = True
                    bearing = self.exit_bearing_sign * self.persistence_bearing_magnitude
                    distance = self.persistence_distance_m
                else:
                    detected = False
                    bearing = 0.0
                    distance = 0.0

        if best_box is not None:
            x1, y1, x2, y2 = best_box
            center_x = (x1 + x2) / 2.0
            if center_x < frame_width / 3.0:
                self.exit_bearing_sign = -1.0
            elif center_x > 2.0 * frame_width / 3.0:
                self.exit_bearing_sign = 1.0
            else:
                self.exit_bearing_sign = 0.0

        # See odin_yolo_tracker_node.py: snapshot the real per-frame result
        # for the stamped raw topic before persistence/search substitutes a
        # held/fabricated value.
        raw_detected = best_box is not None
        raw_bearing = self.bearing_sign * (bearing if raw_detected else 0.0)
        raw_distance = distance if raw_detected else 0.0

        # See odin_yolo_tracker_node.py: applied last as a single point of
        # truth covering both the live-detection and persistence/search
        # branches above.
        bearing = self.bearing_sign * bearing

        self.detected_pub.publish(Bool(data=bool(detected)))
        self.bearing_pub.publish(Float32(data=float(bearing)))
        self.distance_pub.publish(Float32(data=float(distance)))

        raw_msg = TargetDetection()
        raw_msg.header = msg.header
        raw_msg.detected = bool(raw_detected)
        raw_msg.bearing = float(raw_bearing)
        raw_msg.distance = float(raw_distance)
        self.detection_pub.publish(raw_msg)

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
    node = RearYoloTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
