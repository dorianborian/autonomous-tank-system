import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32

# Horizontal field of view for this rear webcam (per spec: 120 degrees),
# used to derive an approximate pixel focal length for distance estimation
# since this camera has no calibration file. Replace with a real calibration
# (camera_info) for more accurate distance later.
ASSUMED_HORIZONTAL_FOV_DEG = 120.0


class ArucoTrackerNode(Node):

    def __init__(self):
        super().__init__('aruco_tracker_node')

        self.declare_parameter('marker_size_m', 0.10)
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.marker_size_m = self.get_parameter('marker_size_m').value
        dict_name = self.get_parameter('aruco_dictionary').value
        dict_id = getattr(cv2.aruco, dict_name)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.bridge = CvBridge()
        self.focal_px = None  # computed on first frame once we know image width

        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)

        self.detected_pub = self.create_publisher(Bool, '/target_detected', 10)
        self.bearing_pub = self.create_publisher(Float32, '/target_bearing', 10)
        self.distance_pub = self.create_publisher(Float32, '/target_distance', 10)
        self.debug_image_pub = self.create_publisher(Image, '/aruco_debug_image', 10)

        self.get_logger().info(
            f'aruco_tracker_node started: dictionary={dict_name}, '
            f'marker_size_m={self.marker_size_m}')

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.focal_px is None:
            width = frame.shape[1]
            hfov_rad = math.radians(ASSUMED_HORIZONTAL_FOV_DEG)
            self.focal_px = (width / 2.0) / math.tan(hfov_rad / 2.0)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)

        detected = ids is not None and len(ids) > 0

        detected_msg = Bool()
        detected_msg.data = bool(detected)
        self.detected_pub.publish(detected_msg)

        debug_frame = frame
        if detected:
            debug_frame = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)

            marker_corners = corners[0][0]  # first detected marker, shape (4, 2)
            center_x = float(np.mean(marker_corners[:, 0]))
            frame_width = frame.shape[1]
            bearing = (center_x - frame_width / 2.0) / (frame_width / 2.0)
            bearing = max(-1.0, min(1.0, bearing))

            side_lengths = [
                np.linalg.norm(marker_corners[i] - marker_corners[(i + 1) % 4])
                for i in range(4)
            ]
            avg_side_px = float(np.mean(side_lengths))
            distance = (self.marker_size_m * self.focal_px) / avg_side_px

            bearing_msg = Float32()
            bearing_msg.data = bearing
            self.bearing_pub.publish(bearing_msg)

            distance_msg = Float32()
            distance_msg.data = distance
            self.distance_pub.publish(distance_msg)

            marker_id = int(ids[0][0])
            label = f'ID {marker_id}  d={distance:.2f}m  b={bearing:+.2f}'
            text_origin = (
                int(marker_corners[:, 0].min()),
                max(15, int(marker_corners[:, 1].min()) - 10),
            )
            cv2.putText(debug_frame, label, text_origin,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            bearing_msg = Float32()
            bearing_msg.data = 0.0
            self.bearing_pub.publish(bearing_msg)

            distance_msg = Float32()
            distance_msg.data = 0.0
            self.distance_pub.publish(distance_msg)

        debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_image_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
