import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32

# ODIN 1 front RGB camera is factory-calibrated (see odin_ws calib.yaml,
# cam_0: A11/A22 ~= 734.92px focal length at 1600x1296). We use that real
# focal length instead of the rear webcam's assumed-FOV approximation used
# in aruco_tracker_node — it's the same pinhole-distance approach, just fed
# a measured focal length rather than one derived from an assumed FOV.
# Note: the ODIN lens is a fisheye ("FishPoly" model per calib.yaml), so this
# pinhole approximation is least accurate for markers near the frame edges —
# same accuracy tradeoff already accepted for the rear tracker, just with a
# better center-frame starting point.
CALIBRATED_FOCAL_PX = 734.92
CALIBRATED_IMAGE_WIDTH_PX = 1600

# ODIN's native 1600x1296 frame is ~6.5x the pixel count of the rear webcam's
# 640x480, and ArUco corner detection cost scales with pixel count. tegrastats
# showed all 6 CPU cores at 65-75% and the GPU completely idle (0%) — OpenCV
# here has no CUDA build anyway, and ArUco detection has no GPU path in
# OpenCV regardless, so downsampling before detection is the real lever.
# Detection, bearing, and the calibrated focal length all operate consistently
# in this downsampled frame — no rescaling back to full-res is needed.
DOWNSAMPLE_FACTOR = 0.5


class OdinTargetTrackerNode(Node):

    def __init__(self):
        super().__init__('odin_target_tracker_node')

        self.declare_parameter('marker_size_m', 0.10)
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('calibrated_focal_px', CALIBRATED_FOCAL_PX)
        self.declare_parameter('calibrated_image_width_px', CALIBRATED_IMAGE_WIDTH_PX)
        self.declare_parameter('downsample_factor', DOWNSAMPLE_FACTOR)
        self.marker_size_m = self.get_parameter('marker_size_m').value
        dict_name = self.get_parameter('aruco_dictionary').value
        dict_id = getattr(cv2.aruco, dict_name)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.calibrated_focal_px = self.get_parameter('calibrated_focal_px').value
        self.calibrated_image_width_px = self.get_parameter('calibrated_image_width_px').value
        self.downsample_factor = self.get_parameter('downsample_factor').value

        self.bridge = CvBridge()
        self.focal_px = None  # scaled to actual frame width on first frame

        self.image_sub = self.create_subscription(
            Image, '/odin1/image', self.image_callback, 10)

        self.detected_pub = self.create_publisher(Bool, '/odin_target_detected', 10)
        self.bearing_pub = self.create_publisher(Float32, '/odin_target_bearing', 10)
        self.distance_pub = self.create_publisher(Float32, '/odin_target_distance', 10)
        self.debug_image_pub = self.create_publisher(Image, '/odin_aruco_debug_image', 10)

        self.get_logger().info(
            f'odin_target_tracker_node started: dictionary={dict_name}, '
            f'marker_size_m={self.marker_size_m}, subscribed to /odin1/image')

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.downsample_factor < 1.0:
            frame = cv2.resize(
                frame, None, fx=self.downsample_factor, fy=self.downsample_factor,
                interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.focal_px is None:
            width = frame.shape[1]
            self.focal_px = self.calibrated_focal_px * (width / self.calibrated_image_width_px)

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
    node = OdinTargetTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
