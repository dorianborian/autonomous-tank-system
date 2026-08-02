import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from robot_behavior.mode_constants import (
    DEFAULT_TRACK_MODE,
    TOPIC_TRACK_MODE_STATE,
    latched_qos,
)


class TargetSelectorNode(Node):
    """Republishes ONE unified target_* topic set sourced from whichever
    tracker (front ODIN for follow, rear webcam for flee) matches the
    current track_mode. Downstream (target_pid) only ever needs to consume
    /target_detected, /target_bearing, /target_distance.
    """

    def __init__(self):
        super().__init__('target_selector_node')

        self.track_mode = DEFAULT_TRACK_MODE

        self.create_subscription(
            String, TOPIC_TRACK_MODE_STATE, self._track_mode_cb, latched_qos())

        self.create_subscription(
            Bool, '/odin_target_detected', self._make_cb('follow', self._pub_detected), 10)
        self.create_subscription(
            Float32, '/odin_target_bearing', self._make_cb('follow', self._pub_bearing), 10)
        self.create_subscription(
            Float32, '/odin_target_distance', self._make_cb('follow', self._pub_distance), 10)

        self.create_subscription(
            Bool, '/rear_target_detected', self._make_cb('flee', self._pub_detected), 10)
        self.create_subscription(
            Float32, '/rear_target_bearing', self._make_cb('flee', self._pub_bearing), 10)
        self.create_subscription(
            Float32, '/rear_target_distance', self._make_cb('flee', self._pub_distance), 10)

        self.detected_pub = self.create_publisher(Bool, '/target_detected', 10)
        self.bearing_pub = self.create_publisher(Float32, '/target_bearing', 10)
        self.distance_pub = self.create_publisher(Float32, '/target_distance', 10)

        self.get_logger().info(
            f'target_selector_node started: track_mode={self.track_mode} '
            '(follow -> ODIN front camera, flee -> rear webcam)')

    def _track_mode_cb(self, msg: String):
        value = msg.data.strip().lower()
        if value != self.track_mode:
            self.get_logger().info(f'target_selector track_mode: {self.track_mode} -> {value}')
        self.track_mode = value

    def _make_cb(self, source_mode, publish_fn):
        def _cb(msg):
            if self.track_mode == source_mode:
                publish_fn(msg)
        return _cb

    def _pub_detected(self, msg: Bool):
        self.detected_pub.publish(msg)

    def _pub_bearing(self, msg: Float32):
        self.bearing_pub.publish(msg)

    def _pub_distance(self, msg: Float32):
        self.distance_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetSelectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
