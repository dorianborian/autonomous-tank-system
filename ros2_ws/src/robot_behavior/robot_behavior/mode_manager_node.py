import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from robot_behavior.mode_constants import (
    DEFAULT_DRIVE_SOURCE,
    DEFAULT_TRACK_MODE,
    DRIVE_SOURCES,
    TOPIC_CURRENT_MODE,
    TOPIC_DRIVE_SOURCE_CMD,
    TOPIC_DRIVE_SOURCE_STATE,
    TOPIC_TRACK_MODE_CMD,
    TOPIC_TRACK_MODE_STATE,
    TRACK_MODES,
    latched_qos,
)

# Latched (transient_local) QoS so a node that starts after mode_manager
# still immediately receives the current state instead of waiting for the
# next change. Subscribers must use the SAME profile (see latched_qos docstring).
LATCHED_QOS = latched_qos()


class ModeManagerNode(Node):

    def __init__(self):
        super().__init__('mode_manager_node')

        self.drive_source = DEFAULT_DRIVE_SOURCE
        self.track_mode = DEFAULT_TRACK_MODE

        self.create_subscription(
            String, TOPIC_DRIVE_SOURCE_CMD, self._drive_source_cmd_cb, 10)
        self.create_subscription(
            String, TOPIC_TRACK_MODE_CMD, self._track_mode_cmd_cb, 10)

        self.drive_source_pub = self.create_publisher(
            String, TOPIC_DRIVE_SOURCE_STATE, LATCHED_QOS)
        self.track_mode_pub = self.create_publisher(
            String, TOPIC_TRACK_MODE_STATE, LATCHED_QOS)
        self.current_mode_pub = self.create_publisher(
            String, TOPIC_CURRENT_MODE, LATCHED_QOS)

        self._publish_state()

        self.get_logger().info(
            f'mode_manager_node started: drive_source={self.drive_source}, '
            f'track_mode={self.track_mode}')

    def _drive_source_cmd_cb(self, msg: String):
        value = msg.data.strip().lower()
        if value not in DRIVE_SOURCES:
            self.get_logger().warn(
                f'Rejected /drive_source_cmd="{msg.data}" — must be one of {DRIVE_SOURCES}')
            return
        if value != self.drive_source:
            self.get_logger().info(f'drive_source: {self.drive_source} -> {value}')
        self.drive_source = value
        self._publish_state()

    def _track_mode_cmd_cb(self, msg: String):
        value = msg.data.strip().lower()
        if value not in TRACK_MODES:
            self.get_logger().warn(
                f'Rejected /track_mode_cmd="{msg.data}" — must be one of {TRACK_MODES}')
            return
        if value != self.track_mode:
            self.get_logger().info(f'track_mode: {self.track_mode} -> {value}')
        self.track_mode = value
        self._publish_state()

    def _publish_state(self):
        ds_msg = String()
        ds_msg.data = self.drive_source
        self.drive_source_pub.publish(ds_msg)

        tm_msg = String()
        tm_msg.data = self.track_mode
        self.track_mode_pub.publish(tm_msg)

        combined_msg = String()
        combined_msg.data = f'{self.drive_source}:{self.track_mode}'
        self.current_mode_pub.publish(combined_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ModeManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
