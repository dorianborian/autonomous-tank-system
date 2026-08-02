"""Shared mode-model constants used across the Phase 6 behaviour nodes."""

from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


def latched_qos():
    """QoS for the mode-state topics (drive_source_state / track_mode_state /
    current_mode). mode_manager publishes these transient_local so that a node
    which starts (or restarts) AFTER a mode was last set still gets the current
    value on join. That only works if the SUBSCRIBER also uses transient_local
    durability — a volatile subscriber silently receives no historical sample
    and runs with its default mode until the next command (found the hard way:
    a restarted target_pid stayed in 'follow' while the system was in 'flee').
    Publishers and subscribers must both use this profile.
    """
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


DRIVE_SOURCES = ('manual', 'semi', 'auto')
TRACK_MODES = ('follow', 'flee')

DEFAULT_DRIVE_SOURCE = 'manual'  # boot safe: nothing drives autonomously until commanded
DEFAULT_TRACK_MODE = 'follow'

TOPIC_DRIVE_SOURCE_CMD = '/drive_source_cmd'
TOPIC_TRACK_MODE_CMD = '/track_mode_cmd'
TOPIC_DRIVE_SOURCE_STATE = '/drive_source_state'
TOPIC_TRACK_MODE_STATE = '/track_mode_state'
TOPIC_CURRENT_MODE = '/current_mode'
