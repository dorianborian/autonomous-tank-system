import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String

from robot_behavior.mode_constants import (
    DEFAULT_DRIVE_SOURCE,
    TOPIC_DRIVE_SOURCE_CMD,
    TOPIC_DRIVE_SOURCE_STATE,
    latched_qos,
)

# --- Tunable constants -------------------------------------------------
# Axis mapping matches the stock teleop_twist_joy PS5 config shipped with
# this ROS install (/opt/ros/jazzy/share/teleop_twist_joy/config/ps5.config.yaml):
# left stick vertical = axis 1 (linear.x), left stick horizontal = axis 0
# (angular.z). No sign inversion is applied, matching that reference file.
# VERIFY against the real controller during step 9a — flip the sign via the
# axis_linear_sign/axis_angular_sign parameters below if forward reads negative.
AXIS_LINEAR = 1
AXIS_ANGULAR = 0
AXIS_LINEAR_SIGN = 1.0
AXIS_ANGULAR_SIGN = 1.0

MAX_LINEAR_X = 1.0   # m/s, matches serial_driver's max_linear_speed default (full scale)
# NOTE: serial_driver's own declared max_angular_speed parameter is dead code —
# its differential-drive mixing only uses TRACK_WIDTH (0.2286m), never
# max_angular_speed. So the only real lever over how much wheel-speed split a
# given stick deflection produces is this MAX_ANGULAR_Z. Live-tested on
# blocks at speed_scale=0.25: 2.0 rad/s gave only a +/-5.7% split (too subtle
# to feel); raised to 4.0 rad/s per your feedback for a clearly noticeable turn.
MAX_ANGULAR_Z = 4.0  # rad/s

# bias_cmd_vel is the "nudge" applied on top of autonomous tracking in semi
# mode. Spec calls for roughly 20-30% of full scale; 25% picked as the
# midpoint. Tunable.
BIAS_SCALE = 0.25

# Raised 0.05 -> 0.1 (2026-07-24, live web UI testing via the Steam Deck's
# built-in sticks): 0.05 let small resting/analog-drift noise on the Deck's
# sticks through as a nonzero command. Applies to both manual (raw stick)
# and semi (bias overlay is computed from the already-deadzoned manual
# value below), which is intended -- same "don't react to noise" rationale
# for both.
STICK_DEADZONE = 0.1

# PS/Home (guide) button. NOT verified against real hardware in this session
# (no controller was paired) — confirm the real index via `ros2 topic echo
# /joy` and press the physical PS button, then adjust the estop_button_index
# parameter if it differs.
ESTOP_BUTTON_INDEX = 10

# Deadman gate for autonomous (auto/semi) driving — added 2026-07-14 per user
# request: intent_to_goal / DWB keep computing goals and planning regardless
# (so the goal marker and route stay visible on the costmap viewer at all
# times), but drive_mux zeroes the actual motor output for auto/semi unless
# this is held. Manual mode is UNCHANGED (raw joystick, no gate) — releasing
# the stick already stops it there, so a deadman requirement would be
# redundant. VERIFIED against real hardware 2026-07-14: R2 reads as a clean
# digital 0->1 on buttons[7] (not just the analog axis[4] trigger travel),
# so the boolean button index is used rather than an axis threshold.
DEADMAN_BUTTON_INDEX = 7

# Goal-authoring lock/delete (2026-07-14, user redesign): in semi mode with
# no marker, the joystick moves a CANDIDATE goal preview (visualized but not
# executed). X locks it in as a fixed, stable target; O deletes it. This
# replaces continuously re-projecting the goal from current heading every
# tick, which caused the goal to drift/never return to the original line
# once the robot deviated even slightly — see intent_to_goal_node.py module
# docstring. VERIFIED against real hardware 2026-07-14 via press-edge
# sampling on /joy (X pressed first, then O, in that order):
#   X (lock)   = buttons[1]
#   O (delete) = buttons[2]
# NOT the "standard" PS5 mapping some references use (0/1) — this controller
# reads differently, hence live verification rather than assuming.
LOCK_BUTTON_INDEX = 1
DELETE_BUTTON_INDEX = 2

# Door open/close in MANUAL mode only — 2026-07-15 explicit request.
# Reuses the SAME physical button as LOCK_BUTTON_INDEX (buttons[1], VERIFIED
# real hardware — user confirmed "the button index is 1"). No conflict: door
# control here is gated to drive_source == 'manual', and the existing lock
# logic above only has any effect in 'semi' (intent_to_goal ignores 'lock'
# outside semi mode) — the two drive states are mutually exclusive, so the
# same button doing different things in different modes is safe, not a race.
# Behavior: buttons[1] read as a direct on/off STATE (not a momentary press
# to toggle) — 0->1 edge sends 'open', 1->0 edge sends 'close', matching
# serial_driver's own one-shot-per-transition door protocol (_door_cmd_cb).
DOOR_BUTTON_INDEX = 1
# -------------------------------------------------------------------


def apply_deadzone(value, deadzone):
    return 0.0 if abs(value) < deadzone else value


class PsControllerBridgeNode(Node):

    def __init__(self):
        super().__init__('ps_controller_bridge_node')

        self.declare_parameter('axis_linear', AXIS_LINEAR)
        self.declare_parameter('axis_angular', AXIS_ANGULAR)
        self.declare_parameter('axis_linear_sign', AXIS_LINEAR_SIGN)
        self.declare_parameter('axis_angular_sign', AXIS_ANGULAR_SIGN)
        self.declare_parameter('max_linear_x', MAX_LINEAR_X)
        self.declare_parameter('max_angular_z', MAX_ANGULAR_Z)
        self.declare_parameter('bias_scale', BIAS_SCALE)
        self.declare_parameter('stick_deadzone', STICK_DEADZONE)
        self.declare_parameter('estop_button_index', ESTOP_BUTTON_INDEX)
        self.declare_parameter('deadman_button_index', DEADMAN_BUTTON_INDEX)
        self.declare_parameter('lock_button_index', LOCK_BUTTON_INDEX)
        self.declare_parameter('delete_button_index', DELETE_BUTTON_INDEX)
        self.declare_parameter('door_button_index', DOOR_BUTTON_INDEX)

        self.axis_linear = self.get_parameter('axis_linear').value
        self.axis_angular = self.get_parameter('axis_angular').value
        self.axis_linear_sign = self.get_parameter('axis_linear_sign').value
        self.axis_angular_sign = self.get_parameter('axis_angular_sign').value
        self.max_linear_x = self.get_parameter('max_linear_x').value
        self.max_angular_z = self.get_parameter('max_angular_z').value
        self.bias_scale = self.get_parameter('bias_scale').value
        self.stick_deadzone = self.get_parameter('stick_deadzone').value
        self.estop_button_index = self.get_parameter('estop_button_index').value
        self.deadman_button_index = self.get_parameter('deadman_button_index').value
        self.lock_button_index = self.get_parameter('lock_button_index').value
        self.delete_button_index = self.get_parameter('delete_button_index').value
        self.door_button_index = self.get_parameter('door_button_index').value

        self._estop_button_prev = False
        self._lock_button_prev = False
        self._delete_button_prev = False
        self._door_button_prev = False
        self.drive_source = DEFAULT_DRIVE_SOURCE

        self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.create_subscription(
            String, TOPIC_DRIVE_SOURCE_STATE, self._drive_source_cb, latched_qos())

        self.manual_cmd_pub = self.create_publisher(Twist, '/manual_cmd_vel', 10)
        self.bias_cmd_pub = self.create_publisher(Twist, '/bias_cmd_vel', 10)
        self.drive_source_cmd_pub = self.create_publisher(String, TOPIC_DRIVE_SOURCE_CMD, 10)
        self.nav_enable_pub = self.create_publisher(Bool, '/nav_enable', 10)
        self.goal_lock_cmd_pub = self.create_publisher(String, '/goal_lock_cmd', 10)
        self.door_cmd_pub = self.create_publisher(String, '/door_cmd', 10)

        self.get_logger().info(
            f'ps_controller_bridge_node started: bias_scale={self.bias_scale}, '
            f'estop_button_index={self.estop_button_index}, '
            f'deadman_button_index={self.deadman_button_index} (R2), '
            f'lock_button_index={self.lock_button_index} (X), '
            f'delete_button_index={self.delete_button_index} (O), '
            f'door_button_index={self.door_button_index} (manual mode only) '
            '— all verified real hardware')

    def _drive_source_cb(self, msg: String):
        self.drive_source = msg.data.strip().lower()

    def _joy_cb(self, msg: Joy):
        raw_linear = 0.0
        raw_angular = 0.0
        if self.axis_linear < len(msg.axes):
            raw_linear = apply_deadzone(msg.axes[self.axis_linear], self.stick_deadzone)
            raw_linear *= self.axis_linear_sign
        if self.axis_angular < len(msg.axes):
            raw_angular = apply_deadzone(msg.axes[self.axis_angular], self.stick_deadzone)
            raw_angular *= self.axis_angular_sign

        manual = Twist()
        manual.linear.x = raw_linear * self.max_linear_x
        manual.angular.z = raw_angular * self.max_angular_z
        self.manual_cmd_pub.publish(manual)

        bias = Twist()
        bias.linear.x = manual.linear.x * self.bias_scale
        bias.angular.z = manual.angular.z * self.bias_scale
        self.bias_cmd_pub.publish(bias)

        self._check_estop(msg)
        self._publish_nav_enable(msg)
        self._check_goal_lock_buttons(msg)
        self._check_door_button(msg)

    def _check_door_button(self, msg: Joy):
        if self.drive_source != 'manual':
            # Not gating the READ of the button, just whether we act on it —
            # avoids publishing door commands as a side effect of aiming/
            # locking a semi-mode goal on the same physical button.
            return
        if self.door_button_index >= len(msg.buttons):
            return
        pressed = bool(msg.buttons[self.door_button_index])
        if pressed and not self._door_button_prev:
            self.door_cmd_pub.publish(String(data='open'))
            self.get_logger().info('Door button -> OPEN (manual mode)')
        elif not pressed and self._door_button_prev:
            self.door_cmd_pub.publish(String(data='close'))
            self.get_logger().info('Door button -> CLOSE (manual mode)')
        self._door_button_prev = pressed

    def _check_goal_lock_buttons(self, msg: Joy):
        lock_pressed = (self.lock_button_index < len(msg.buttons)
                        and bool(msg.buttons[self.lock_button_index]))
        delete_pressed = (self.delete_button_index < len(msg.buttons)
                          and bool(msg.buttons[self.delete_button_index]))
        if lock_pressed and not self._lock_button_prev:
            self.goal_lock_cmd_pub.publish(String(data='lock'))
        if delete_pressed and not self._delete_button_prev:
            self.goal_lock_cmd_pub.publish(String(data='delete'))
        self._lock_button_prev = lock_pressed
        self._delete_button_prev = delete_pressed

    def _publish_nav_enable(self, msg: Joy):
        held = (self.deadman_button_index < len(msg.buttons)
                and bool(msg.buttons[self.deadman_button_index]))
        self.nav_enable_pub.publish(Bool(data=held))

    def _check_estop(self, msg: Joy):
        if self.estop_button_index >= len(msg.buttons):
            return
        pressed = bool(msg.buttons[self.estop_button_index])
        if pressed and not self._estop_button_prev:
            self.get_logger().warn('E-STOP button pressed -> forcing drive_source=manual + zero Twist')
            mode_msg = String()
            mode_msg.data = 'manual'
            self.drive_source_cmd_pub.publish(mode_msg)
            self.manual_cmd_pub.publish(Twist())
        self._estop_button_prev = pressed


def main(args=None):
    rclpy.init(args=args)
    node = PsControllerBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
