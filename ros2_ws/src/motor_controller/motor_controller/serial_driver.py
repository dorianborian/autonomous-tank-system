import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial


# Measured axle-to-axle / track centerline-to-centerline from the real chassis.
TRACK_WIDTH = 0.2286  # meters

BAUD_RATE = 115200
TIMER_HZ = 20       # Must be >= 2x watchdog frequency; ESP32 watchdog triggers at 500ms
LOG_HZ = 1          # Log rate to avoid terminal flooding

# Motors don't physically turn below this commanded speed (same [-100, 100]
# scale as _left_speed/_right_speed) — live-tested, ~10 is where they start
# moving. Any nonzero-but-smaller command gets boosted to this floor
# (preserving sign); an exact 0 command is left at 0 so the robot can still
# actually stop.
MIN_EFFECTIVE_SPEED = 10.0

# Left wheel's own calibrated floor — see left/right_min_effective_speed
# parameter comments below. Live-tuned and CONFIRMED working (user drove
# straight at low throttle in manual mode) 2026-07-14; persisted here as the
# real default so it survives a restart instead of living only in a
# `ros2 param set` that a power cycle wipes out.
LEFT_MIN_EFFECTIVE_SPEED = 14.0

# Split into independent forward/reverse floors per wheel (2026-07-25, web UI
# settings menu request) -- previously one floor per wheel covered both
# directions via sign-preserving apply_min_effective_speed. Defaults below
# reproduce the OLD single-floor behavior exactly (forward == reverse per
# wheel) until actually retuned apart; nothing changes unless someone moves
# a slider.
LEFT_MIN_FORWARD_SPEED = LEFT_MIN_EFFECTIVE_SPEED
LEFT_MIN_REVERSE_SPEED = LEFT_MIN_EFFECTIVE_SPEED
RIGHT_MIN_FORWARD_SPEED = MIN_EFFECTIVE_SPEED
RIGHT_MIN_REVERSE_SPEED = MIN_EFFECTIVE_SPEED


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def apply_min_effective_speed(speed, forward_floor, reverse_floor):
    if speed == 0.0:
        return 0.0
    if speed > 0.0:
        return forward_floor if speed < forward_floor else speed
    return -reverse_floor if -speed < reverse_floor else speed


def speed_to_byte(speed_normalized):
    """Convert signed speed in [-100, 100] to protocol byte [0, 200]."""
    return clamp(int(round(speed_normalized + 100)), 0, 200)


class SerialDriverNode(Node):
    def __init__(self):
        super().__init__('serial_driver')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('max_linear_speed', 1.0)   # m/s at byte 200 (full forward)
        self.declare_parameter('max_angular_speed', 2.0)  # rad/s at full spin
        # speed_scale caps motor output as a fraction of full power [0.0–1.0].
        # Default 0.15 = 15% max power, safe for initial bench testing.
        # Raise gradually once motor direction and trim are confirmed correct.
        self.declare_parameter('speed_scale', 0.15)
        # Wheel trim: a straight (angular=0) command produces IDENTICAL
        # left/right speeds out of the differential-drive mixing below —
        # confirmed no asymmetry in that math, so any real turn-during-straight
        # is downstream of this node. Kept available (both default 1.0, no-op)
        # for a genuine PROPORTIONAL gain difference between sides, but see
        # left/right_min_effective_speed below for the specific issue actually
        # observed 2026-07-14: live-tested, the left tread is slower than the
        # right ONLY below ~50% throttle, and they sync up above that — a
        # STICTION/breakaway-threshold difference (left motor needs a higher
        # minimum command before it starts turning), not a proportional one.
        # A multiplicative trim would be the WRONG tool for that: it stays
        # proportional at high speed too, and would reintroduce an asymmetry
        # there that doesn't currently exist. Left as a separate, independent
        # knob in case a real proportional difference shows up later.
        self.declare_parameter('left_trim', 1.0)
        self.declare_parameter('right_trim', 1.0)
        # Per-wheel, per-direction minimum-effective-speed floor — THE fix
        # for the low-speed asymmetry above, further split (2026-07-25) so
        # forward and reverse can be tuned independently per wheel via the
        # web UI settings menu, since there's no guarantee a wheel's
        # stiction/breakaway threshold is symmetric in both directions.
        # TUNE LIVE (`ros2 param set /serial_driver left_min_forward_speed
        # <value>`, etc.) at low throttle in manual mode until it goes
        # straight in both directions, then persist as the new defaults above.
        self.declare_parameter('left_min_forward_speed', LEFT_MIN_FORWARD_SPEED)
        self.declare_parameter('left_min_reverse_speed', LEFT_MIN_REVERSE_SPEED)
        self.declare_parameter('right_min_forward_speed', RIGHT_MIN_FORWARD_SPEED)
        self.declare_parameter('right_min_reverse_speed', RIGHT_MIN_REVERSE_SPEED)

        port = self.get_parameter('port').get_parameter_value().string_value
        self.max_linear = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.speed_scale = clamp(
            self.get_parameter('speed_scale').get_parameter_value().double_value, 0.0, 1.0
        )
        self.left_trim = self.get_parameter('left_trim').get_parameter_value().double_value
        self.right_trim = self.get_parameter('right_trim').get_parameter_value().double_value
        self.left_min_forward_speed = (
            self.get_parameter('left_min_forward_speed').get_parameter_value().double_value
        )
        self.left_min_reverse_speed = (
            self.get_parameter('left_min_reverse_speed').get_parameter_value().double_value
        )
        self.right_min_forward_speed = (
            self.get_parameter('right_min_forward_speed').get_parameter_value().double_value
        )
        self.right_min_reverse_speed = (
            self.get_parameter('right_min_reverse_speed').get_parameter_value().double_value
        )

        self._left_speed = 0.0   # signed, range [-100, 100]
        self._right_speed = 0.0
        self._door_byte = 2      # 2 = no change; only 0/1 sent immediately after a door command
        self._door_cmd_pending = False

        self._log_counter = 0
        self._log_every = TIMER_HZ // LOG_HZ

        try:
            self._serial = serial.Serial(port, BAUD_RATE, timeout=0.1)
            self.get_logger().info(f'Opened serial port {port} at {BAUD_RATE} baud')
        except serial.SerialException as e:
            self.get_logger().fatal(f'Failed to open {port}: {e}')
            raise

        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(String, '/door_cmd', self._door_cmd_cb, 10)
        # Live-tunable trim: other params here are read once at startup, but
        # left_trim/right_trim are meant to be iterated on with the robot
        # actually driving (`ros2 param set /serial_driver left_trim <val>`),
        # so keep self.left_trim/right_trim in sync with the live param value.
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # Timer drives packet transmission at fixed 20 Hz regardless of topic rate.
        # This is critical: the ESP32 watchdog kills motors if no valid packet arrives
        # within 500ms. We must send continuously, not only on new /cmd_vel messages.
        self.create_timer(1.0 / TIMER_HZ, self._timer_cb)

        self.get_logger().info(
            f'serial_driver ready — sending at {TIMER_HZ} Hz | '
            f'speed_scale={self.speed_scale:.2f} ({self.speed_scale*100:.0f}% max power) | '
            f'track_width={TRACK_WIDTH}m | trim L={self.left_trim:.2f} R={self.right_trim:.2f} | '
            f'min_speed L(fwd/rev)={self.left_min_forward_speed:.1f}/{self.left_min_reverse_speed:.1f} '
            f'R(fwd/rev)={self.right_min_forward_speed:.1f}/{self.right_min_reverse_speed:.1f}'
        )

    def _on_set_parameters(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'left_trim':
                self.left_trim = p.value
                self.get_logger().info(f'left_trim -> {p.value:.3f}')
            elif p.name == 'right_trim':
                self.right_trim = p.value
                self.get_logger().info(f'right_trim -> {p.value:.3f}')
            elif p.name == 'left_min_forward_speed':
                self.left_min_forward_speed = p.value
                self.get_logger().info(f'left_min_forward_speed -> {p.value:.2f}')
            elif p.name == 'left_min_reverse_speed':
                self.left_min_reverse_speed = p.value
                self.get_logger().info(f'left_min_reverse_speed -> {p.value:.2f}')
            elif p.name == 'right_min_forward_speed':
                self.right_min_forward_speed = p.value
                self.get_logger().info(f'right_min_forward_speed -> {p.value:.2f}')
            elif p.name == 'right_min_reverse_speed':
                self.right_min_reverse_speed = p.value
                self.get_logger().info(f'right_min_reverse_speed -> {p.value:.2f}')
            elif p.name == 'speed_scale':
                # speed_scale was declared as a parameter but, unlike trim/
                # min_effective_speed above, was previously only ever read
                # ONCE at startup into self.speed_scale -- `ros2 param set`
                # would update the parameter server's stored value but never
                # reach the actual scaling math in _cmd_vel_cb. Added here
                # (2026-07-24) so the web UI's max-speed control actually
                # works live, same clamp as the startup read.
                self.speed_scale = clamp(p.value, 0.0, 1.0)
                self.get_logger().info(
                    f'speed_scale -> {self.speed_scale:.2f} '
                    f'({self.speed_scale*100:.0f}% max power)')
        return SetParametersResult(successful=True)

    def _cmd_vel_cb(self, msg: Twist):
        linear = msg.linear.x    # m/s
        angular = msg.angular.z  # rad/s

        # Textbook diff-drive is v_left = v - omega*d/2, v_right = v + omega*d/2
        # (REP-103: +omega = CCW = turn left = outer/right wheel speeds up).
        # INVERTED here (2026-07-25): live-tested with a real +omega command
        # while watching /odin1/odometry yaw directly (unwrapped, short burst
        # to avoid angle-wrap ambiguity) -- a commanded +0.4 rad/s produced a
        # consistent, repeatable NEGATIVE (clockwise/right) yaw change, the
        # opposite of REP-103. This single inversion was the actual root
        # cause of two reported bugs upstream (follow steering the wrong way,
        # flee "spinning in a circle" -- the latter because DWB's heading
        # correction became POSITIVE feedback: it always steered further from
        # the intended direction, not toward it). Root physical cause not
        # isolated (could be L/R physically swapped at the connector, one
        # channel's motor polarity reversed, or the ODIN's own odometry yaw
        # sign -- doesn't matter which: everything upstream (DWB, Nav2,
        # intent_to_goal_node, ps_controller_bridge) already assumes standard
        # REP-103 and is internally consistent with the SAME odom frame this
        # was tested against, so correcting the sign here makes the whole
        # chain self-consistent regardless of which physical link was
        # actually reversed.
        v_left = linear + (angular * TRACK_WIDTH / 2.0)
        v_right = linear - (angular * TRACK_WIDTH / 2.0)

        # Normalize to [-100, 100], then apply speed_scale to cap max motor power
        scale = 100.0 / self.max_linear
        left_speed = clamp(v_left * scale, -100.0, 100.0) * self.speed_scale
        right_speed = clamp(v_right * scale, -100.0, 100.0) * self.speed_scale

        # Wheel trim: compensates for real per-side motor/gearing/wiring
        # variance (see left_trim/right_trim parameter comments in __init__).
        # Applied after speed_scale, re-clamped in case trim pushes over 100.
        left_speed = clamp(left_speed * self.left_trim, -100.0, 100.0)
        right_speed = clamp(right_speed * self.right_trim, -100.0, 100.0)

        # Small-but-nonzero commands never actually move the motors — boost
        # them up to each wheel's OWN floor instead (exact zero stays zero).
        # Independent per-wheel, per-direction floors — see the
        # left/right_min_forward/reverse_speed comments in __init__.
        self._left_speed = apply_min_effective_speed(
            left_speed, self.left_min_forward_speed, self.left_min_reverse_speed)
        self._right_speed = apply_min_effective_speed(
            right_speed, self.right_min_forward_speed, self.right_min_reverse_speed)

    def _door_cmd_cb(self, msg: String):
        # BANDAID (2026-07-15): the door servo's physical open/close states
        # are wired/firmware-inverted relative to the ESP32 protocol's byte
        # meaning, and the ESP32 can't be reflashed right now. Swapped here
        # instead — byte 0 now sent for a logical "open" and byte 1 for
        # "close" — so /door_cmd's meaning (and everything upstream, e.g.
        # ps_controller_bridge's door button) stays correct; only this one
        # mapping is inverted to compensate. Real fix, if the firmware is
        # ever reflashed: swap it back to the "natural" 1=open/0=close below
        # and remove this comment.
        cmd = msg.data.strip().lower()
        if cmd == 'open':
            self._door_byte = 0
            self._door_cmd_pending = True
            self.get_logger().info('Door command: OPEN (byte=0, inverted-servo bandaid)')
        elif cmd == 'close':
            self._door_byte = 1
            self._door_cmd_pending = True
            self.get_logger().info('Door command: CLOSE (byte=1, inverted-servo bandaid)')
        else:
            self.get_logger().warn(f'Unknown door command: "{msg.data}" — expected "open" or "close"')

    def _timer_cb(self):
        left_byte = speed_to_byte(self._left_speed)
        right_byte = speed_to_byte(self._right_speed)

        # Send the actual door command (0 or 1) only on the first packet after a fresh
        # door command, then revert to 2 (no change) so we don't re-trigger every packet.
        if self._door_cmd_pending:
            door_byte = self._door_byte
            self._door_cmd_pending = False
        else:
            door_byte = 2

        packet = bytes([0xFF, left_byte, right_byte, door_byte])
        try:
            self._serial.write(packet)
            self._serial.flush()
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write failed: {e}')

        self._log_counter += 1
        if self._log_counter >= self._log_every:
            self._log_counter = 0
            self.get_logger().info(
                f'TX -> L={left_byte} R={right_byte} D={door_byte} '
                f'(speeds: L={self._left_speed:.1f} R={self._right_speed:.1f})'
            )

    def destroy_node(self):
        # Send neutral packet before closing so motors don't coast on shutdown
        try:
            self._serial.write(bytes([0xFF, 100, 100, 2]))
            self._serial.flush()
        except Exception:
            pass
        self._serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
