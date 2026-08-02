"""Phase 7 live test instrument panel.

A read-only monitor (publishes NOTHING, commands nothing) that prints one
refreshing status line so you can watch the whole Nav2 pipeline at a glance
while conducting real-world tests. Run it in its own terminal:

    ros2 run robot_behavior test_monitor_node

Line format:
  [auto:follow] tgt=DET b=+0.12 d=2.30 | goal=(1.7,-0.9) | cm L=29 I=357 |
     nav vx=+0.31 wz=-0.20 | EN=HELD | CMD vx=+0.31 wz=-0.20 | load=3.9 |
     age j=0.03 cm=0.2

  mode      -> /current_mode (drive_source:track_mode)
  tgt       -> /target_detected + bearing[-1,1] + distance[m]
  goal      -> /intent_goal_debug (x,y in odom) — the ACTIVE goal (marker,
               or a LOCKED joystick goal being routed to). "--" if none.
  cand      -> /candidate_goal_debug — unlocked joystick preview (aiming,
               not yet executing). "--" if stick is neutral or already
               locked. Press X to lock (moves into "goal" above), O to clear.
  cm L/I    -> local costmap lethal / inflated cell counts
  nav       -> /nav2/cmd_vel_smoothed (what Nav2 wants, post-smoother —
               this keeps updating even while EN=released, since intent_to_goal
               and DWB plan regardless of the deadman gate)
  EN        -> /nav_enable (R2 deadman gate, drive_mux). HELD = R2 pressed,
               auto/semi output can reach the motors. released = auto/semi
               forced to zero regardless of nav/goal (manual is never gated).
  CMD       -> /cmd_vel (final, what actually reaches the motors). In
               auto/semi, nav != CMD while EN=released is expected — that's
               the gate doing its job, not a bug.
  load      -> system 1-min load average (6 cores => 6.0 == saturated)
  age       -> staleness (s) of /joy (j) and costmap (cm); rising j = controller
               dropping out (=> watchdog may force auto!)
"""

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32, String

from robot_behavior.mode_constants import latched_qos


class TestMonitorNode(Node):

    def __init__(self):
        super().__init__('test_monitor_node')

        self.mode = '?'
        self.detected = False
        self.bearing = 0.0
        self.distance = 0.0
        self.goal = None
        self.candidate = None
        self.cm_lethal = 0
        self.cm_inflated = 0
        self.nav = Twist()
        self.cmd = Twist()
        self.nav_enable = False
        self._joy_t = None
        self._cm_t = None
        self._goal_t = None
        self._candidate_t = None
        self._nav_enable_t = None

        self.create_subscription(String, '/current_mode', self._mode_cb, latched_qos())
        self.create_subscription(Bool, '/target_detected', self._det_cb, 10)
        self.create_subscription(Float32, '/target_bearing', self._brg_cb, 10)
        self.create_subscription(Float32, '/target_distance', self._dst_cb, 10)
        self.create_subscription(PoseStamped, '/intent_goal_debug', self._goal_cb, latched_qos())
        self.create_subscription(
            PoseStamped, '/candidate_goal_debug', self._candidate_cb, latched_qos())
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self._cm_cb, 1)
        self.create_subscription(Twist, '/nav2/cmd_vel_smoothed', self._nav_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.create_subscription(Bool, '/nav_enable', self._nav_enable_cb, 10)
        self.create_subscription(Joy, '/joy', self._joy_cb, 10)

        self.create_timer(0.5, self._print_line)
        self.get_logger().info('test_monitor_node started (read-only).')

    def _mode_cb(self, m): self.mode = m.data
    def _det_cb(self, m): self.detected = bool(m.data)
    def _brg_cb(self, m): self.bearing = float(m.data)
    def _dst_cb(self, m): self.distance = float(m.data)

    def _goal_cb(self, m):
        self.goal = (m.pose.position.x, m.pose.position.y)
        self._goal_t = time.monotonic()

    def _candidate_cb(self, m):
        self.candidate = (m.pose.position.x, m.pose.position.y)
        self._candidate_t = time.monotonic()

    def _cm_cb(self, m):
        d = m.data
        lethal = inflated = 0
        for v in d:
            if v >= 99:
                lethal += 1
            elif v > 0:
                inflated += 1
        self.cm_lethal, self.cm_inflated = lethal, inflated
        self._cm_t = time.monotonic()

    def _nav_cb(self, m): self.nav = m
    def _cmd_cb(self, m): self.cmd = m
    def _joy_cb(self, m): self._joy_t = time.monotonic()

    def _nav_enable_cb(self, m):
        self.nav_enable = bool(m.data)
        self._nav_enable_t = time.monotonic()

    @staticmethod
    def _load():
        try:
            with open('/proc/loadavg') as f:
                return float(f.read().split()[0])
        except Exception:  # noqa: BLE001
            return -1.0

    def _age(self, t):
        return '--' if t is None else f'{time.monotonic() - t:0.2f}'

    def _print_line(self):
        # Goal is only "live" if refreshed recently; else the robot isn't being
        # commanded by Nav2 right now.
        goal_str = '--'
        if self.goal is not None and self._goal_t and (time.monotonic() - self._goal_t) < 1.0:
            goal_str = f'({self.goal[0]:+.1f},{self.goal[1]:+.1f})'
        cand_str = '--'
        if (self.candidate is not None and self._candidate_t
                and (time.monotonic() - self._candidate_t) < 1.0):
            cand_str = f'({self.candidate[0]:+.1f},{self.candidate[1]:+.1f})'
        tgt = 'DET' if self.detected else 'none'
        # Treat a stale/never-received /nav_enable as "released", matching
        # drive_mux's own fail-safe staleness handling for this gate.
        if self._nav_enable_t is not None and (time.monotonic() - self._nav_enable_t) < 0.5:
            en_str = 'HELD' if self.nav_enable else 'released'
        else:
            en_str = 'released(stale)'
        line = (
            f'[{self.mode}] tgt={tgt} b={self.bearing:+.2f} d={self.distance:4.2f} | '
            f'goal={goal_str} cand={cand_str} | cm L={self.cm_lethal} I={self.cm_inflated} | '
            f'nav vx={self.nav.linear.x:+.2f} wz={self.nav.angular.z:+.2f} | '
            f'EN={en_str} | '
            f'CMD vx={self.cmd.linear.x:+.2f} wz={self.cmd.angular.z:+.2f} | '
            f'load={self._load():.1f} | age j={self._age(self._joy_t)} cm={self._age(self._cm_t)}'
        )
        # \r-style refresh in place; print newline every ~10s handled by terminal.
        print(line, flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = TestMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
