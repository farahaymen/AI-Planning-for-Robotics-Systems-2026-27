"""Use a saved Lab 6 Q-table in the matching Gazebo world. No training here."""

import math
from pathlib import Path
from time import monotonic

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from teaching.tabular_motion import DIRECTIONS, GridRun


class GridPolicyDriver(Node):
    def __init__(self):
        super().__init__('grid_policy_driver')
        self.declare_parameter('q_table', '')
        self.result = 0
        self.stop_at = None
        self.pose = None
        self.odom_at = -math.inf
        self.started_at = monotonic()
        self.move_at = None
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        try:
            path = Path(self.get_parameter('q_table').value).expanduser()
            if path.suffix != '.npy':
                raise ValueError('pass the saved q.npy from Q-learning')
            self.run = GridRun(np.load(path, allow_pickle=False))
        except (OSError, ValueError, EOFError) as exc:
            self.finish(1, f'Cannot load Q-table: {exc}')
            return
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_timer(.05, self.tick, clock=self.steady_clock)
        self.get_logger().info(f'Loaded {path}. Waiting for Gazebo odometry and command relay.')

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
        if msg.header.frame_id == 'odom' and all(map(math.isfinite, (p.x, p.y, yaw))):
            self.pose = p.x, p.y, yaw
            self.odom_at = monotonic()

    def finish(self, result, message):
        self.result = result
        self.stop_at = monotonic() + .4
        log = self.get_logger()
        (log.error if result else log.info)(message)
        self.publisher.publish(Twist())

    def tick(self):
        now = monotonic()
        if self.stop_at is not None:
            self.publisher.publish(Twist())
            return
        odom_fresh = now - self.odom_at < 1.
        relay_ready = self.publisher.get_subscription_count() > 0
        if not self.run.moves:
            if not (odom_fresh and relay_ready):
                if now - self.started_at > 90.:
                    self.finish(1, 'No fresh /odom or no /cmd_vel subscriber. Check the Gazebo launch.')
                return
            if math.hypot(*self.pose[:2]) > .12:
                self.finish(1, 'Robot is away from the start. Relaunch the Lab 6 world to reset it.')
                return
        elif not (odom_fresh and relay_ready):
            self.finish(1, 'Lost /odom or the /cmd_vel subscriber. Robot stopped.')
            return
        if self.run.target is not None and now - self.move_at > 50.:
            self.finish(1, f'Move to {self.run.target} timed out. Check Gazebo and the controllers.')
            return

        last_moves = self.run.moves
        try:
            forward, turn = self.run.command(*self.pose)
        except RuntimeError as exc:
            self.finish(1, str(exc))
            return
        if self.run.moves != last_moves:
            self.move_at = now
            direction = DIRECTIONS[self.run.action]
            self.get_logger().info(f'Grid {self.run.cell}: {direction} -> {self.run.target}')
        if self.run.done:
            self.finish(0, f'Goal reached at {self.run.cell} in {self.run.moves} moves.')
            return
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = forward, turn
        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GridPolicyDriver()
    try:
        while rclpy.ok() and (node.stop_at is None or monotonic() < node.stop_at):
            rclpy.spin_once(node, timeout_sec=.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()
    return node.result
