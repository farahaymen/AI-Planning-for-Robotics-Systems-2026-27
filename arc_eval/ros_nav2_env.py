"""
Autonomous Robotics Course (ARC) - Nav2 evaluation adapter.

This is what lets `arc_eval.runner` benchmark a Nav2 configuration using exactly
the same code path it uses for a trained policy. Nothing in the runner knows
whether it is driving a neural network or a behaviour tree.

The shape is slightly unusual and worth understanding rather than copying. Nav2
navigation is one long action, not a per-step decision, so `step()` sends the
goal on its first call and afterwards simply spins the executor and polls for
completion while accumulating metrics. The policy passed to the runner is a stub
that ignores its observation and returns None.

STATUS: experimental and excluded from the nine-lab execution path.
No validated reset or physical contact metric is implemented here. Use the
Lab 5 action client and Lab 9 single-episode ROS recorder for course experiments.
"""

from __future__ import annotations

import math
import time
from typing import Optional

import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan

from arc_rl.nav_core import ROBOT

# LaserScan is published best effort with a small queue. Subscribing with the
# default reliable QoS silently receives nothing, which is the single most
# common ROS 2 mistake in this course and the reason Lab 1 spends time on it.
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class Nav2EvalEnv:
    """Runs one navigation mission and reports the standard ARC metrics."""

    def __init__(
        self,
        goals: list[tuple[float, float, float]],
        system_label: str = "nav2",
        time_limit_s: float = 120.0,
        reset_service: Optional[str] = None,
        node_name: str = "arc_nav2_eval",
    ):
        if not rclpy.ok():
            rclpy.init()
        self.node = Node(node_name)
        self.goals = goals
        self.system_label = system_label
        self.time_limit_s = time_limit_s
        self.reset_service = reset_service

        self._client = ActionClient(self.node, NavigateToPose, "navigate_to_pose")
        self.node.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.node.create_subscription(LaserScan, "/scan", self._on_scan, SENSOR_QOS)

        self._reset_metrics()

    # -- callbacks ----------------------------------------------------------

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        if self._last_xy is not None:
            self.path_length += math.hypot(p.x - self._last_xy[0], p.y - self._last_xy[1])
        self._last_xy = (p.x, p.y)

    def _on_scan(self, msg: LaserScan) -> None:
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        finite = ranges[np.isfinite(ranges) & (ranges >= msg.range_min)]
        if finite.size == 0:
            return
        clearance = float(np.min(finite)) - ROBOT.footprint_radius
        self.min_clearance = min(self.min_clearance, clearance)
        # A contact is counted once per continuous contact episode, otherwise a
        # single scrape against a wall inflates the count by the scan rate.
        if clearance <= 0.0:
            if not self._in_contact:
                self.collisions += 1
                self._in_contact = True
        else:
            self._in_contact = False

    def _on_feedback(self, msg) -> None:
        self.recovery_events = int(msg.feedback.number_of_recoveries)

    # -- lifecycle ----------------------------------------------------------

    def _reset_metrics(self) -> None:
        self.path_length = 0.0
        self.collisions = 0
        self.min_clearance = float("inf")
        self.recovery_events = 0
        self.goal_index = 0
        self._last_xy = None
        self._in_contact = False
        self._goal_handle = None
        self._result_future = None
        self._started_at = None
        self._terminated = False
        self._success = False
        self._reason = "running"

    def _spin(self, seconds: float = 0.05) -> None:
        rclpy.spin_once(self.node, timeout_sec=seconds)

    def _reset_world(self, seed: int) -> None:
        """Return the simulation to a known state.

        Deliberately left as an explicit hook rather than hidden inside the
        launch file. Determinism in the simulator is the whole basis of the
        seeded comparison, so it should be visible in the code that claims it.
        """
        if self.reset_service is None:
            raise NotImplementedError("No validated reset. Use the Lab 9 single-episode recorder.")
        # Implemented in the Lab 6 starter package against ros_gz_interfaces.
        raise NotImplementedError(
            "Wire _reset_world to the ros_gz world control service before use. "
            "This adapter is outside the nine-lab execution path."
        )

    def _send_goal(self) -> None:
        x, y, yaw = self.goals[self.goal_index]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        if not self._client.wait_for_server(timeout_sec=20.0):
            raise RuntimeError(
                "navigate_to_pose action server not available. Check that the "
                "Nav2 lifecycle nodes are active: ros2 lifecycle get /bt_navigator"
            )
        send_future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=20.0)
        self._goal_handle = send_future.result()
        if self._goal_handle is None or not self._goal_handle.accepted:
            raise RuntimeError("Nav2 rejected the goal. Is it inside the map bounds?")
        self._result_future = self._goal_handle.get_result_async()

    # -- runner interface ---------------------------------------------------

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        self._reset_metrics()
        if seed is not None and self.reset_service is not None:
            self._reset_world(seed)
        deadline = time.time() + 10.0
        while self._last_xy is None and time.time() < deadline:
            self._spin()
        self._started_at = time.time()
        return np.zeros(1, dtype=np.float32), {"system": self.system_label}

    def step(self, _action):
        if self._result_future is None and not self._terminated:
            self._send_goal()

        self._spin()
        elapsed = time.time() - self._started_at

        if self._result_future is not None and self._result_future.done():
            status = self._result_future.result().status
            succeeded = status == 4  # GoalStatus.STATUS_SUCCEEDED
            if succeeded and self.goal_index < len(self.goals) - 1:
                self.goal_index += 1
                self._result_future = None
            else:
                self._terminated = True
                self._success = succeeded
                self._reason = "goal_reached" if succeeded else "nav2_aborted"

        if not self._terminated and elapsed > self.time_limit_s:
            self._terminated = True
            self._reason = "timeout"
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()

        info = {
            "success": bool(self._success),
            "collisions": int(self.collisions),
            "min_clearance_m": float(self.min_clearance),
            "path_length_m": float(self.path_length),
            "time_s": float(elapsed),
            "checkpoints_reached": int(self.goal_index + (1 if self._success else 0)),
            "recovery_events": int(self.recovery_events),
            "interventions": 0,
            "termination_reason": self._reason,
        }
        return np.zeros(1, dtype=np.float32), 0.0, self._terminated, False, info

    def close(self) -> None:
        self.node.destroy_node()


def make_nav2_policy():
    """Stub policy. Nav2 decides; the harness only needs something callable."""
    return lambda _obs: None


def make_nav2_env(goals, **kwargs):
    return Nav2EvalEnv(goals=[tuple(g) for g in goals], **kwargs)
