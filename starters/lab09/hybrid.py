"""
Lab 9: a hybrid controller that switches between classical and learned.

The classical controller is the default. A handwritten rule requests learned
control when recent displacement is small and the front sector is tight.
Lab 9 measures whether that decision helps; neither controller is assumed
to be safe or superior just because of its implementation style.

The switching criterion is geometric and hand written rather than learned. That
is a deliberate design decision, not a shortcut. A learned switch inherits the
generalisation problems of the thing it is supposed to guard, so when the RL
policy fails on an unfamiliar layout the switch is likely to fail there too.

Both controllers consume the observation defined in arc_rl.nav_core. The
teaching.rollout helper supplies current simulator position to the switch and
constructs a fresh controller per episode. That is not a measured ROS transfer.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from arc_rl.nav_core import OBS, ROBOT

_ANGLES = np.linspace(-math.pi, math.pi, OBS.n_beams, endpoint=False)
_FRONT = np.abs(_ANGLES) < 0.6


@dataclass
class HybridConfig:
    stall_window: int = 40              # control steps, 2 s at 20 Hz
    stall_displacement: float = 0.12    # m
    tight_clearance: float = 0.55       # m ahead before the space counts as tight
    handover_steps: int = 60            # commit to the learned policy for 3 s
    max_handovers: int = 12             # safety cap per episode


@dataclass
class HybridStats:
    handovers: int = 0
    learned_steps: int = 0
    classical_steps: int = 0

    @property
    def learned_fraction(self) -> float:
        total = self.learned_steps + self.classical_steps
        return self.learned_steps / total if total else 0.0


class HybridController:
    """Classical by default, learned when the classical controller is stuck.

    The handover is committed for a fixed number of steps rather than being
    re-decided every tick. Without that commitment the controller oscillates
    between the two policies at the boundary condition and behaves worse than
    either alone, which is a failure mode worth seeing once.
    """

    def __init__(self, classical_policy, learned_policy,
                 config: HybridConfig = HybridConfig()):
        self.classical = classical_policy
        self.learned = learned_policy
        self.cfg = config
        self.reset()

    def reset(self) -> None:
        self.positions: list[tuple[float, float]] = []
        self.remaining_handover = 0
        self.stats = HybridStats()

    # -- the switching criterion --------------------------------------------

    def _stalled(self) -> bool:
        if len(self.positions) < self.cfg.stall_window:
            return False
        ox, oy = self.positions[-self.cfg.stall_window]
        x, y = self.positions[-1]
        return math.hypot(x - ox, y - oy) < self.cfg.stall_displacement

    @staticmethod
    def _front_clearance(obs: np.ndarray) -> float:
        beams = obs[:OBS.n_beams] * (OBS.lidar_max_range - OBS.lidar_min_range) \
            + OBS.lidar_min_range
        return float(np.min(beams[_FRONT]))

    def should_hand_over(self, obs: np.ndarray) -> bool:
        if self.stats.handovers >= self.cfg.max_handovers:
            return False
        return self._stalled() and self._front_clearance(obs) < self.cfg.tight_clearance

    # -- the controller ------------------------------------------------------

    def __call__(self, obs: np.ndarray, position: tuple[float, float] | None = None):
        if position is not None:
            self.positions.append(position)

        if self.remaining_handover > 0:
            self.remaining_handover -= 1
            self.stats.learned_steps += 1
            return self.learned(obs)

        if self.should_hand_over(obs):
            self.remaining_handover = self.cfg.handover_steps - 1
            self.stats.handovers += 1
            self.stats.learned_steps += 1
            self.positions.clear()      # do not re-trigger on the same history
            return self.learned(obs)

        self.stats.classical_steps += 1
        return self.classical(obs)


def make_hybrid_policy(classical_policy, learned_policy,
                       config: HybridConfig = HybridConfig()):
    """Adapt the hybrid controller to the evaluation harness's policy signature.

    The harness passes only the observation, so position is recovered from the
    goal-relative part of the observation vector rather than being threaded
    through. It is a proxy rather than the true pose, which is enough for a
    displacement test and keeps the harness interface unchanged.
    """
    controller = HybridController(classical_policy, learned_policy, config)
    n = OBS.n_beams

    def policy(obs):
        distance = float(obs[n]) * OBS.max_goal_distance
        angle = math.atan2(float(obs[n + 1]), float(obs[n + 2]))
        # Position of the goal relative to the robot, used purely as a
        # displacement proxy: if this is not changing, the robot is not moving.
        proxy = (distance * math.cos(angle), distance * math.sin(angle))
        return controller(obs, proxy)

    policy.controller = controller
    return policy
