"""Non-learned baselines. Students benchmark learned policies against these."""
from __future__ import annotations

import math
import numpy as np

from arc_rl.nav_core import OBS, ROBOT

_ANGLES = np.linspace(-math.pi, math.pi, OBS.n_beams, endpoint=False)


def _denormalise_beams(obs: np.ndarray) -> np.ndarray:
    n = OBS.n_beams
    return obs[:n] * (OBS.lidar_max_range - OBS.lidar_min_range) + OBS.lidar_min_range


def make_gap_follow_policy(goal_weight: float = 1.6, clearance_weight: float = 1.0,
                           safety_distance: float = 0.55, turn_gain: float = 1.5):
    """Follow the gap: choose the heading that trades clearance against goal alignment.

    A naive potential field fails on a head-on obstacle because the lateral
    repulsion is symmetric and cancels, so the robot drives into the object at
    full speed. Scoring candidate headings directly avoids that failure without
    adding state, which makes this a fair classical floor for the Lab 9 and
    Lab 10 comparisons.
    """
    n = OBS.n_beams
    front = np.abs(_ANGLES) < (math.pi / 2)

    def policy(obs):
        beams = _denormalise_beams(obs)
        sin_g, cos_g = float(obs[n + 1]), float(obs[n + 2])
        goal_angle = math.atan2(sin_g, cos_g)

        # Clearance available along each candidate heading, blurred over
        # neighbours so a single noisy beam cannot create a phantom gap.
        blurred = np.minimum.reduce([np.roll(beams, k) for k in (-1, 0, 1)])
        alignment = np.cos(_ANGLES - goal_angle)
        score = clearance_weight * np.clip(blurred / OBS.lidar_max_range, 0.0, 1.0) \
            + goal_weight * alignment
        score = np.where(front & (blurred > safety_distance), score, -np.inf)

        if not np.isfinite(score).any():
            # Boxed in. Rotate towards the side with more room and back off.
            left = float(np.mean(beams[(_ANGLES > 0.2) & (_ANGLES < 1.6)]))
            right = float(np.mean(beams[(_ANGLES < -0.2) & (_ANGLES > -1.6)]))
            return np.array([-0.3, 1.0 if left > right else -1.0], dtype=np.float32)

        heading = float(_ANGLES[int(np.argmax(score))])
        w = turn_gain * heading
        ahead = float(np.min(blurred[np.abs(_ANGLES) < 0.4]))
        v = np.clip((ahead - 0.3) / 1.0, 0.15, 1.0)
        v *= max(0.25, 1.0 - abs(heading) / (math.pi / 2))
        return np.array([v, np.clip(w / ROBOT.max_angular_velocity, -1.0, 1.0)],
                        dtype=np.float32)

    return policy


def make_random_policy(seed: int = 0):
    rng = np.random.default_rng(seed)
    return lambda obs: rng.uniform(-1.0, 1.0, 2).astype(np.float32)
