"""
Tests for Lab 6. The navigation contract and the fast surrogate environment.

By default these test the reference environment in `arc_rl/nav_core.py`. To
test your own work in `nav_core_skeleton.py`:

    ARC_NAV_CORE=nav_core_skeleton python3 -m pytest starters/lab06 -q

They cover the four blanked pieces and nothing else: reset, the pose
integration in step, the termination logic and the info dictionary. The
numbers are exact where the kinematics make them exact, because a step that
moves the robot almost the right distance is a bug you will not find by
watching a rollout.

The info keys are imported from arc_eval rather than copied, so this file
cannot drift away from what the evaluation harness actually demands.
"""

from __future__ import annotations

import importlib
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # nav_core_skeleton
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))   # arc_rl, arc_eval

from arc_eval.runner import REQUIRED_INFO_KEYS

V = importlib.import_module(os.environ.get("ARC_NAV_CORE", "arc_rl.nav_core"))

DT = V.ROBOT.control_period
FORWARD = np.array([1.0, 0.0], dtype=np.float32)     # full speed ahead
REVERSE = np.array([-1.0, 0.0], dtype=np.float32)    # limited reverse
SPIN = np.array([0.0, 1.0], dtype=np.float32)        # rotate in place
IDLE = np.zeros(2, dtype=np.float32)


def room(start=(1.0, 1.0, 0.0), goals=((8.5, 8.5),), max_steps=600):
    """An empty 10 m by 10 m room with four walls and no obstacles."""
    sc = V.Scenario.boxed_room(10.0, 10.0)
    sc.start_pose = start
    sc.goals = list(goals)
    sc.max_episode_steps = max_steps
    return sc


def make_env(scenario=None, reward="dense_progress", noise=0.0, randomise=False):
    return V.FastNavEnv(scenario=scenario or room(), reward=reward,
                        lidar_noise_std=noise, randomise_start=randomise)


# ---------------------------------------------------------------------------
# reset
# ---------------------------------------------------------------------------

def test_reset_returns_an_observation_of_the_contracted_shape_and_space():
    """A policy is built from observation_space, so an observation outside it is
    a silent contract break rather than an error."""
    env = make_env()
    obs, _ = env.reset(seed=0)
    assert obs.shape == (V.OBS.size,)
    assert obs.shape[0] == V.OBS.n_beams + 5
    assert obs.dtype == np.float32
    assert env.observation_space.contains(obs)


def test_reset_info_names_the_scenario_and_the_reward_function():
    env = make_env(room(), reward="sparse_safe")
    _, info = env.reset(seed=0)
    assert info["scenario"] == "boxed_room"
    assert info["reward_fn"] == "sparse_safe"


def test_reset_is_reproducible_from_its_seed():
    """The same seed must give the same episode, on a fresh environment."""
    a, _ = make_env(noise=0.01, randomise=True).reset(seed=7)
    b, _ = make_env(noise=0.01, randomise=True).reset(seed=7)
    c, _ = make_env(noise=0.01, randomise=True).reset(seed=8)
    assert np.array_equal(a, b)
    assert not np.array_equal(a, c)


def test_reset_never_spawns_the_robot_inside_an_obstacle():
    sc = room()
    sc.static_circles = [(x, y, 0.5) for x in (3.0, 5.0, 7.0) for y in (3.0, 5.0, 7.0)]
    env = V.FastNavEnv(scenario=sc, lidar_noise_std=0.0, randomise_start=True)
    for seed in range(8):
        env.reset(seed=seed)
        _, _, _, _, info = env.step(IDLE)
        assert info["collisions"] == 0, f"spawned in collision on seed {seed}"


def test_reset_primes_the_progress_baseline_for_the_first_step():
    """With the baseline left at zero the first dense reward reads as if the
    whole distance to the goal had just been lost."""
    env = make_env(room(start=(1.0, 1.0, 0.0), goals=[(9.0, 9.0)]))
    env.reset(seed=0)
    _, reward, _, _, _ = env.step(FORWARD)
    assert reward > -1.0


# ---------------------------------------------------------------------------
# pose integration in step
# ---------------------------------------------------------------------------

def test_step_integrates_a_known_action_along_the_current_heading():
    forward = V.ROBOT.max_linear_velocity * DT          # 0.025 m per step
    env = make_env(room(start=(5.0, 5.0, 0.0)))
    env.reset(seed=0)
    env.step(FORWARD)
    assert (env.x, env.y) == pytest.approx((5.0 + forward, 5.0), abs=1e-9)
    assert env.theta == pytest.approx(0.0, abs=1e-9)
    assert env.path_length == pytest.approx(forward, abs=1e-9)

    north = make_env(room(start=(5.0, 5.0, math.pi / 2)))
    north.reset(seed=0)
    north.step(FORWARD)
    assert (north.x, north.y) == pytest.approx((5.0, 5.0 + forward), abs=1e-9)

    # Reverse is limited to a quarter of the forward speed, and still adds to
    # the distance travelled.
    back = abs(V.ROBOT.min_linear_velocity) * DT
    rev = make_env(room(start=(5.0, 5.0, 0.0)))
    rev.reset(seed=0)
    rev.step(REVERSE)
    assert rev.x == pytest.approx(5.0 - back, abs=1e-9)
    assert rev.path_length == pytest.approx(back, abs=1e-9)


def test_step_rotates_in_place_and_wraps_the_heading():
    turn = V.ROBOT.max_angular_velocity * DT            # 0.09 rad per step
    env = make_env(room(start=(5.0, 5.0, 3.0)))
    env.reset(seed=0)
    for _ in range(2):
        env.step(SPIN)
    assert (env.x, env.y) == pytest.approx((5.0, 5.0), abs=1e-9)
    assert env.theta == pytest.approx(3.0 + 2 * turn - 2 * math.pi, abs=1e-6)
    assert -math.pi <= env.theta <= math.pi


# ---------------------------------------------------------------------------
# termination logic
# ---------------------------------------------------------------------------

def test_episode_terminates_on_collision():
    env = make_env(room(start=(0.3, 5.0, math.pi)))     # 0.08 m of clearance, facing the wall
    env.reset(seed=0)
    for _ in range(10):
        _, _, terminated, truncated, info = env.step(FORWARD)
        if terminated:
            break
    assert terminated and not truncated
    assert info["termination_reason"] == "collision"
    assert info["success"] is False
    assert info["collisions"] >= 1


def test_episode_terminates_on_reaching_the_goal():
    env = make_env(room(start=(5.0, 5.0, 0.0), goals=[(5.1, 5.0)]))
    env.reset(seed=0)
    _, _, terminated, truncated, info = env.step(FORWARD)
    assert terminated and not truncated
    assert info["termination_reason"] == "goal_reached"
    assert info["success"] is True
    assert info["checkpoints_reached"] == 1


def test_episode_truncates_at_the_step_limit_without_terminating():
    """Timeout is truncation, not termination. Gymnasium bootstraps through the
    one and not the other."""
    env = make_env(room(start=(1.0, 1.0, 0.0), goals=[(9.0, 9.0)], max_steps=5))
    env.reset(seed=0)
    for _ in range(5):
        _, _, terminated, truncated, info = env.step(FORWARD)
    assert truncated and not terminated
    assert info["termination_reason"] == "timeout"
    assert info["success"] is False


def test_a_step_that_ends_nothing_reports_running():
    env = make_env(room(start=(5.0, 5.0, 0.0), goals=[(9.0, 9.0)]))
    env.reset(seed=0)
    _, _, terminated, truncated, info = env.step(FORWARD)
    assert not terminated and not truncated
    assert info["termination_reason"] == "running"


# ---------------------------------------------------------------------------
# the info dictionary
# ---------------------------------------------------------------------------

def test_info_carries_every_key_the_evaluation_harness_requires():
    env = make_env()
    env.reset(seed=0)
    _, _, _, _, info = env.step(FORWARD)
    missing = [k for k in REQUIRED_INFO_KEYS if k not in info]
    assert missing == []
    # Plain Python scalars, because the runner writes info straight to JSON.
    assert isinstance(info["success"], bool)
    assert isinstance(info["collisions"], int)
    assert isinstance(info["checkpoints_reached"], int)
    assert isinstance(info["recovery_events"], int)
    assert isinstance(info["interventions"], int)
    assert isinstance(info["termination_reason"], str)
    for key in ("min_clearance_m", "path_length_m", "time_s"):
        assert isinstance(info[key], float)


def test_info_reports_simulated_time_and_distance_not_wall_clock():
    env = make_env(room(start=(1.0, 1.0, 0.0), goals=[(9.0, 9.0)]))
    env.reset(seed=0)
    for _ in range(10):
        _, _, _, _, info = env.step(FORWARD)
    assert info["time_s"] == pytest.approx(10 * DT)
    assert info["path_length_m"] == pytest.approx(10 * V.ROBOT.max_linear_velocity * DT)
    assert info["min_clearance_m"] == pytest.approx(1.0 - V.ROBOT.footprint_radius, abs=0.3)


def test_info_counts_a_checkpoint_passed_on_the_way_to_the_final_goal():
    """Passing an intermediate goal advances the count without ending the run."""
    env = make_env(room(start=(5.0, 5.0, 0.0), goals=[(5.1, 5.0), (8.0, 5.0)]))
    env.reset(seed=0)
    _, _, terminated, _, info = env.step(FORWARD)
    assert not terminated
    assert info["checkpoints_reached"] == 1
    assert info["success"] is False
