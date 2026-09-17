"""
Lab 6 self-check. Run this before you submit.

The pure Python tests run anywhere including graphics Tier C. The ROS tests are
skipped automatically if rclpy is unavailable, so a failing environment does not
produce a wall of confusing errors.

    pytest starters/lab06/tests -v
"""

import importlib.util
import os
import sys
import types

import numpy as np
import pytest

# Evaluate this once, at import, without raising. Calling pytest.importorskip
# inside a decorator argument skips the WHOLE module, including the pure Python
# tests that have nothing to do with ROS.
HAS_ROS = importlib.util.find_spec("rclpy") is not None

sys.path.insert(0, "starters/lab06")

from grid_tools import (GridInfo, grid_to_world, inflate, occupancy_to_numpy,
                        path_length_m, to_binary_obstacle_map, world_to_grid)


def fake_occupancy(width=6, height=4, resolution=0.05, ox=-1.0, oy=-2.0):
    msg = types.SimpleNamespace(
        info=types.SimpleNamespace(
            resolution=resolution, width=width, height=height,
            origin=types.SimpleNamespace(
                position=types.SimpleNamespace(x=ox, y=oy))),
        data=[0] * (width * height))
    return msg


def test_grid_orientation_is_height_by_width():
    """The classic transposition bug. A 6x4 map must reshape to (4, 6)."""
    grid, info = occupancy_to_numpy(fake_occupancy())
    assert grid.shape == (4, 6)
    assert (info.width, info.height) == (6, 4)


def test_world_grid_roundtrip():
    _, info = occupancy_to_numpy(fake_occupancy())
    for cell in [(0, 0), (2, 3), (3, 5)]:
        assert world_to_grid(*grid_to_world(*cell, info), info) == cell


def test_unknown_handling_changes_the_obstacle_map():
    msg = fake_occupancy()
    msg.data[5] = -1
    grid, _ = occupancy_to_numpy(msg)
    assert to_binary_obstacle_map(grid, unknown_is_obstacle=True).sum() == 1
    assert to_binary_obstacle_map(grid, unknown_is_obstacle=False).sum() == 0


def test_inflation_grows_obstacles_by_a_disc():
    obstacles = np.zeros((9, 9), dtype=bool)
    obstacles[4, 4] = True
    assert inflate(obstacles, 0).sum() == 1
    assert inflate(obstacles, 1).sum() == 5      # centre plus four neighbours
    assert inflate(obstacles, 2).sum() == 13


def test_inflation_does_not_wrap_at_the_border():
    obstacles = np.zeros((5, 5), dtype=bool)
    obstacles[0, 0] = True
    assert not inflate(obstacles, 2)[4, 4]


def test_path_length_is_metric_not_cell_count():
    info = GridInfo(resolution=0.05, origin_x=0.0, origin_y=0.0, width=10, height=10)
    straight = path_length_m([(0, 0), (0, 1), (0, 2)], info)
    assert straight == pytest.approx(0.10, abs=1e-6)


# Needs a LIVE Nav2 stack, not merely a sourced ROS. rclpy being importable says
# nothing about whether navigate_to_pose is being served, and letting this run in
# the offline suite costs a 40 second timeout before failing for the wrong reason.
#     ARC_INTEGRATION=1 pytest starters/lab06/tests -v
@pytest.mark.skipif(not HAS_ROS or os.environ.get("ARC_INTEGRATION") != "1",
                    reason="needs a running Nav2 stack; set ARC_INTEGRATION=1")
def test_nav2_adapter_reports_every_required_key():
    from arc_eval.runner import REQUIRED_INFO_KEYS
    from arc_eval.ros_nav2_env import Nav2EvalEnv
    env = Nav2EvalEnv(goals=[(1.0, 1.0, 0.0)])
    try:
        env.reset(seed=0)
        _, _, _, _, info = env.step(None)
        missing = [k for k in REQUIRED_INFO_KEYS if k not in info]
        assert not missing, f"adapter does not report {missing}"
    finally:
        env.close()
