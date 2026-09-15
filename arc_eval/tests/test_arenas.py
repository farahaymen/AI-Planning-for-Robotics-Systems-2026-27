import sys; sys.path.insert(0, ".")
import numpy as np, pytest
from arc_rl.arenas import (DEFAULT_GRAMMAR, TRAINING_GRAMMAR, bfs_reachable,
                           competition_scenario, rasterise, training_scenario)
from arc_rl.nav_core import ROBOT


@pytest.mark.parametrize("seed", [1001, 1002, 1003, 1004, 1005, 1006])
def test_every_checkpoint_is_reachable_with_real_clearance(seed):
    """Not just reachable by a point robot: reachable by one with a body."""
    sc = competition_scenario(seed)
    grid = rasterise(sc, DEFAULT_GRAMMAR.grid_resolution,
                     ROBOT.footprint_radius + DEFAULT_GRAMMAR.clearance_margin)
    to_cell = lambda x, y: (int(x / DEFAULT_GRAMMAR.grid_resolution),
                            int(y / DEFAULT_GRAMMAR.grid_resolution))
    start = to_cell(sc.start_pose[0], sc.start_pose[1])
    for gx, gy in sc.goals:
        assert bfs_reachable(grid, start, to_cell(gx, gy)), f"goal ({gx:.1f},{gy:.1f}) unreachable"


@pytest.mark.parametrize("seed", [1001, 1002, 1003])
def test_a_seed_reproduces_the_same_arena_exactly(seed):
    a, b = competition_scenario(seed), competition_scenario(seed)
    assert a.bounds == b.bounds and a.goals == b.goals and a.walls == b.walls


def test_different_seeds_give_different_arenas():
    assert competition_scenario(1001).goals != competition_scenario(1002).goals


def test_the_grammar_bounds_are_respected():
    for seed in range(1001, 1011):
        sc = competition_scenario(seed)
        w, h = sc.bounds
        assert DEFAULT_GRAMMAR.width_range[0] <= w <= DEFAULT_GRAMMAR.width_range[1]
        assert DEFAULT_GRAMMAR.height_range[0] <= h <= DEFAULT_GRAMMAR.height_range[1]
        assert len(sc.goals) == DEFAULT_GRAMMAR.n_goals
        assert (DEFAULT_GRAMMAR.n_static[0] <= len(sc.static_circles)
                <= DEFAULT_GRAMMAR.n_static[1])


def test_checkpoints_respect_the_minimum_separation():
    sc = competition_scenario(1003)
    points = [sc.start_pose[:2]] + list(sc.goals)
    for i, a in enumerate(points):
        for b in points[i + 1:]:
            assert np.hypot(a[0] - b[0], a[1] - b[1]) >= \
                DEFAULT_GRAMMAR.min_goal_separation - 0.15


def test_the_training_grammar_is_easier_than_the_competition_grammar():
    train, comp = training_scenario(7), competition_scenario(7)
    assert len(train.goals) < len(comp.goals)
    assert max(train.bounds) <= max(comp.bounds)
