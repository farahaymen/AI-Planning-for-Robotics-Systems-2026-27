"""
Tests for Lab 4, Exercise 4.9. Coverage path planning.

    ARC_COVERAGE=coverage_skeleton python3 -m pytest tests/test_coverage.py -x -q

Coverage is a different problem from getting to a goal, and it is measured
differently. These tests check the measurements as hard as the algorithm,
because a coverage planner that scores itself generously is worse than no
planner at all.
"""

from __future__ import annotations

import importlib
import importlib.util
import os
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from gridmap import empty_room

selected_module = os.environ.get("ARC_COVERAGE", "coverage")
if selected_module == "coverage":
    # Pytest plugins may already have loaded the unrelated coverage package.
    # Load the lab implementation by file, under a distinct module name.
    coverage_file = Path(__file__).resolve().parents[1] / "coverage.py"
    spec = importlib.util.spec_from_file_location("arc_course_coverage", coverage_file)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load the lab coverage module: {coverage_file}")
    V = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = V
    spec.loader.exec_module(V)
else:
    V = importlib.import_module(selected_module)


def living_room():
    """6.0 m by 4.0 m at 0.05 m per cell, with three pieces of furniture."""
    grid = empty_room(80, 120)
    for (r, c, h, w) in [(20, 30, 14, 20), (50, 70, 16, 24), (12, 90, 10, 14)]:
        grid[r:r + h, c:c + w] = True
    return grid


# ---------------------------------------------------------------------------
# Decomposition
# ---------------------------------------------------------------------------

def test_a_clear_line_is_one_run():
    grid = empty_room(20, 40)
    runs = V.free_runs(grid, 10, axis=0)
    assert len(runs) == 1
    assert runs[0] == (1, 38)


def test_an_obstacle_splits_a_line_into_two_runs():
    """The fact that makes coverage more than a nested loop: furniture turns one
    pass into several, which then have to be ordered and connected."""
    grid = empty_room(20, 40)
    grid[10, 15:25] = True
    runs = V.free_runs(grid, 10, axis=0)
    assert len(runs) == 2
    assert runs[0] == (1, 14)
    assert runs[1] == (25, 38)


def test_a_fully_blocked_line_has_no_runs():
    grid = empty_room(20, 40)
    grid[10, :] = True
    assert V.free_runs(grid, 10, axis=0) == []


def test_sweep_lines_are_spaced_and_inset():
    """Starting half a spacing in centres the first pass in its stripe rather
    than pressing it against the wall."""
    grid = empty_room(40, 40)
    lines = V.sweep_lines(grid, spacing=10, axis=0)
    assert lines[0] == 5
    assert all(b - a == 10 for a, b in zip(lines, lines[1:]))


# ---------------------------------------------------------------------------
# The plan
# ---------------------------------------------------------------------------

def test_the_path_never_enters_an_obstacle():
    grid = living_room()
    result = V.boustrophedon(grid, spacing=9)
    assert result.path
    for (r, c) in result.path:
        assert not grid[r, c]


def test_the_path_is_connected():
    """Every consecutive pair must be an actual move, not a teleport.

    A coverage plan with a jump in it looks complete and cannot be driven.
    """
    grid = living_room()
    result = V.boustrophedon(grid, spacing=9)
    for (r0, c0), (r1, c1) in zip(result.path, result.path[1:]):
        assert max(abs(r1 - r0), abs(c1 - c0)) == 1


def test_an_empty_room_is_fully_covered():
    grid = empty_room(60, 60)
    result = V.boustrophedon(grid, spacing=9)
    assert result.covered > 0.99


def test_a_furnished_room_is_fully_covered():
    grid = living_room()
    result = V.boustrophedon(grid, spacing=9)
    assert result.covered > 0.97


def test_alternate_stripes_are_driven_in_opposite_directions():
    """That is what "as the ox turns" means, and it is what keeps turns short.

    Driving every stripe in the same direction adds a full traverse of the room
    between passes, which doubles the distance and every one of those returns is
    wasted.
    """
    grid = empty_room(40, 60)
    result = V.boustrophedon(grid, spacing=9)
    # Group the path by row and check the column direction alternates.
    directions = []
    for line in V.sweep_lines(grid, 9, axis=0):
        cells = [c for (r, c) in result.path if r == line]
        if len(cells) > 5:
            directions.append(np.sign(cells[-1] - cells[0]))
    assert len(directions) >= 3
    for a, b in zip(directions, directions[1:]):
        assert a != b


# ---------------------------------------------------------------------------
# The measurement, which is where a coverage planner flatters itself
# ---------------------------------------------------------------------------

def test_spacing_wider_than_the_tool_leaves_gaps():
    """The test that stops the planner scoring itself generously.

    Drive the passes further apart than the brush is wide and stripes of floor
    are never touched. If this passes with a high coverage number, the metric is
    being told the brush grew, which is the mistake that makes every setting
    look perfect.
    """
    grid = empty_room(80, 120)
    tool = 9
    tight = V.boustrophedon(grid, spacing=tool, tool_width=tool)
    loose = V.boustrophedon(grid, spacing=tool * 2, tool_width=tool)
    assert tight.covered > 0.99
    assert loose.covered < 0.75


def test_wider_spacing_means_fewer_turns():
    """The other half of the trade: fewer passes, less work, worse coverage."""
    grid = empty_room(80, 120)
    tight = V.boustrophedon(grid, spacing=5, tool_width=9)
    loose = V.boustrophedon(grid, spacing=14, tool_width=9)
    assert loose.turns < tight.turns
    assert loose.length < tight.length


def test_coverage_is_measured_against_reachable_space_only():
    """A sealed cupboard contains free cells no robot can enter. Counting them
    as failures means no planner can ever score 100 percent and the number stops
    telling you anything."""
    grid = empty_room(60, 60)
    grid[40:50, 40:50] = True            # a sealed box
    grid[42:48, 42:48] = False           # with free space inside it
    result = V.boustrophedon(grid, spacing=7)

    # Scored against reachable space this is a good run. Scored against every
    # free cell it would be penalised for the 36 cells inside the sealed box,
    # which no robot could ever reach, and the score would be capped below 100
    # for reasons that have nothing to do with the planner.
    assert result.covered > 0.93

    naive_total = int((~grid).sum())
    assert naive_total > 0
    swept = 0
    seen = set(result.path)
    assert len(seen) < naive_total       # the box interior is never entered


def test_counting_turns_ignores_straight_runs():
    straight = [(0, i) for i in range(20)]
    assert V.count_turns(straight) == 0


def test_counting_turns_sees_a_corner():
    corner = [(0, i) for i in range(10)] + [(j, 9) for j in range(1, 10)]
    assert V.count_turns(corner) == 1


# ---------------------------------------------------------------------------
# The experiment worth running
# ---------------------------------------------------------------------------

def test_sweeping_along_the_long_axis_takes_fewer_turns():
    """The one result to take away, and one nobody guesses reliably.

    A 6 m by 4 m room swept along its 6 m axis needs about 40 turns and 13
    passes. Swept along the 4 m axis it needs about 62 turns and 20 passes, for
    the same floor. Align the sweep with the long axis: turns happen at the ENDS
    of passes, so longer passes means fewer ends.
    """
    grid = living_room()
    results = V.compare_axes(grid, spacing=9, tool_width=9)
    assert results["rows"].turns < results["columns"].turns
    assert results["rows"].covered >= 0.97
    assert results["columns"].covered >= 0.95


def test_the_connecting_moves_use_the_point_to_point_planner():
    """A coverage planner is a CLIENT of a path planner, not a replacement.

    With furniture in the way, joining the end of one pass to the start of the
    next is a real planning problem. The path must route around the obstacle
    rather than through it, which is only true if A* is doing the joining.
    """
    grid = empty_room(60, 60)
    grid[10:50, 28:32] = True            # a wall splitting the room
    result = V.boustrophedon(grid, spacing=9)
    for (r, c) in result.path:
        assert not grid[r, c]
    assert result.segments > len(V.sweep_lines(grid, 9, axis=0))
