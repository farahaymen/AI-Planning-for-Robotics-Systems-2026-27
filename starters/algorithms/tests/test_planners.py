"""
Tests for Lab 4, Exercise 4.2.

These are the specification for the planners. Read them before you write any
code; they say what each algorithm has to do, including the cases that are easy
to get wrong and hard to notice.

By default they test the reference implementation in `planners.py`. To test your
own work in `planners_skeleton.py`:

    ARC_PLANNERS=planners_skeleton python3 -m pytest tests/ -x -q

Run from the lab04_planning directory. `-x` stops at the first failure, which is
what you want while you are working through the TODOs in order.

Why several of these assert exact numbers. Three of the four algorithms are one
or two lines apart from each other, so it is entirely possible to write
something that runs, returns a valid path, and is not the algorithm you meant.
A test that only checks "a path came back" would pass on all of those. Checking
expansion counts and path costs is what actually pins down which algorithm you
wrote.
"""

from __future__ import annotations

import importlib
import math
import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from gridmap import (GOAL, MAPS, MOVES_4, MOVES_8, SOLVABLE, START,
                     is_valid_path, path_length)

P = importlib.import_module(os.environ.get("ARC_PLANNERS", "planners"))

SQRT2 = math.sqrt(2.0)


def run(label, grid, moves=MOVES_8):
    return P.PLANNERS[label](grid, START, GOAL, moves)


# ---------------------------------------------------------------------------
# The heuristics
# ---------------------------------------------------------------------------

def test_manhattan_is_the_sum_of_the_two_distances():
    assert P.manhattan((0, 0), (3, 4)) == 7
    assert P.manhattan((5, 5), (5, 5)) == 0


def test_octile_is_diagonals_then_straights():
    # Pure diagonal: three diagonal steps.
    assert P.octile((0, 0), (3, 3)) == pytest.approx(3 * SQRT2)
    # Pure straight: five straight steps, no diagonal helps.
    assert P.octile((0, 0), (0, 5)) == pytest.approx(5.0)
    # Mixed: two diagonals to square it up, then three straight.
    assert P.octile((0, 0), (2, 5)) == pytest.approx(2 * SQRT2 + 3.0)
    assert P.octile((4, 4), (4, 4)) == pytest.approx(0.0)


def test_octile_is_admissible_and_manhattan_is_not_on_eight_connected():
    """The single fact that makes the Manhattan heuristic wrong on this grid.

    Admissible means the estimate never exceeds the true remaining cost. On a
    diagonal grid the true cost from (0,0) to (3,3) is 3*sqrt(2) = 4.24, and
    Manhattan says 6. It overestimates, so A* using it is no longer guaranteed
    to return a shortest path.
    """
    true_cost = 3 * SQRT2
    assert P.octile((0, 0), (3, 3)) <= true_cost + 1e-9
    assert P.manhattan((0, 0), (3, 3)) > true_cost


def test_euclidean_never_exceeds_octile():
    """Both are admissible; Euclidean is the looser of the two.

    A looser admissible heuristic is still correct, it just expands more cells.
    That ordering is exactly what the expansion counts in the comparison table
    show: octile 182, euclidean 244 on the cluttered map.
    """
    for a, b in [((0, 0), (7, 3)), ((2, 9), (11, 4)), ((0, 0), (6, 6))]:
        assert P.euclidean(a, b) <= P.octile(a, b) + 1e-9


# ---------------------------------------------------------------------------
# Every planner, on every map
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("label", list(P.PLANNERS))
@pytest.mark.parametrize("map_name", SOLVABLE)
def test_every_planner_returns_a_walkable_path(label, map_name):
    """A path that teleports through a wall still draws a convincing picture."""
    grid = MAPS[map_name]()
    result = run(label, grid)
    assert result.found, f"{label} found no path on {map_name}, but one exists"
    assert is_valid_path(grid, result.path, START, GOAL), (
        f"{label} on {map_name} returned a path that is not walkable")


@pytest.mark.parametrize("label", list(P.PLANNERS))
def test_every_planner_terminates_and_reports_failure_on_sealed_goal(label):
    """No path exists. Report it; do not hang and do not invent one."""
    grid = MAPS["sealed_goal"]()
    result = run(label, grid)
    assert result.found is False
    assert result.path == []


@pytest.mark.parametrize("label", list(P.PLANNERS))
def test_a_path_never_cuts_a_wall_corner(label):
    """Diagonal moves between two blocked cells are forbidden.

    A point robot could slip through the corner where two walls meet. A robot
    with a 0.22 m footprint cannot, and the plan you hand Nav2 in Lab 6 has to
    be one the robot can actually drive.
    """
    grid = MAPS["cluttered"]()
    result = run(label, grid)
    for (r0, c0), (r1, c1) in zip(result.path, result.path[1:]):
        if r0 != r1 and c0 != c1:
            assert not (grid[r1, c0] and grid[r0, c1]), (
                f"{label} cut the corner at {(r0, c0)} -> {(r1, c1)}")


# ---------------------------------------------------------------------------
# Optimality
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("map_name", SOLVABLE)
def test_the_optimal_planners_all_agree(map_name):
    """Different algorithms, same shortest distance. This is the headline claim.

    They may return different routes; ties are everywhere on a grid. What they
    must not do is disagree on the cost.
    """
    grid = MAPS[map_name]()
    lengths = {label: run(label, grid).length for label in P.OPTIMAL_ON_8}
    best = min(lengths.values())
    for label, length in lengths.items():
        assert length == pytest.approx(best), (
            f"{label} returned {length:.2f} on {map_name}, optimum is {best:.2f}")


def test_astar_with_a_zero_heuristic_is_exactly_dijkstra():
    """Not approximately. The priority is literally the same expression.

    If these two differ you have a bug in one of them, and this test will tell
    you which before any of the interesting maps confuse you.
    """
    for map_name in SOLVABLE:
        grid = MAPS[map_name]()
        d = run("Dijkstra", grid)
        z = run("A* zero (= Dijkstra)", grid)
        assert z.expanded == d.expanded, f"on {map_name}"
        assert z.length == pytest.approx(d.length), f"on {map_name}"


def test_a_good_heuristic_expands_far_fewer_cells():
    """The reason to use A* at all.

    On an open map the octile estimate is exact, so A* walks almost straight to
    the goal while Dijkstra expands a disc covering the whole room.
    """
    grid = MAPS["empty_room"]()
    d = run("Dijkstra", grid)
    a = run("A* octile", grid)
    assert a.length == pytest.approx(d.length)
    assert a.expanded < d.expanded / 10


def test_a_heuristic_helps_less_when_walls_are_in_the_way():
    """And the reason A* is not a free lunch.

    On `corridors` the straight line to the goal points at a wall for most of
    the search, so the estimate is badly wrong almost everywhere and A* expands
    nearly as much as Dijkstra does.
    """
    grid = MAPS["corridors"]()
    d = run("Dijkstra", grid)
    a = run("A* octile", grid)
    assert a.expanded > d.expanded / 2


# ---------------------------------------------------------------------------
# The three failures the maps were built to show
# ---------------------------------------------------------------------------

def test_bfs_minimises_steps_and_not_distance():
    """BFS is optimal in the wrong currency once diagonals exist.

    On `steps_vs_cost` BFS returns a route of 30 steps measuring 39.94 and
    Dijkstra returns one of 32 steps measuring 39.46. BFS wins on the thing it
    optimises and loses on the thing you care about, because it counts a
    diagonal as one move when it costs sqrt(2).
    """
    grid = MAPS["steps_vs_cost"]()
    b = run("BFS", grid, MOVES_8)
    d = run("Dijkstra", grid, MOVES_8)
    assert b.steps < d.steps, "BFS should use fewer steps"
    assert b.length > d.length + 1e-9, "and cover more distance doing it"


def test_bfs_is_optimal_again_on_a_four_connected_grid():
    """Remove the diagonals and every move costs 1, so steps and cost coincide.

    This is the other half of the lesson. BFS is not a bad algorithm; it is an
    algorithm with a precondition, and the precondition is a uniform edge cost.
    """
    for map_name in SOLVABLE:
        grid = MAPS[map_name]()
        b = run("BFS", grid, MOVES_4)
        d = run("Dijkstra", grid, MOVES_4)
        assert b.length == pytest.approx(d.length), f"on {map_name}"


def test_an_inadmissible_heuristic_trades_optimality_for_speed():
    """Both halves of the trade, measured, on the map built to show it.

    It really is faster and it really does cost you. State the trade honestly
    in your write-up rather than calling one of them better.
    """
    grid = MAPS["greedy_trap"]()
    good = run("A* octile", grid)
    bad = run("A* inadmissible", grid)
    assert bad.expanded < good.expanded / 2, "should be much cheaper to run"
    assert bad.length > good.length + 1e-9, "and should return a longer path"


def test_greedy_best_first_drives_into_the_trap():
    """Drop the cost-so-far term from A* and this is what you get.

    Every cell inside the box is nearer the goal than the cells outside it, so
    a search ordered on the heuristic alone descends into it and has to climb
    back out. Plot the path on this map; the dive is visible in the line.
    """
    grid = MAPS["greedy_trap"]()
    good = run("A* octile", grid)
    greedy = run("Greedy best first", grid)
    assert greedy.found
    assert greedy.length > good.length * 1.2, (
        "greedy best first should be badly suboptimal here")


def test_dfs_is_valid_and_bad():
    """DFS is not wrong, it is unbounded. Both halves matter.

    On `corridors` it returns a path four times longer than necessary while
    still being a path the robot could drive. A planner can be perfectly correct
    and completely useless.
    """
    grid = MAPS["corridors"]()
    d = run("DFS", grid)
    best = run("Dijkstra", grid)
    assert is_valid_path(grid, d.path, START, GOAL)
    assert d.length > best.length * 2


def test_dfs_looking_good_on_an_empty_room_is_an_accident():
    """Worth knowing so you do not draw the wrong conclusion from one map.

    On `empty_room` DFS happens to find the optimal diagonal, because the first
    branch it follows happens to be the right one. Change the order of MOVES_8
    and the result changes. Never judge an algorithm on its best case.
    """
    grid = MAPS["empty_room"]()
    assert run("DFS", grid).length == pytest.approx(run("Dijkstra", grid).length)

    reordered = tuple(reversed(MOVES_8))
    shuffled = P.PLANNERS["DFS"](grid, START, GOAL, reordered)
    assert shuffled.length > run("Dijkstra", grid).length


# ---------------------------------------------------------------------------
# Bookkeeping, which is what makes the comparison table trustworthy
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("label", list(P.PLANNERS))
def test_the_counters_are_filled_in(label):
    """An empty metric column is a table that cannot be compared."""
    result = run(label, MAPS["cluttered"]())
    assert result.expanded > 0
    assert result.generated >= result.expanded
    assert result.max_frontier > 0
    assert result.runtime_ms > 0.0


def test_bfs_marks_visited_at_push_time():
    """Detects the classic BFS bug by its signature: a bloated frontier.

    Marking visited at pop time instead of push time lets the same cell enter
    the queue once per neighbour that finds it. The path still comes out right,
    which is why this is worth a test rather than an eyeball. On a 30x30 map it
    costs nothing; on a full costmap it is the difference between planning and
    not planning.
    """
    grid = MAPS["empty_room"]()
    b = run("BFS", grid, MOVES_8)
    free_cells = int((~grid).sum())
    assert b.max_frontier < free_cells, (
        "the BFS frontier grew past the number of free cells, which means the "
        "same cells are being queued repeatedly: mark visited when you push")


def test_path_length_counts_a_diagonal_as_root_two():
    assert path_length([(0, 0), (0, 1), (0, 2)]) == pytest.approx(2.0)
    assert path_length([(0, 0), (1, 1)]) == pytest.approx(SQRT2)
    assert path_length([(0, 0)]) == pytest.approx(0.0)


def test_is_valid_path_rejects_the_paths_it_should():
    grid = MAPS["cluttered"]()
    good = run("Dijkstra", grid).path
    assert is_valid_path(grid, good, START, GOAL)
    assert not is_valid_path(grid, [], START, GOAL)
    assert not is_valid_path(grid, good[:-1], START, GOAL)       # stops short
    assert not is_valid_path(grid, good[::2], START, GOAL)       # teleports
