"""
Lab 4, Exercise 4.9. Coverage path planning.

This is the file you edit.

    ARC_COVERAGE=coverage_skeleton python3 -m pytest tests/test_coverage.py -x -q

Every other planner in this folder answers "how do I get from A to B". This one
answers a different question: **how do I visit all of it**.

That is not a variant of the same problem, it is a different problem with a
different objective and a different metric, and it is the question a large part
of the robotics market actually asks. Vacuum cleaners, lawn mowers, floor
scrubbers, agricultural sprayers, inspection robots. None of them has a goal
pose.

**Turns are the cost, not distance.** On a straight run the machine moves at
full speed with the tool at rated throughput. Every turn is decelerate, rotate,
accelerate, and often disengage and re-engage the tool. Forty long passes finish
sooner than a hundred and twenty short ones even at the same total distance. A
coverage planner that optimises path length is optimising the wrong quantity.

**Boustrophedon** is Greek for "as the ox turns": up one furrow and back down
the next, the way a field is ploughed. It is the oldest path planning algorithm
there is and it is what your robot vacuum does tonight.

Nav2 ships a Coverage Server built on Fields2Cover, so once you have written
this you can go and configure the production version.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from gridmap import MOVES_8
from planners import astar, octile

Cell = tuple[int, int]


@dataclass
class CoverageResult:
    """One coverage run, with the numbers that matter. Given to you complete."""

    path: list[Cell] = field(default_factory=list)
    covered: float = 0.0
    turns: int = 0
    length: float = 0.0
    segments: int = 0

    def __str__(self) -> str:
        return (f"covered {self.covered * 100:5.1f}%  turns {self.turns:4d}  "
                f"length {self.length:7.1f}  segments {self.segments:3d}")


# ---------------------------------------------------------------------------
# Exercise 4.9a. Decomposition
# ---------------------------------------------------------------------------

def sweep_lines(grid: np.ndarray, spacing: int, axis: int = 0) -> list[int]:
    """The rows (axis 0) or columns (axis 1) the robot will drive along."""
    # TODO 1: return indices spaced `spacing` apart, starting at spacing // 2
    #         so the first pass is centred in its stripe rather than pressed
    #         against the wall. Stop before grid.shape[axis].
    raise NotImplementedError("TODO: implement sweep_lines")


def free_runs(grid: np.ndarray, index: int, axis: int = 0) -> list[tuple[int, int]]:
    """Maximal runs of free cells along one sweep line, as (start, end) pairs.

    An obstacle in the MIDDLE of a line splits it in two. This is the thing that
    makes coverage planning more than a nested loop: a stripe is not one pass,
    it is however many passes the furniture leaves, and those have to be ordered
    and connected afterwards.
    """
    line = grid[index, :] if axis == 0 else grid[:, index]
    # TODO 2: walk along `line` and collect the inclusive (start, end) index
    #         pairs of every maximal run of free cells. Remember the run that is
    #         still open when you reach the end of the line.
    raise NotImplementedError("TODO: implement free_runs")


# ---------------------------------------------------------------------------
# Exercise 4.9b. The plan
# ---------------------------------------------------------------------------

def _cell(axis: int, line: int, along: int) -> Cell:
    """Given complete. Assemble a (row, col) from a line index and a position."""
    return (line, along) if axis == 0 else (along, line)


def _connect(grid: np.ndarray, a: Cell, b: Cell):
    """Route between two sweep runs. Given complete, and worth reading.

    It calls the A* you wrote in Exercise 4.2. A coverage planner is not a
    replacement for a point to point planner, it is a CLIENT of one, and that is
    also how it works in production: Nav2's coverage server plans the sweep and
    hands the connecting moves to the ordinary navigation stack.
    """
    if a == b:
        return [a]
    plan = astar(grid, a, b, MOVES_8, octile)
    return plan.path if plan.found else None


def boustrophedon(grid: np.ndarray, spacing: int, axis: int = 0,
                  connect: bool = True,
                  tool_width: int | None = None) -> CoverageResult:
    """Plan a back-and-forth sweep over all reachable free space.

    `spacing` is how far apart the passes are driven. `tool_width` is how wide
    the brush, blade or sensor actually is. **They are different numbers and
    conflating them is the mistake that makes a coverage planner look perfect
    at every setting**: widen the gap between passes, widen the imaginary brush
    to match, and the uncovered stripes disappear from the measurement while
    remaining on the floor.
    """
    if tool_width is None:
        tool_width = spacing
    result = CoverageResult()
    path: list[Cell] = []

    forward = True
    for line in sweep_lines(grid, spacing, axis):
        runs = free_runs(grid, line, axis)
        if not runs:
            continue

        # TODO 3: drive the runs in alternating order, left to right on one
        #         stripe and right to left on the next. Driving every stripe
        #         the same way adds a wasted traverse of the room between them.
        #
        # TODO 4: build each run's cells with _cell(axis, line, i), in the
        #         direction you are driving it.
        #
        # TODO 5: if the path is not empty and `connect` is set, join the last
        #         cell of the path to the run's first cell with _connect. None
        #         means unreachable, so skip that run.
        #
        #         The bridge already ends at the run's first cell, so append
        #         bridge[1:] then run_cells[1:]. Appending the whole run repeats
        #         that cell and leaves a zero length step. There is a test.
        #
        #         Increment result.segments for each run you drive.
        raise NotImplementedError("TODO: implement boustrophedon")

    result.path = path
    result.turns = count_turns(path)
    result.length = float(len(path))
    result.covered = coverage_fraction(grid, path, tool_width)
    return result


# ---------------------------------------------------------------------------
# Exercise 4.9c. The measurement
# ---------------------------------------------------------------------------

def count_turns(path: list[Cell]) -> int:
    """How many times the direction of travel changes."""
    # TODO 6: walk the path comparing each step's direction to the previous
    #         one, and count the changes. A path shorter than three cells has
    #         no turns.
    raise NotImplementedError("TODO: implement count_turns")


def coverage_fraction(grid: np.ndarray, path: list[Cell], tool_width: int) -> float:
    """Fraction of REACHABLE free cells swept by a tool `tool_width` cells wide.

    Reachable is the important word. A map with a sealed cupboard contains free
    cells no robot can ever enter. Counting those as failures means no planner
    can score 100 percent and the number stops telling you anything about the
    planner, so the denominator is the free space connected to where the robot
    actually started.
    """
    if not path:
        return 0.0
    # TODO 7: mark every cell within tool_width // 2 of any path cell as swept,
    #         then remove any that are obstacles.
    #
    # TODO 8: use _reachable_free(grid, path[0]) as the denominator, not the
    #         total free area. Return swept-and-reachable over reachable.
    raise NotImplementedError("TODO: implement coverage_fraction")


def _reachable_free(grid: np.ndarray, start: Cell) -> np.ndarray:
    """Flood fill of free cells connected to `start`. Given complete."""
    h, w = grid.shape
    seen = np.zeros_like(grid, dtype=bool)
    if grid[start]:
        return seen
    stack = [start]
    seen[start] = True
    while stack:
        r, c = stack.pop()
        for dr, dc in MOVES_8:
            nr, nc = r + dr, c + dc
            if 0 <= nr < h and 0 <= nc < w and not grid[nr, nc] and not seen[nr, nc]:
                seen[nr, nc] = True
                stack.append((nr, nc))
    return seen


def compare_axes(grid: np.ndarray, spacing: int,
                 tool_width: int | None = None) -> dict[str, CoverageResult]:
    """Sweep along rows, then along columns, and compare. Given complete.

    The single most useful experiment here. Both cover the same area and one
    takes far fewer turns, because the sweep should run along the LONG axis of
    the space. On the 6 m by 4 m living room it is about 40 turns against 62.

    Run it before you guess. Exercise 4.9 asks you to explain the result, and
    then to say what a lawn with a long thin shape implies for a mower that
    plans its own passes.
    """
    return {"rows": boustrophedon(grid, spacing, 0, tool_width=tool_width),
            "columns": boustrophedon(grid, spacing, 1, tool_width=tool_width)}
