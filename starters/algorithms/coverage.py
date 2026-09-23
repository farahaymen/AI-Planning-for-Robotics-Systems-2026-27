"""
Coverage path planning: visit all of the free space, not a point in it.

Vacuum cleaners, lawn mowers, floor scrubbers and agricultural sprayers have no
goal pose. They have an area to cover, which is a different problem with a
different metric: coverage achieved and the number of turns it took.

Turns are the cost. A straight run happens at full speed with the tool at rated
throughput; every turn means decelerate, rotate, accelerate, and often
disengage the tool. Forty long passes finish sooner than 120 short ones at the
same total distance.

Boustrophedon is Greek for "as the ox turns", up one furrow and back down the
next. Nav2 ships a Coverage Server built on Fields2Cover.

Reference implementation. The version you fill in is `coverage_skeleton.py`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

try:
    from .gridmap import MOVES_8
except ImportError:  # Standalone exercise.
    from gridmap import MOVES_8
try:
    from .planners import astar, octile
except ImportError:  # Standalone exercise.
    from planners import astar, octile

Cell = tuple[int, int]


@dataclass
class CoverageResult:
    """One coverage run, with the numbers that actually matter."""

    path: list[Cell] = field(default_factory=list)
    covered: float = 0.0          # fraction of reachable free cells swept
    turns: int = 0                # direction changes: the real cost
    length: float = 0.0           # cells travelled
    segments: int = 0             # sweep runs before connections

    def __str__(self) -> str:
        return (f"covered {self.covered * 100:5.1f}%  turns {self.turns:4d}  "
                f"length {self.length:7.1f}  segments {self.segments:3d}")


def sweep_lines(grid: np.ndarray, spacing: int, axis: int = 0) -> list[int]:
    """The rows (axis 0) or columns (axis 1) the robot will drive along.

    Spacing is the sweep width in cells, which is the robot's working width, not
    its footprint. Slightly less, in practice, because a gap between passes
    leaves an uncleaned stripe and customers notice stripes.
    """
    limit = grid.shape[axis]
    # Start half a spacing in, so the first pass is centred in its stripe rather
    # than hard against the wall.
    return list(range(spacing // 2, limit, spacing))


def free_runs(grid: np.ndarray, index: int, axis: int = 0) -> list[tuple[int, int]]:
    """Maximal runs of free cells along one sweep line, as (start, end) pairs.

    An obstacle in the middle of a line SPLITS it. This is the thing that makes
    coverage planning more than a nested loop: a stripe is not one pass, it is
    however many passes the furniture leaves, and they have to be ordered and
    connected.
    """
    line = grid[index, :] if axis == 0 else grid[:, index]
    runs, start = [], None
    for i, blocked in enumerate(line):
        if not blocked and start is None:
            start = i
        elif blocked and start is not None:
            runs.append((start, i - 1))
            start = None
    if start is not None:
        runs.append((start, len(line) - 1))
    return runs


def _cell(axis: int, line: int, along: int) -> Cell:
    return (line, along) if axis == 0 else (along, line)


def boustrophedon(grid: np.ndarray, spacing: int, axis: int = 0,
                  connect: bool = True, tool_width: int | None = None) -> CoverageResult:
    """Plan a back-and-forth sweep over all reachable free space.

    1. Slice the map into stripes `spacing` cells apart.
    2. In each stripe find the free runs, and drive them in alternating
       direction so the turns stay short.
    3. Join the end of one run to the start of the next with A*. A coverage
       planner is a client of a point to point planner, which is also how
       Nav2's coverage server works.

    `spacing` is how far apart the passes are driven. `tool_width` is how wide
    the brush or blade is. They are different numbers, and passing the spacing
    as the tool width makes every setting score 100 percent while the floor
    stays dirty.

    They default to equal, the edge case where passes just touch. Real machines
    overlap by ten to twenty percent to allow for slip and localisation error.
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
        ordered = runs if forward else list(reversed(runs))
        for start, end in ordered:
            a, b = (start, end) if forward else (end, start)
            step = 1 if b >= a else -1
            run_cells = [_cell(axis, line, i) for i in range(a, b + step, step)]

            if path and connect:
                bridge = _connect(grid, path[-1], run_cells[0])
                if bridge is None:
                    continue           # unreachable from here; skip this run
                # The bridge already ENDS at run_cells[0]. Appending the whole
                # run after it repeats that cell, and a path with a repeated
                # point has a zero length step in it, which every downstream
                # consumer treats as a teleport or a divide by zero.
                path.extend(bridge[1:])
                path.extend(run_cells[1:])
            else:
                path.extend(run_cells)
            result.segments += 1
        forward = not forward

    result.path = path
    result.turns = count_turns(path)
    result.length = float(len(path))
    result.covered = coverage_fraction(grid, path, tool_width)
    return result


def _connect(grid: np.ndarray, a: Cell, b: Cell):
    """Route between two sweep runs using the point to point planner.

    Adjacent runs are usually one step apart and A* returns immediately. When
    an obstacle sits between them it does the real work, which is exactly why
    the coverage planner does not try to do it itself.
    """
    if a == b:
        return [a]
    plan = astar(grid, a, b, MOVES_8, octile)
    return plan.path if plan.found else None


def count_turns(path: list[Cell]) -> int:
    """How many times the direction of travel changes.

    The metric that decides how long a coverage run really takes, and the one a
    planner optimising path length ignores. Every turn is decelerate, rotate,
    accelerate, and on a real machine often disengage and re-engage the tool.
    """
    if len(path) < 3:
        return 0
    turns = 0
    previous = (path[1][0] - path[0][0], path[1][1] - path[0][1])
    for i in range(1, len(path) - 1):
        d = (path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
        if d != previous:
            turns += 1
            previous = d
    return turns


def coverage_fraction(grid: np.ndarray, path: list[Cell], tool_width: int) -> float:
    """Fraction of REACHABLE free cells swept by a tool `tool_width` cells wide.

    Note the tool width, not the sweep spacing. Passing the spacing here is the
    mistake that makes a planner look perfect at any setting: widen the gap
    between passes, widen the imaginary brush to match, and the uncovered
    stripes vanish from the measurement while remaining on the floor.

    Reachable matters. A map with a sealed cupboard in it has free cells no
    robot can ever enter, and counting those as failures means no planner can
    ever score 100 percent and the metric stops being informative. So the
    denominator is the free space connected to where the robot actually went.
    """
    if not path:
        return 0.0
    h, w = grid.shape
    swept = np.zeros_like(grid, dtype=bool)
    half = max(0, tool_width // 2)

    for (r, c) in path:
        r0, r1 = max(0, r - half), min(h, r + half + 1)
        c0, c1 = max(0, c - half), min(w, c + half + 1)
        swept[r0:r1, c0:c1] = True
    swept &= ~grid

    reachable = _reachable_free(grid, path[0])
    total = int(reachable.sum())
    if total == 0:
        return 0.0
    return float((swept & reachable).sum()) / total


def _reachable_free(grid: np.ndarray, start: Cell) -> np.ndarray:
    """Flood fill of free cells connected to `start`, eight connected."""
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
    """Sweep along rows, then along columns, and compare.

    The single most useful experiment in this module. Both cover the same area
    and one of them takes far fewer turns, because the sweep should run along
    the LONG axis of the space: fewer, longer passes and fewer end turns.

    On a corridor shaped room the difference is large enough that no reasonable
    person would choose by instinct, which is the point. Measure it.
    """
    return {"rows": boustrophedon(grid, spacing, 0, tool_width=tool_width),
            "columns": boustrophedon(grid, spacing, 1, tool_width=tool_width)}
