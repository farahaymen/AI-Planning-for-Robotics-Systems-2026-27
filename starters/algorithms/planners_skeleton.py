"""
Lab 4, Exercise 4.2. Implement the four search algorithms.

This is the file you edit. Fill in every block marked TODO and nothing else;
the helpers above and below them are already correct and the tests depend on
them staying that way.

How to work:

    python3 -m pytest tests/ -x -q          run the tests, stop at the first failure
    python3 compare.py --skeleton           run YOUR planners on all six maps
    python3 compare.py                      run the reference ones, to compare against

The tests are the specification. Read `tests/test_planners.py` before you start
writing; it tells you exactly what each function has to do, including the
awkward cases you would otherwise discover in week 9 on the real robot.

A warning worth taking seriously. Three of these four algorithms differ from
each other by one or two lines. BFS and DFS differ by `popleft()` against
`pop()`. Dijkstra and A* differ by one `+ heuristic(...)` in the priority. It
is very easy to write something that runs, returns a path, draws a picture that
looks right, and is not the algorithm you meant. That is why the tests check
expansion counts and path costs and not just "a path came back".
"""

from __future__ import annotations

import heapq
import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Optional

import numpy as np

try:
    from .gridmap import MOVES_4, MOVES_8, path_length
except ImportError:  # Standalone exercise.
    from gridmap import MOVES_4, MOVES_8, path_length

Cell = tuple[int, int]


@dataclass
class PlanResult:
    """One planning run, with everything the comparison table needs.

    Given to you complete. Every planner must fill in `path`, `found`,
    `expanded_cells`, `generated`, `max_frontier` and `runtime_ms`, because the
    comparison is only fair if all four report the same things the same way.
    """

    name: str
    path: list[Cell] = field(default_factory=list)
    expanded_cells: list[Cell] = field(default_factory=list)
    generated: int = 0           # neighbours ever pushed, including duplicates
    runtime_ms: float = 0.0
    found: bool = False
    max_frontier: int = 0        # peak memory pressure, in nodes

    @property
    def expanded(self) -> int:
        """How many cells were popped and processed.

        Kept as the length of `expanded_cells` rather than a separate counter,
        so the number in the table and the picture in the figure can never
        disagree. Appending the cell is the counter.
        """
        return len(self.expanded_cells)

    @property
    def length(self) -> float:
        return path_length(self.path)

    @property
    def steps(self) -> int:
        return max(0, len(self.path) - 1)


# ---------------------------------------------------------------------------
# Helpers. These are complete. Do not change them.
# ---------------------------------------------------------------------------

def _neighbours(grid: np.ndarray, cell: Cell, moves) -> list[Cell]:
    """Free, in-bounds neighbours of `cell`, without cutting wall corners."""
    r, c = cell
    h, w = grid.shape
    out = []
    for dr, dc in moves:
        nr, nc = r + dr, c + dc
        if 0 <= nr < h and 0 <= nc < w and not grid[nr, nc]:
            # A diagonal move between two blocked cells would slice through a
            # wall junction, which a robot with a body cannot do.
            if dr != 0 and dc != 0:
                if grid[r + dr, c] and grid[r, c + dc]:
                    continue
            out.append((nr, nc))
    return out


def _reconstruct(came_from: dict[Cell, Cell], start: Cell, goal: Cell) -> list[Cell]:
    """Walk the parent links back from the goal and reverse them."""
    path, node = [goal], goal
    while node != start:
        node = came_from[node]
        path.append(node)
    path.reverse()
    return path


def _step_cost(a: Cell, b: Cell) -> float:
    """1 for a straight move, sqrt(2) for a diagonal one."""
    return math.sqrt(2.0) if (a[0] != b[0] and a[1] != b[1]) else 1.0


# ---------------------------------------------------------------------------
# Exercise 4.2a. Uninformed search.
# ---------------------------------------------------------------------------

def bfs(grid, start: Cell, goal: Cell, moves=MOVES_4) -> PlanResult:
    """Breadth first search: expand in order of STEP COUNT from the start.

    The frontier is a FIFO queue, so every cell one step from the start is
    expanded before any cell two steps away, and the first time you reach the
    goal you have reached it in the fewest possible steps.

    One decision matters more than the rest. Mark a cell visited when you PUSH
    it, not when you pop it. Mark at pop and the same cell enters the queue once
    per neighbour that finds it, the queue grows quadratically, and on a 30x30
    map you will not notice; on a 2000x2000 costmap you will.
    """
    result = PlanResult("BFS")
    t0 = time.perf_counter()

    frontier = deque([start])
    came_from: dict[Cell, Cell] = {}
    visited = {start}

    while frontier:
        result.max_frontier = max(result.max_frontier, len(frontier))

        # TODO 1: take the next cell off the FRONT of the queue and append it to
        #         result.expanded_cells (that list IS the expansion counter, and
        #         it is what draws the heatmap in figures.py). Then, if it is the
        #         goal, set result.path using _reconstruct, set result.found,
        #         and break.
        #
        # TODO 2: for each neighbour from _neighbours(grid, node, moves):
        #         count it in result.generated, and if it has not been visited,
        #         mark it visited, record came_from[neighbour] = node, and push
        #         it onto the BACK of the queue.
        raise NotImplementedError("TODO: implement bfs")

    result.runtime_ms = (time.perf_counter() - t0) * 1000.0
    return result


def dfs(grid, start: Cell, goal: Cell, moves=MOVES_4) -> PlanResult:
    """Depth first search: follow one branch to exhaustion, then back up.

    Structurally this is BFS with a stack instead of a queue, and that single
    change costs you every guarantee BFS had. The path it returns is valid and
    can be many times longer than necessary. Run it on `corridors` and look at
    the number.

    Two differences from BFS beyond the container, and both are deliberate:
    check `visited` when you POP rather than when you push, and let came_from be
    overwritten freely. DFS makes no claim about the route, so there is nothing
    to protect.
    """
    result = PlanResult("DFS")
    t0 = time.perf_counter()

    frontier = [start]
    came_from: dict[Cell, Cell] = {}
    visited: set[Cell] = set()

    while frontier:
        result.max_frontier = max(result.max_frontier, len(frontier))

        # TODO 3: pop from the END of the list. If the cell is already in
        #         visited, skip it. Otherwise mark it visited, append it to
        #         result.expanded_cells, and handle the goal exactly as in bfs.
        #
        # TODO 4: push every unvisited neighbour, recording came_from as you go.
        raise NotImplementedError("TODO: implement dfs")

    result.runtime_ms = (time.perf_counter() - t0) * 1000.0
    return result


# ---------------------------------------------------------------------------
# Exercise 4.2b. Informed search.
# ---------------------------------------------------------------------------

def dijkstra(grid, start: Cell, goal: Cell, moves=MOVES_4) -> PlanResult:
    """Dijkstra: expand in order of PATH COST from the start.

    BFS counts moves; this counts distance. That is the whole difference, and it
    is why Dijkstra stays optimal once diagonals cost sqrt(2) and BFS does not.

    Use a heap, keyed on cost. `heapq` has no decrease-key, so a cell can sit in
    the heap several times with different costs. The standard answer is a
    `closed` set: when you pop a cell you have already closed, it is a stale
    copy, so skip it. This is cheaper than trying to remove entries.
    """
    result = PlanResult("Dijkstra")
    t0 = time.perf_counter()

    frontier: list[tuple[float, Cell]] = [(0.0, start)]
    cost = {start: 0.0}
    came_from: dict[Cell, Cell] = {}
    closed: set[Cell] = set()

    while frontier:
        result.max_frontier = max(result.max_frontier, len(frontier))

        # TODO 5: pop the cheapest (g, node). If node is in closed, continue.
        #         Otherwise close it, append it to result.expanded_cells, and
        #         handle the goal.
        #
        # TODO 6: for each neighbour, compute new_cost = g + _step_cost(node, nb).
        #         If that beats cost.get(nb, math.inf), store it, record the
        #         parent, and push (new_cost, nb).
        raise NotImplementedError("TODO: implement dijkstra")

    result.runtime_ms = (time.perf_counter() - t0) * 1000.0
    return result


def manhattan(a: Cell, b: Cell) -> float:
    """Sum of the row and column differences. Correct for FOUR-connected grids."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def euclidean(a: Cell, b: Cell) -> float:
    """Straight-line distance. Always admissible, often loose."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def octile(a: Cell, b: Cell) -> float:
    """The correct heuristic for EIGHT-connected movement.

    TODO 7: implement it. The cheapest possible route to the goal, ignoring
    every obstacle, takes as many diagonal steps as it can and straight steps
    for the remainder. With dr and dc as the absolute row and column distances,
    that is min(dr, dc) diagonals and max(dr, dc) - min(dr, dc) straights.

    Write the total cost of that and simplify. Check your answer against the
    test: octile((0, 0), (3, 3)) must be 3 * sqrt(2), and octile((0, 0), (0, 5))
    must be 5.
    """
    raise NotImplementedError("TODO: implement octile")


def inadmissible(a: Cell, b: Cell, weight: float = 4.0) -> float:
    """Manhattan, scaled up. Deliberately overestimates the remaining cost.

    Admissible means the heuristic never overestimates the true remaining cost.
    Break that and A* is no longer guaranteed to return a shortest path: it
    expands far fewer cells, returns faster, and the path can be longer. The
    notebook asks you to measure exactly that trade on `cluttered` and
    `greedy_trap`, so leave this function alone.
    """
    return weight * manhattan(a, b)


def astar(grid, start: Cell, goal: Cell, moves=MOVES_4,
          heuristic: Callable[[Cell, Cell], float] = manhattan,
          name: Optional[str] = None) -> PlanResult:
    """A*: expand in order of cost-so-far PLUS estimated cost-to-go.

    Copy your Dijkstra and change one thing: the value you push as the heap key
    becomes new_cost + heuristic(nb, goal). Keep pushing the true cost alongside
    it so you still have g when you pop, which is why the heap entries here are
    (f, g, cell) rather than (g, cell).

    Two checks you can run on your own work. With a heuristic that returns 0,
    A* must expand exactly what Dijkstra expands, because the priority is then
    identical. With the octile heuristic on `empty_room` it must expand about 26
    cells against Dijkstra's 777, because the estimate is exact when there is
    nothing in the way.
    """
    result = PlanResult(name or f"A* ({heuristic.__name__})")
    t0 = time.perf_counter()

    frontier: list[tuple[float, float, Cell]] = [(heuristic(start, goal), 0.0, start)]
    cost = {start: 0.0}
    came_from: dict[Cell, Cell] = {}
    closed: set[Cell] = set()

    while frontier:
        result.max_frontier = max(result.max_frontier, len(frontier))

        # TODO 8: as Dijkstra, but the heap entries are (f, g, cell) and the
        #         key you push is new_cost + heuristic(nb, goal).
        #         Expand on g, order on f. Mixing those up is the usual bug and
        #         it produces a planner that still works, just not optimally.
        raise NotImplementedError("TODO: implement astar")

    result.runtime_ms = (time.perf_counter() - t0) * 1000.0
    return result


# ---------------------------------------------------------------------------
# Exercise 4.2c, optional. Greedy best first.
# ---------------------------------------------------------------------------

def greedy_best_first(grid, start: Cell, goal: Cell, moves=MOVES_4,
                      heuristic: Callable[[Cell, Cell], float] = octile) -> PlanResult:
    """A* with the cost-so-far term removed. Orders the frontier on h alone.

    TODO 9 (optional): implement it, then run it on `greedy_trap`. It returns a
    path 33 percent longer than the shortest one, because every cell inside the
    box is nearer the goal than the cells outside and "nearer the goal" is the
    only thing it looks at. This is the clearest demonstration that the g term
    in A* is load bearing.
    """
    raise NotImplementedError("TODO (optional): implement greedy_best_first")


def zero(a: Cell, b: Cell) -> float:
    """The null heuristic. A* with this IS Dijkstra; verify that yourself."""
    return 0.0


def _with(h, label):
    def run(grid, start, goal, moves=MOVES_4):
        return astar(grid, start, goal, moves, h, label)
    return run


PLANNERS: dict[str, Callable[..., PlanResult]] = {
    "BFS": bfs,
    "DFS": dfs,
    "Dijkstra": dijkstra,
    "A* octile": _with(octile, "A* octile"),
    "A* euclidean": _with(euclidean, "A* euclidean"),
    "A* manhattan": _with(manhattan, "A* manhattan"),
    "A* inadmissible": _with(inadmissible, "A* inadmissible"),
    "A* zero (= Dijkstra)": _with(zero, "A* zero (= Dijkstra)"),
    "Greedy best first": greedy_best_first,
}

OPTIMAL_ON_8 = ("Dijkstra", "A* octile", "A* euclidean", "A* zero (= Dijkstra)")
