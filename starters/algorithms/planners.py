"""
The four search algorithms, in plain Python.

Every planner has the same signature and returns the same PlanResult, so the
comparison in the notebook is a fair one: same map, same start, same goal, same
metrics. That uniformity is the point. A comparison where each algorithm is
called differently and measured differently is not a comparison.

Only heapq and collections are used. No planning libraries, because the whole
exercise is understanding what the library would have done.

REFERENCE IMPLEMENTATION. The version you are given in the lab has the bodies
removed; see planners_skeleton.py.
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
    """One planning run, with everything the comparison table needs."""

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

    def __str__(self) -> str:
        if not self.found:
            return f"{self.name:22s} NO PATH   expanded {self.expanded:5d}  {self.runtime_ms:7.2f} ms"
        return (f"{self.name:22s} len {self.length:7.2f}  expanded {self.expanded:5d}  "
                f"frontier {self.max_frontier:4d}  {self.runtime_ms:7.2f} ms")


def _neighbours(grid: np.ndarray, cell: Cell, moves) -> list[Cell]:
    r, c = cell
    h, w = grid.shape
    out = []
    for dr, dc in moves:
        nr, nc = r + dr, c + dc
        if 0 <= nr < h and 0 <= nc < w and not grid[nr, nc]:
            # Do not allow a diagonal move to cut a corner between two blocked
            # cells. Without this the path slices through wall junctions, which
            # a real robot with a body cannot do.
            if dr != 0 and dc != 0:
                if grid[r + dr, c] and grid[r, c + dc]:
                    continue
            out.append((nr, nc))
    return out


def _reconstruct(came_from: dict[Cell, Cell], start: Cell, goal: Cell) -> list[Cell]:
    path, node = [goal], goal
    while node != start:
        node = came_from[node]
        path.append(node)
    path.reverse()
    return path


def _step_cost(a: Cell, b: Cell) -> float:
    return math.sqrt(2.0) if (a[0] != b[0] and a[1] != b[1]) else 1.0


# ---------------------------------------------------------------------------
# Uninformed search
# ---------------------------------------------------------------------------

def bfs(grid, start: Cell, goal: Cell, moves=MOVES_4) -> PlanResult:
    """Breadth first search. Explores in order of STEP COUNT from the start.

    Optimal on an unweighted graph, which a four-connected grid is: every move
    costs 1. Add diagonals and it stops being optimal, because a diagonal step
    still counts as one move while being sqrt(2) long. That is the point of the
    notebook question about eight-connected movement.
    """
    result = PlanResult("BFS")
    t0 = time.perf_counter()

    frontier = deque([start])
    came_from: dict[Cell, Cell] = {}
    # Marking visited at PUSH time, not pop time, is what keeps BFS linear. Mark
    # at pop and the same cell enters the queue many times over.
    visited = {start}

    while frontier:
        result.max_frontier = max(result.max_frontier, len(frontier))
        node = frontier.popleft()
        result.expanded_cells.append(node)

        if node == goal:
            result.path = _reconstruct(came_from, start, goal)
            result.found = True
            break

        for nb in _neighbours(grid, node, moves):
            result.generated += 1
            if nb not in visited:
                visited.add(nb)
                came_from[nb] = node
                frontier.append(nb)

    result.runtime_ms = (time.perf_counter() - t0) * 1000.0
    return result


def dfs(grid, start: Cell, goal: Cell, moves=MOVES_4) -> PlanResult:
    """Depth first search. Follows one branch to exhaustion before backtracking.

    Not optimal and not even close. It is here so you can see how bad a path
    can be while still being a valid path, and because the contrast with BFS,
    one line of difference between a queue and a stack, is the clearest possible
    illustration of what the frontier ordering does.
    """
    result = PlanResult("DFS")
    t0 = time.perf_counter()

    frontier = [start]
    came_from: dict[Cell, Cell] = {}
    visited: set[Cell] = set()

    while frontier:
        result.max_frontier = max(result.max_frontier, len(frontier))
        node = frontier.pop()          # the only structural difference from BFS
        if node in visited:
            continue
        visited.add(node)
        result.expanded_cells.append(node)

        if node == goal:
            result.path = _reconstruct(came_from, start, goal)
            result.found = True
            break

        for nb in _neighbours(grid, node, moves):
            result.generated += 1
            if nb not in visited:
                came_from[nb] = node   # overwritten freely; DFS makes no claim
                frontier.append(nb)

    result.runtime_ms = (time.perf_counter() - t0) * 1000.0
    return result


# ---------------------------------------------------------------------------
# Informed search
# ---------------------------------------------------------------------------

def dijkstra(grid, start: Cell, goal: Cell, moves=MOVES_4) -> PlanResult:
    """Dijkstra. Explores in order of PATH COST from the start.

    Optimal for any non-negative edge costs, so unlike BFS it stays optimal once
    diagonals cost sqrt(2). It knows nothing about where the goal is, so it
    expands outwards in every direction equally: a circle of explored cells
    centred on the start.
    """
    result = PlanResult("Dijkstra")
    t0 = time.perf_counter()

    frontier: list[tuple[float, Cell]] = [(0.0, start)]
    cost = {start: 0.0}
    came_from: dict[Cell, Cell] = {}
    closed: set[Cell] = set()

    while frontier:
        result.max_frontier = max(result.max_frontier, len(frontier))
        g, node = heapq.heappop(frontier)
        # heapq has no decrease-key, so a cell can sit in the heap several times
        # with different costs. Skipping the stale copies here is the standard
        # way round that and is cheaper than trying to remove them.
        if node in closed:
            continue
        closed.add(node)
        result.expanded_cells.append(node)

        if node == goal:
            result.path = _reconstruct(came_from, start, goal)
            result.found = True
            break

        for nb in _neighbours(grid, node, moves):
            result.generated += 1
            new_cost = g + _step_cost(node, nb)
            if new_cost < cost.get(nb, math.inf):
                cost[nb] = new_cost
                came_from[nb] = node
                heapq.heappush(frontier, (new_cost, nb))

    result.runtime_ms = (time.perf_counter() - t0) * 1000.0
    return result


def manhattan(a: Cell, b: Cell) -> float:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def euclidean(a: Cell, b: Cell) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def octile(a: Cell, b: Cell) -> float:
    """The correct heuristic for eight-connected movement.

    Straight moves cost 1 and diagonals sqrt(2), so the cheapest possible route
    takes as many diagonals as it can and straight moves for the remainder.
    This is admissible on an eight-connected grid where Manhattan is not.
    """
    dr, dc = abs(a[0] - b[0]), abs(a[1] - b[1])
    return (math.sqrt(2.0) - 1.0) * min(dr, dc) + max(dr, dc)


def inadmissible(a: Cell, b: Cell, weight: float = 4.0) -> float:
    """Manhattan, scaled up. Deliberately overestimates the remaining cost.

    Admissibility means the heuristic never overestimates. Break it and A* is no
    longer guaranteed optimal: it finds a path faster, by expanding fewer nodes,
    and that path can be longer than the best one. The notebook asks you to
    measure exactly that trade.
    """
    return weight * manhattan(a, b)


def astar(grid, start: Cell, goal: Cell, moves=MOVES_4,
          heuristic: Callable[[Cell, Cell], float] = manhattan,
          name: Optional[str] = None) -> PlanResult:
    """A*. Explores in order of cost-so-far plus estimated cost-to-go.

    Identical to Dijkstra except for the `+ heuristic(...)` in the priority. That
    one term is what turns a circle of expanded cells into a beam pointed at the
    goal. With h = 0 it IS Dijkstra, which is a useful thing to verify yourself.
    """
    result = PlanResult(name or f"A* ({heuristic.__name__})")
    t0 = time.perf_counter()

    frontier: list[tuple[float, float, Cell]] = [(heuristic(start, goal), 0.0, start)]
    cost = {start: 0.0}
    came_from: dict[Cell, Cell] = {}
    closed: set[Cell] = set()

    while frontier:
        result.max_frontier = max(result.max_frontier, len(frontier))
        _f, g, node = heapq.heappop(frontier)
        if node in closed:
            continue
        closed.add(node)
        result.expanded_cells.append(node)

        if node == goal:
            result.path = _reconstruct(came_from, start, goal)
            result.found = True
            break

        for nb in _neighbours(grid, node, moves):
            result.generated += 1
            new_cost = g + _step_cost(node, nb)
            if new_cost < cost.get(nb, math.inf):
                cost[nb] = new_cost
                came_from[nb] = node
                heapq.heappush(frontier, (new_cost + heuristic(nb, goal), new_cost, nb))

    result.runtime_ms = (time.perf_counter() - t0) * 1000.0
    return result


def greedy_best_first(grid, start: Cell, goal: Cell, moves=MOVES_4,
                      heuristic: Callable[[Cell, Cell], float] = octile) -> PlanResult:
    """Greedy best first. Orders the frontier by the heuristic ALONE.

    A* without the cost-so-far term. It is fast and it is not optimal, because
    nothing in the priority remembers what the route has already cost. On
    `greedy_trap` it descends into the box for exactly this reason: every cell
    in there is nearer the goal than the cells outside, and "nearer the goal" is
    the only thing it is looking at.

    Included so you can see that the g term in A* is not decoration. Delete it
    and you get this.
    """
    result = PlanResult("Greedy best first")
    t0 = time.perf_counter()

    frontier: list[tuple[float, Cell]] = [(heuristic(start, goal), start)]
    came_from: dict[Cell, Cell] = {}
    closed: set[Cell] = set()

    while frontier:
        result.max_frontier = max(result.max_frontier, len(frontier))
        _h, node = heapq.heappop(frontier)
        if node in closed:
            continue
        closed.add(node)
        result.expanded_cells.append(node)

        if node == goal:
            result.path = _reconstruct(came_from, start, goal)
            result.found = True
            break

        for nb in _neighbours(grid, node, moves):
            result.generated += 1
            if nb not in closed and nb not in came_from:
                came_from[nb] = node
                heapq.heappush(frontier, (heuristic(nb, goal), nb))

    result.runtime_ms = (time.perf_counter() - t0) * 1000.0
    return result


def zero(a: Cell, b: Cell) -> float:
    """The null heuristic. A* with this IS Dijkstra; verify that yourself."""
    return 0.0


def _with(h, label):
    def run(grid, start, goal, moves=MOVES_4):
        return astar(grid, start, goal, moves, h, label)
    return run


# The comparison table in the notebook iterates over this in order.
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

# Which of those are guaranteed to return a shortest path on an eight-connected
# grid. The tests assert this list and only this list. BFS is absent on purpose:
# it is optimal in step count, which is not the same thing once diagonals cost
# sqrt(2). See gridmap.steps_vs_cost.
OPTIMAL_ON_8 = ("Dijkstra", "A* octile", "A* euclidean", "A* zero (= Dijkstra)")
