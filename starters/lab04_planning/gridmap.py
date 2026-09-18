"""
Grid maps for the Lab 4 path planning notebook.

A map here is a 2D NumPy boolean array: True means blocked. Indexing is
[row, col] throughout, which matches how a ROS OccupancyGrid reshapes and how
Lab 6 will hand you a real costmap. Get used to (row, col) rather than (x, y)
now; mixing them up produces a transposed map that looks plausible and is wrong.

The six maps are not decoration. Each one exists because it makes a specific
pair of algorithms disagree, and a comparison on a map where everything ties
teaches nothing. What each map is for:

    empty_room     the control. Every optimal planner must agree here, and the
                   only thing that differs is how many cells each one touched.
    corridors      forced detour. The straight-line heuristic points at a wall
                   for most of the search, so A* loses most of its advantage.
    greedy_trap    a three-sided box straddling the line from start to goal.
                   A planner that trusts the heuristic too much drives into it.
    cluttered      random obstacles, seed 7. This is where an inadmissible
                   heuristic quietly returns a longer path.
    steps_vs_cost  random obstacles, seed 0. Chosen because BFS returns a path
                   with FEWER steps and MORE length than Dijkstra, which is the
                   whole argument for why step count is not cost.
    sealed_goal    no path exists. Every planner must terminate and say so.

Run this file directly to print all six as ASCII.
"""

from __future__ import annotations

import numpy as np

# Four-connected movement: the four edge neighbours, every move costs 1.
MOVES_4 = ((-1, 0), (1, 0), (0, -1), (0, 1))
# Eight-connected: add the four corner neighbours, which cost sqrt(2).
MOVES_8 = MOVES_4 + ((-1, -1), (-1, 1), (1, -1), (1, 1))

# Standard start and goal for every map, so the numbers are comparable.
START = (2, 2)
GOAL = (27, 27)


def empty_room(height: int = 30, width: int = 30) -> np.ndarray:
    """No obstacles inside a bounding wall.

    The control case. Every optimal planner returns the same 35.36 here, so any
    difference you see in the table is purely search effort, not path quality.
    """
    grid = np.zeros((height, width), dtype=bool)
    grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = True
    return grid


def corridors(height: int = 30, width: int = 30) -> np.ndarray:
    """Three vertical walls with doorways that alternate top and bottom.

    The route has to zig-zag the full height of the room three times, so the
    straight-line distance to the goal is a bad estimate almost everywhere. A*
    still beats Dijkstra here but by much less than it does in open space, and
    that shrinking margin is the thing worth noticing.
    """
    grid = empty_room(height, width)
    for i, x in enumerate((width // 4, width // 2, 3 * width // 4)):
        grid[1:-1, x] = True
        if i % 2 == 0:
            grid[height - 6:height - 1, x] = False
        else:
            grid[1:6, x] = False
    return grid


def greedy_trap(height: int = 30, width: int = 30) -> np.ndarray:
    """A three-sided box, open at the top, straddling the line to the goal.

    Everything inside the box is closer to the goal in straight-line terms than
    the cells outside it, so a search that weights the heuristic too heavily
    descends into the box, runs out of floor, and has to climb back out. Plot
    the path from `astar(..., heuristic=inadmissible)` on this map and you can
    see the dive and the recovery in the drawn line.

    BFS and Dijkstra never enter it in preference to anything else, because
    neither one knows where the goal is.
    """
    grid = empty_room(height, width)
    grid[21, 8:24] = True        # floor of the box
    grid[10:22, 8] = True        # left wall
    grid[10:22, 23] = True       # right wall
    return grid


def cluttered(height: int = 30, width: int = 30, seed: int = 7,
              n_blocks: int = 22) -> np.ndarray:
    """Random rectangular obstacles, reproducible from the seed.

    Seed 7 is the default because on it A* with the Manhattan heuristic returns
    a path 1.5 percent longer than optimal while expanding 38 cells against the
    182 that the correct octile heuristic needs. That is the honest shape of the
    trade: the wrong heuristic really is faster, and it really does cost you.
    """
    rng = np.random.default_rng(seed)
    grid = empty_room(height, width)
    for _ in range(n_blocks):
        r = int(rng.integers(2, height - 5))
        c = int(rng.integers(2, width - 5))
        h = int(rng.integers(1, 4))
        w = int(rng.integers(1, 4))
        grid[r:r + h, c:c + w] = True
    # Keep the corners clear so start and goal are always free.
    grid[1:4, 1:4] = False
    grid[-4:-1, -4:-1] = False
    return grid


def steps_vs_cost(height: int = 30, width: int = 30) -> np.ndarray:
    """Clutter chosen so BFS is visibly suboptimal on an eight-connected grid.

    On this map BFS returns a path of 30 steps measuring 39.94, and Dijkstra
    returns one of 32 steps measuring 39.46. BFS wins on the thing it optimises,
    step count, and loses on the thing you actually care about, distance. It
    happens because a diagonal step counts as one move to BFS and costs sqrt(2)
    in reality, so BFS will happily trade two straight moves for one diagonal.

    Most random maps hide this, because the two answers tie. Out of 200 seeds
    only four separate them, which is the real warning: BFS on a diagonal grid
    is usually right, and you will not notice the day it is not.
    """
    return cluttered(height, width, seed=0)


def sealed_goal(height: int = 30, width: int = 30) -> np.ndarray:
    """The goal is walled into its corner. No path exists.

    Every planner must terminate and report failure. One that hangs, or returns
    a path not reaching the goal, fails this map. Handling "unreachable"
    correctly is not an edge case; it is what a real robot needs the moment a
    door closes behind it.
    """
    grid = empty_room(height, width)
    grid[height - 8, width - 8:] = True
    grid[height - 8:, width - 8] = True
    return grid


MAPS = {
    "empty_room": empty_room,
    "corridors": corridors,
    "greedy_trap": greedy_trap,
    "cluttered": cluttered,
    "steps_vs_cost": steps_vs_cost,
    "sealed_goal": sealed_goal,
}

# Maps on which a path exists. sealed_goal is deliberately not in here.
SOLVABLE = tuple(k for k in MAPS if k != "sealed_goal")


def path_length(path: list[tuple[int, int]], diagonal_cost: float = 2 ** 0.5) -> float:
    """Length in cell units, counting a diagonal step as sqrt(2).

    Counting steps instead would make an eight-connected path look shorter than
    it is and every comparison against Dijkstra meaningless. This function is
    the reason `steps_vs_cost` has anything to show.
    """
    if len(path) < 2:
        return 0.0
    total = 0.0
    for (r0, c0), (r1, c1) in zip(path, path[1:]):
        total += diagonal_cost if (r0 != r1 and c0 != c1) else 1.0
    return total


def is_valid_path(grid: np.ndarray, path: list[tuple[int, int]],
                  start: tuple[int, int], goal: tuple[int, int]) -> bool:
    """Check a returned path really is walkable, connected, and ends at the goal.

    Worth running on your own output before you believe any number in the table.
    A planner with an off-by-one in its neighbour loop can return a path that
    teleports through a wall, and the picture still looks fine.
    """
    if not path:
        return False
    if path[0] != start or path[-1] != goal:
        return False
    for (r, c) in path:
        if not (0 <= r < grid.shape[0] and 0 <= c < grid.shape[1]):
            return False
        if grid[r, c]:
            return False
    for (r0, c0), (r1, c1) in zip(path, path[1:]):
        if max(abs(r1 - r0), abs(c1 - c0)) != 1:
            return False
        if (r0, c0) == (r1, c1):
            return False
    return True


def render(grid: np.ndarray, path=None, start=START, goal=GOAL) -> str:
    """ASCII picture of a map, optionally with a path drawn on it.

    Useful in a terminal and useful in a bug report. `#` blocked, `.` free,
    `o` path, `S` start, `G` goal.
    """
    chars = np.where(grid, "#", ".")
    for cell in (path or []):
        chars[cell] = "o"
    chars[start] = "S"
    chars[goal] = "G"
    return "\n".join("".join(row) for row in chars)


if __name__ == "__main__":
    for name, factory in MAPS.items():
        grid = factory()
        print(f"--- {name} ({grid.shape[0]}x{grid.shape[1]}, "
              f"{int(grid.sum())} blocked cells) ---")
        print(render(grid))
        print()
