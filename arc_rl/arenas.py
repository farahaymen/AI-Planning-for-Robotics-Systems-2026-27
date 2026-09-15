"""
Autonomous Robotics Course (ARC) - arena generation grammar.

The competition arena is unseen, but it is not arbitrary. Teams are given this
grammar, and every scored arena is drawn from it. That is the difference between
a benchmark and a surprise: students can reason about the distribution they must
generalise over, which is what a generalisation claim actually means.

The same generator produces the training scenarios used in Labs 8 to 10 and the
evaluation seeds used by arc_eval, so a single seed integer fully identifies an
arena for grading and dispute resolution.

Every generated arena is checked for reachability with a breadth first search on
a coarse occupancy grid. This is the same BFS the students implement in Lab 4,
reused here as a production validation step rather than a toy exercise.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np

from arc_rl.nav_core import ROBOT, Scenario


@dataclass(frozen=True)
class ArenaGrammar:
    """Published parameters of the arena distribution."""

    width_range: tuple[float, float] = (10.0, 14.0)
    height_range: tuple[float, float] = (10.0, 14.0)
    n_partitions: tuple[int, int] = (2, 4)
    doorway_width: tuple[float, float] = (0.9, 1.6)
    n_static: tuple[int, int] = (8, 16)
    static_radius: tuple[float, float] = (0.15, 0.45)
    n_dynamic: tuple[int, int] = (2, 3)
    dynamic_speed: tuple[float, float] = (0.20, 0.40)
    dynamic_radius: float = 0.25
    n_goals: int = 4
    min_goal_separation: float = 3.0
    min_clearance: float = 0.35
    clearance_margin: float = 0.15      # on top of the 0.22 m footprint radius
    grid_resolution: float = 0.10
    max_episode_steps: int = 2400          # 120 s of simulated time at 20 Hz


DEFAULT_GRAMMAR = ArenaGrammar()

# Training arenas are drawn from a deliberately easier grammar so that students
# see the generalisation gap rather than failing everywhere equally.
TRAINING_GRAMMAR = ArenaGrammar(
    width_range=(9.0, 11.0),
    height_range=(9.0, 11.0),
    n_partitions=(1, 2),
    n_static=(5, 9),
    n_dynamic=(1, 2),
    n_goals=1,
    max_episode_steps=900,
)


@dataclass
class OccupancyGrid:
    resolution: float
    width_cells: int
    height_cells: int
    data: np.ndarray = field(repr=False)

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        return int(x / self.resolution), int(y / self.resolution)


def rasterise(scenario: Scenario, resolution: float, inflation: float) -> OccupancyGrid:
    """Build an inflated occupancy grid from the analytic scenario description.

    Inflation by the robot footprint turns a geometric collision test into a
    point reachability test, which is exactly the trick Nav2 costmaps use and a
    useful thing for students to see stated plainly.
    """
    w, h = scenario.bounds
    nx, ny = int(np.ceil(w / resolution)), int(np.ceil(h / resolution))
    xs = (np.arange(nx) + 0.5) * resolution
    ys = (np.arange(ny) + 0.5) * resolution
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    occupied = np.zeros((nx, ny), dtype=bool)

    for (ax, ay, bx, by) in scenario.walls:
        ex, ey = bx - ax, by - ay
        L2 = max(ex * ex + ey * ey, 1e-12)
        u = np.clip(((gx - ax) * ex + (gy - ay) * ey) / L2, 0.0, 1.0)
        d = np.hypot(gx - (ax + u * ex), gy - (ay + u * ey))
        occupied |= d <= inflation

    for (cx, cy, r) in scenario.static_circles:
        occupied |= np.hypot(gx - cx, gy - cy) <= (r + inflation)

    return OccupancyGrid(resolution, nx, ny, occupied)


def bfs_reachable(grid: OccupancyGrid, start: tuple[int, int], goal: tuple[int, int]) -> bool:
    """Four connected BFS. Identical in spirit to the Lab 4 student implementation."""
    nx, ny = grid.width_cells, grid.height_cells
    sx, sy = start
    gx, gy = goal
    if not (0 <= sx < nx and 0 <= sy < ny and 0 <= gx < nx and 0 <= gy < ny):
        return False
    if grid.data[sx, sy] or grid.data[gx, gy]:
        return False
    seen = np.zeros_like(grid.data)
    seen[sx, sy] = True
    queue = deque([(sx, sy)])
    while queue:
        cx, cy = queue.popleft()
        if (cx, cy) == (gx, gy):
            return True
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ux, uy = cx + dx, cy + dy
            if 0 <= ux < nx and 0 <= uy < ny and not seen[ux, uy] and not grid.data[ux, uy]:
                seen[ux, uy] = True
                queue.append((ux, uy))
    return False


def _partition_walls(rng, w: float, h: float, grammar: ArenaGrammar):
    walls = []
    count = int(rng.integers(grammar.n_partitions[0], grammar.n_partitions[1] + 1))
    for _ in range(count):
        gap = float(rng.uniform(*grammar.doorway_width))
        if rng.random() < 0.5:
            x = float(rng.uniform(0.3 * w, 0.7 * w))
            door = float(rng.uniform(1.0, h - 1.0 - gap))
            walls.append((x, 0.0, x, door))
            walls.append((x, door + gap, x, h))
        else:
            y = float(rng.uniform(0.3 * h, 0.7 * h))
            door = float(rng.uniform(1.0, w - 1.0 - gap))
            walls.append((0.0, y, door, y))
            walls.append((door + gap, y, w, y))
    return walls


def sample_scenario(seed: int, grammar: ArenaGrammar = DEFAULT_GRAMMAR,
                    max_attempts: int = 40) -> Scenario:
    """Draw one arena from the grammar. The seed alone reproduces it exactly."""
    for attempt in range(max_attempts):
        rng = np.random.default_rng(seed * 1000 + attempt)
        w = float(rng.uniform(*grammar.width_range))
        h = float(rng.uniform(*grammar.height_range))

        walls = [(0, 0, w, 0), (w, 0, w, h), (w, h, 0, h), (0, h, 0, 0)]
        walls += _partition_walls(rng, w, h, grammar)

        n_static = int(rng.integers(grammar.n_static[0], grammar.n_static[1] + 1))
        statics = [
            (float(rng.uniform(0.8, w - 0.8)), float(rng.uniform(0.8, h - 0.8)),
             float(rng.uniform(*grammar.static_radius)))
            for _ in range(n_static)
        ]

        scenario = Scenario(
            name=f"arena_seed{seed:04d}",
            bounds=(w, h),
            walls=walls,
            static_circles=statics,
            max_episode_steps=grammar.max_episode_steps,
        )

        # Validate with a clearance MARGIN, not just the footprint. At footprint
        # plus 5 cm the check only proves a nearly point-sized robot can pass, so
        # two partitions falling close together can produce a route that is
        # technically reachable and unusable in practice. The margin guarantees
        # roughly 2 x (radius + margin) of navigable width along the route.
        inflation = ROBOT.footprint_radius + grammar.clearance_margin
        grid = rasterise(scenario, grammar.grid_resolution, inflation)
        free = np.argwhere(~grid.data)
        if free.shape[0] < 200:
            continue

        picks = free[rng.choice(free.shape[0], size=min(600, free.shape[0]), replace=False)]
        start_cell = tuple(int(v) for v in picks[0])
        chosen = [start_cell]
        for cell in picks[1:]:
            cell = (int(cell[0]), int(cell[1]))
            far_enough = all(
                np.hypot((cell[0] - c[0]), (cell[1] - c[1])) * grammar.grid_resolution
                >= grammar.min_goal_separation
                for c in chosen
            )
            if far_enough and bfs_reachable(grid, start_cell, cell):
                chosen.append(cell)
            if len(chosen) == grammar.n_goals + 1:
                break

        if len(chosen) < grammar.n_goals + 1:
            continue

        to_world = lambda c: ((c[0] + 0.5) * grammar.grid_resolution,
                              (c[1] + 0.5) * grammar.grid_resolution)
        sx, sy = to_world(chosen[0])
        scenario.start_pose = (sx, sy, float(rng.uniform(-np.pi, np.pi)))
        scenario.goals = [to_world(c) for c in chosen[1:]]

        n_dyn = int(rng.integers(grammar.n_dynamic[0], grammar.n_dynamic[1] + 1))
        dyn_cells = picks[rng.choice(picks.shape[0], size=n_dyn, replace=False)]
        scenario.dynamic_circles = [
            (*to_world((int(c[0]), int(c[1]))),
             float(rng.uniform(*grammar.dynamic_speed)),
             float(rng.uniform(-np.pi, np.pi)),
             grammar.dynamic_radius)
            for c in dyn_cells
        ]
        return scenario

    raise RuntimeError(
        f"Could not generate a valid arena for seed {seed} in {max_attempts} attempts. "
        "Loosen the grammar or raise max_attempts."
    )


def training_scenario(seed: int) -> Scenario:
    return sample_scenario(seed, TRAINING_GRAMMAR)


def competition_scenario(seed: int) -> Scenario:
    return sample_scenario(seed, DEFAULT_GRAMMAR)
