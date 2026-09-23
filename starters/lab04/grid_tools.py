"""
Optional Lab 4 extension: converting a ROS OccupancyGrid for grid search.

These helpers prepare an occupancy array for optional BFS, Dijkstra or A*
experiments. Nav2 uses its configured planner and costmaps; importing these
helpers does not replace that planner or reproduce every costmap layer.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

FREE, UNKNOWN, LETHAL = 0, -1, 100


@dataclass
class GridInfo:
    """The three numbers you need to move between world metres and grid cells."""

    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int


def occupancy_to_numpy(msg) -> tuple[np.ndarray, GridInfo]:
    """Convert nav_msgs/OccupancyGrid into a 2D array indexed [row, col].

    The message stores data in row major order starting at the origin corner, so
    the reshape is (height, width) and NOT (width, height). Getting this wrong
    produces a map that looks plausible but is transposed, and the symptom is a
    planner that refuses goals in apparently open space.
    """
    info = GridInfo(
        resolution=msg.info.resolution,
        origin_x=msg.info.origin.position.x,
        origin_y=msg.info.origin.position.y,
        width=msg.info.width,
        height=msg.info.height,
    )
    grid = np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width)
    return grid, info


def world_to_grid(x: float, y: float, info: GridInfo) -> tuple[int, int]:
    # Floor keeps a point just below the origin outside cell zero.
    col = int(np.floor((x - info.origin_x) / info.resolution))
    row = int(np.floor((y - info.origin_y) / info.resolution))
    return row, col


def grid_to_world(row: int, col: int, info: GridInfo) -> tuple[float, float]:
    x = info.origin_x + (col + 0.5) * info.resolution
    y = info.origin_y + (row + 0.5) * info.resolution
    return x, y


def to_binary_obstacle_map(grid: np.ndarray, occupied_threshold: int = 65,
                           unknown_is_obstacle: bool = True) -> np.ndarray:
    """Reduce the 0 to 100 occupancy values to a boolean obstacle mask.

    Whether unknown space counts as an obstacle is a real design decision, not a
    detail. Treating it as free lets the planner route optimistically through
    unmapped areas. Treating it as blocked is more conservative. Compare both
    in the optional grid-search extension. Nav2 behaviour depends on its
    planner and costmap configuration; inspect those before comparing routes.
    """
    obstacles = grid >= occupied_threshold
    if unknown_is_obstacle:
        obstacles |= grid == UNKNOWN
    return obstacles


def inflate(obstacles: np.ndarray, radius_cells: int) -> np.ndarray:
    """Grow obstacles by radius_cells, the same idea as the Nav2 inflation layer.

    Inflating by the robot's circumscribed radius converts a robot-shaped
    collision problem into a point collision problem, which is why your
    point-based A* is usable at all on a real robot.
    """
    if radius_cells <= 0:
        return obstacles.copy()
    inflated = obstacles.copy()
    ys, xs = np.mgrid[-radius_cells:radius_cells + 1, -radius_cells:radius_cells + 1]
    disc = (ys ** 2 + xs ** 2) <= radius_cells ** 2
    rows, cols = np.nonzero(obstacles)
    h, w = obstacles.shape
    for dy, dx in zip(ys[disc], xs[disc]):
        r, c = rows + dy, cols + dx
        keep = (r >= 0) & (r < h) & (c >= 0) & (c < w)
        inflated[r[keep], c[keep]] = True
    return inflated


def path_length_m(path_cells: list[tuple[int, int]], info: GridInfo) -> float:
    """Metric length of a cell path, so it is comparable with Nav2's plan."""
    if len(path_cells) < 2:
        return 0.0
    pts = np.array([grid_to_world(r, c, info) for r, c in path_cells])
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))
