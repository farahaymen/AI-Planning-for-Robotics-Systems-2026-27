"""
Optional Lab 4 extension: converting a ROS OccupancyGrid for grid search.

This is the file you edit. Fill in the TODO blocks and nothing else; GridInfo,
`occupancy_to_numpy` and the two coordinate conversions are given and the tests
depend on them staying that way.

    ARC_GRID_TOOLS=grid_tools_skeleton python3 -m pytest starters/lab04 -q

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
    # TODO 1: build a boolean array that is True where the value is at or above
    #         occupied_threshold.
    #
    # TODO 2: if unknown_is_obstacle, mark the UNKNOWN cells as well. UNKNOWN is
    #         -1, which is below the threshold, so the comparison on its own
    #         quietly reports unmapped space as free.
    raise NotImplementedError("TODO: implement to_binary_obstacle_map")


def inflate(obstacles: np.ndarray, radius_cells: int) -> np.ndarray:
    """Grow obstacles by radius_cells, the same idea as the Nav2 inflation layer.

    Inflating by the robot's circumscribed radius converts a robot-shaped
    collision problem into a point collision problem, which is why your
    point-based A* is usable at all on a real robot.
    """
    # TODO 3: for radius_cells <= 0 return a copy of `obstacles` rather than the
    #         array itself, so the caller cannot mutate the input through it.
    #
    # TODO 4: build the disc of offsets (dy, dx) with dy**2 + dx**2 <=
    #         radius_cells**2, then mark every obstacle cell shifted by each
    #         offset. np.nonzero gives you the obstacle rows and columns to
    #         shift.
    #
    # TODO 5: drop the shifted indices that fall outside the array. NumPy does
    #         not raise on a negative index, it wraps, so an obstacle near the
    #         left edge would reappear on the right one.
    raise NotImplementedError("TODO: implement inflate")


def path_length_m(path_cells: list[tuple[int, int]], info: GridInfo) -> float:
    """Metric length of a cell path, so it is comparable with Nav2's plan."""
    # TODO 6: a path of fewer than two cells has no length. Return 0.0.
    #
    # TODO 7: convert each cell with grid_to_world and sum the straight line
    #         distances between consecutive points. The result is in metres, so
    #         a diagonal step counts resolution * sqrt(2) and not one cell.
    raise NotImplementedError("TODO: implement path_length_m")
