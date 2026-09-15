"""
Lab 4: occupancy grid mapping with an inverse sensor model.

This is the first algorithm in the course that students implement in full rather
than configure. It runs against the rosbag recorded in Lab 3, which means it is
deterministic: the same bag and the same code produce the same map every time.
That is what makes it fair to grade and possible to debug.

Nothing here depends on ROS. The whole module is NumPy, so it works in graphics
Tier C on a machine where Gazebo will not start at all.

Course: Autonomous Robotics with ROS 2
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class MappingParams:
    """Inverse sensor model parameters.

    The two log odds values are the entire model. l_occ is how much evidence one
    beam endpoint gives that a cell is occupied; l_free is how much evidence a
    beam passing through gives that a cell is empty. Their ratio decides whether
    the map trusts hits or misses more.

    l_free is smaller in magnitude than l_occ on purpose. A single beam sweeps
    through many free cells and terminates on only one, so weighting them equally
    would let free space evidence overwhelm every obstacle.
    """

    resolution: float = 0.05        # metres per cell
    l_occ: float = 0.85             # log odds added at a hit
    l_free: float = -0.40           # log odds added along the ray
    l_min: float = -5.0             # clamp, see clamping note below
    l_max: float = 5.0
    l_prior: float = 0.0            # log odds of 0.5, meaning unknown
    max_range: float = 12.0
    min_range: float = 0.12
    hit_tolerance: float = 0.10     # a return within this of max_range is a miss


class OccupancyMap:
    """A log odds occupancy grid.

    Probabilities are stored as log odds rather than probabilities because the
    Bayesian update then becomes addition. Multiplying many small probabilities
    together underflows; adding log odds does not, and it is also faster.

        l = log( p / (1 - p) )        p = 1 / (1 + exp(-l))
    """

    def __init__(self, width_m: float, height_m: float,
                 origin: tuple[float, float] = (0.0, 0.0),
                 params: MappingParams = MappingParams()):
        self.params = params
        self.origin_x, self.origin_y = origin
        self.width = int(math.ceil(width_m / params.resolution))
        self.height = int(math.ceil(height_m / params.resolution))
        self.log_odds = np.full((self.height, self.width), params.l_prior, dtype=np.float32)

    # -- coordinate handling -------------------------------------------------

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        col = int((x - self.origin_x) / self.params.resolution)
        row = int((y - self.origin_y) / self.params.resolution)
        return row, col

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.height and 0 <= col < self.width

    # -- the update ----------------------------------------------------------

    def integrate_scan(self, pose: tuple[float, float, float],
                       ranges: np.ndarray, angles: np.ndarray) -> int:
        """Fold one laser scan into the map. Returns the number of cells touched.

        pose is (x, y, theta) of the SENSOR, not the robot base. Passing the base
        pose is the most common error in this exercise and it produces a map that
        is subtly shifted rather than obviously wrong, which makes it hard to
        spot. Transform the pose before calling this.
        """
        x, y, theta = pose
        p = self.params
        touched = 0

        ranges = np.asarray(ranges, dtype=np.float64)
        angles = np.asarray(angles, dtype=np.float64)

        for r, a in zip(ranges, angles):
            if not np.isfinite(r) or r < p.min_range:
                continue

            # A return at (or beyond) the maximum range is a MISS, not a hit.
            # It means the beam found nothing, so the cells along it are free and
            # the endpoint tells you nothing. Marking it occupied puts a ring of
            # phantom obstacles at the sensor's range limit, which is the single
            # most recognisable bug in a student occupancy map.
            is_hit = r < (p.max_range - p.hit_tolerance)
            reach = min(r, p.max_range)

            world_angle = theta + a
            end_x = x + reach * math.cos(world_angle)
            end_y = y + reach * math.sin(world_angle)

            start = self.world_to_cell(x, y)
            end = self.world_to_cell(end_x, end_y)

            cells = bresenham(start, end)
            for row, col in cells[:-1]:
                if self.in_bounds(row, col):
                    self._update_cell(row, col, p.l_free)
                    touched += 1

            if is_hit:
                row, col = cells[-1]
                if self.in_bounds(row, col):
                    self._update_cell(row, col, p.l_occ)
                    touched += 1
            else:
                row, col = cells[-1]
                if self.in_bounds(row, col):
                    self._update_cell(row, col, p.l_free)
                    touched += 1

        return touched

    def _update_cell(self, row: int, col: int, delta: float) -> None:
        # Clamping matters more than it looks. Without it a cell observed a
        # thousand times reaches a log odds of 850 and can never be revised, so
        # a chair that moves stays in the map forever. Clamping bounds how
        # confident the map is allowed to become and keeps it correctable.
        value = self.log_odds[row, col] + delta
        self.log_odds[row, col] = min(max(value, self.params.l_min), self.params.l_max)

    # -- output --------------------------------------------------------------

    def probability(self) -> np.ndarray:
        """Convert log odds back to probabilities in [0, 1]."""
        return 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))

    def to_ros_occupancy(self, occupied_threshold: float = 0.65,
                         free_threshold: float = 0.25) -> np.ndarray:
        """Produce the 0 to 100 and -1 encoding that ROS map files use.

        The two thresholds create a deliberate band of cells that are neither
        confidently free nor confidently occupied and are reported as unknown.
        Collapsing that band to a single threshold produces a map that looks
        decisive and is wrong at every boundary.
        """
        p = self.probability()
        out = np.full(p.shape, -1, dtype=np.int8)
        out[p >= occupied_threshold] = 100
        out[p <= free_threshold] = 0
        return out

    def coverage(self) -> float:
        """Fraction of cells that are no longer at the prior. A progress metric."""
        return float(np.mean(self.log_odds != self.params.l_prior))


def bresenham(start: tuple[int, int], end: tuple[int, int]) -> list[tuple[int, int]]:
    """Integer line from start to end inclusive, using Bresenham's algorithm.

    Supplied rather than set as an exercise because tracing a line is not the
    learning objective and getting the octant handling right consumes time that
    should go on the sensor model. Read it, do not rewrite it.
    """
    (r0, c0), (r1, c1) = start, end
    cells = []
    dr, dc = abs(r1 - r0), abs(c1 - c0)
    sr = 1 if r1 > r0 else -1
    sc = 1 if c1 > c0 else -1
    err = dr - dc
    r, c = r0, c0
    while True:
        cells.append((r, c))
        if (r, c) == (r1, c1):
            return cells
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc


def sensor_pose_from_base(base_pose: tuple[float, float, float],
                          offset_x: float, offset_y: float = 0.0
                          ) -> tuple[float, float, float]:
    """Transform a base pose into the sensor pose.

    On the ARC robot the LiDAR sits 0.10 m forward of base_link. Skipping this
    shifts the whole map by 10 cm in the robot's heading direction, which looks
    like a small blur rather than an obvious error.
    """
    x, y, theta = base_pose
    return (x + offset_x * math.cos(theta) - offset_y * math.sin(theta),
            y + offset_x * math.sin(theta) + offset_y * math.cos(theta),
            theta)
