"""
Lab 3: occupancy grid mapping with an inverse sensor model.

This is the file you edit. Fill in the TODO blocks in `integrate_scan` and
`_update_cell` and nothing else; the data structures, the Bresenham line tracer
and the coordinate handling are given and the tests depend on them staying that
way.

    ARC_OCCUPANCY=occupancy_skeleton python3 -m pytest starters/lab03 -q

The tests are the specification. Read `tests/test_occupancy.py` before you
start writing; it names each case the sensor model has to get right.

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

    Under the chosen independent-measurement model, odds accumulate by
    multiplication and log odds accumulate by addition. This representation
    makes the hit/free evidence increments explicit; updates are also clamped.

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
        col = math.floor((x - self.origin_x) / self.params.resolution)
        row = math.floor((y - self.origin_y) / self.params.resolution)
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
            # TODO 1: skip this return if it is not finite or is shorter than
            #         p.min_range. Neither carries any information about a cell.
            #
            # TODO 2: decide whether the beam hit something. It is a hit only if
            #         r < p.max_range - p.hit_tolerance. A return at the range
            #         limit means the beam found nothing, so it is a miss;
            #         marking its endpoint occupied draws a ring of phantom
            #         obstacles at the sensor's limit. Trace out to
            #         min(r, p.max_range) either way.
            #
            # TODO 3: compute the endpoint in world coordinates from the angle
            #         theta + a and the traced range, convert the sensor position
            #         and the endpoint with self.world_to_cell, and get the cells
            #         between them from bresenham(start, end).
            #
            # TODO 4: apply p.l_free to every cell except the last with
            #         self._update_cell, and add each one to `touched`. Skip
            #         cells that fail self.in_bounds: a negative index does not
            #         raise, it wraps round to the opposite edge of the map.
            #
            # TODO 5: update the last cell as well, with p.l_occ if the beam hit
            #         something and p.l_free if it did not. Same bounds check,
            #         same count.
            raise NotImplementedError("TODO: implement integrate_scan")

        return touched

    def _update_cell(self, row: int, col: int, delta: float) -> None:
        # TODO 6: add `delta` to self.log_odds[row, col] and clamp the result to
        #         [self.params.l_min, self.params.l_max]. Without the clamp a
        #         cell seen often enough can never be revised, so a chair that
        #         gets moved stays in the map forever.
        raise NotImplementedError("TODO: implement _update_cell")

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
