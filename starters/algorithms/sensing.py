"""
A LiDAR, simulated on a grid map. Shared by the reactive, SLAM and exploration
exercises.

Why this exists rather than using Gazebo: a scan you can produce in a
microsecond lets you write a test that says "with a wall exactly here, the
controller must do exactly that". A scan that requires launching a simulator
does not, and the tests you would have written never get written.

The geometry matches the ARC robot's sensor, declared in `arc_bot.gazebo.xacro`:
360 beams over a full circle, 0.12 m to 12.0 m. Keeping those the same means a
controller tuned here behaves the same way on the robot.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from motion import Pose


@dataclass(frozen=True)
class LidarSpec:
    """Course constants, mirroring the simulated sensor."""

    n_beams: int = 360
    min_range: float = 0.12
    max_range: float = 12.0
    angle_min: float = -math.pi
    angle_max: float = math.pi


LIDAR = LidarSpec()


def beam_angles(spec: LidarSpec = LIDAR) -> np.ndarray:
    """The angle of each beam, in the robot's frame. 0 is straight ahead."""
    return np.linspace(spec.angle_min, spec.angle_max, spec.n_beams,
                       endpoint=False)


def raycast(grid: np.ndarray, pose: Pose, resolution: float = 0.05,
            spec: LidarSpec = LIDAR, noise: float = 0.0,
            seed: int | None = None) -> np.ndarray:
    """Cast every beam against an occupancy grid and return the ranges.

    `grid` is True where blocked, indexed [row, col], with row as y and column
    as x. The robot's pose is in metres.

    Beams that hit nothing within `max_range` return `inf`, exactly as the real
    sensor does through the ROS bridge. Do not quietly replace those with
    max_range: "nothing there" and "something at exactly 12 m" are different
    facts, and code that confuses them builds walls out of empty space. The
    occupancy mapping exercise in Lab 4 depends on the distinction.

    This steps along each ray at half a cell, which is simple and slightly
    conservative. A real sensor does not sample; it has a beam width, and the
    difference shows up as thin obstacles occasionally being missed. That is a
    fair thing for students to discover.
    """
    rng = np.random.default_rng(seed)
    angles = beam_angles(spec)
    h, w = grid.shape
    ranges = np.full(spec.n_beams, np.inf)
    stride = resolution * 0.5
    n_steps = int(spec.max_range / stride)

    for i, a in enumerate(angles):
        world = pose.theta + a
        dx, dy = math.cos(world) * stride, math.sin(world) * stride
        x, y = pose.x, pose.y
        for s in range(1, n_steps + 1):
            x += dx
            y += dy
            col = int(x / resolution)
            row = int(y / resolution)
            if not (0 <= row < h and 0 <= col < w):
                break                       # left the map: nothing to hit
            if grid[row, col]:
                r = s * stride
                if r >= spec.min_range:
                    ranges[i] = r
                else:
                    ranges[i] = spec.min_range
                break

    if noise > 0.0:
        finite = np.isfinite(ranges)
        ranges[finite] += rng.normal(0.0, noise, int(finite.sum()))
        ranges = np.clip(ranges, spec.min_range, None)
    return ranges


def scan_to_points(ranges: np.ndarray, spec: LidarSpec = LIDAR) -> np.ndarray:
    """Turn ranges into (x, y) points in the ROBOT's frame, dropping infinities.

    This is the form scan matching wants, and the form to plot when you are
    trying to see what the robot sees.
    """
    angles = beam_angles(spec)
    finite = np.isfinite(ranges)
    r = ranges[finite]
    a = angles[finite]
    return np.column_stack([r * np.cos(a), r * np.sin(a)])


def in_corridor(ranges: np.ndarray, half_width: float = 0.25,
                spec: LidarSpec = LIDAR) -> np.ndarray:
    """Which returns lie in the strip the robot would sweep driving forwards.

    A robot 0.44 m wide does not care about a wall 80 degrees off to the side,
    however close it is. Braking for everything produces a robot that cannot
    pass through a doorway, which is the classic first version of an emergency
    stop and is worse than useless, because people switch it off.

    The test is per BEAM and depends on where that beam actually hit. A return
    at angle `a` and range `r` lies `r * sin(a)` from the centre line, so it is
    in the way when that is less than half the robot's width. A fixed angular
    cone cannot express this: the angle that matters at 3 m is much narrower
    than the one that matters at 0.3 m.
    """
    angles = beam_angles(spec)
    with np.errstate(invalid="ignore"):
        lateral = np.abs(ranges * np.sin(angles))
    forward = np.cos(angles) > 0.0
    return forward & np.isfinite(ranges) & (lateral <= half_width)
