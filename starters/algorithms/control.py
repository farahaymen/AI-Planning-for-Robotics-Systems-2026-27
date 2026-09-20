"""
Two path followers: PID heading control and Pure Pursuit.

A planner returns a list of points. A controller turns those points into wheel
velocities.

PID is an error controller: measure how wrong you are, multiply by gains, drive
the error to zero. Pure Pursuit is geometric: pick a point on the path ahead of
the robot, work out the arc that reaches it, drive that arc.

Nav2 ships Regulated Pure Pursuit, which is this with speed limits added.

Reference implementation. The version you fill in is `control_skeleton.py`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from motion import ROBOT, Pose, wrap_angle


# ---------------------------------------------------------------------------
# PID
# ---------------------------------------------------------------------------

@dataclass
class PID:
    """A proportional, integral, derivative controller for a scalar error.

    output = kp * e + ki * integral(e) + kd * de/dt

    kp reacts to how wrong you are. ki removes steady state error left by a
    constant disturbance. kd damps, and amplifies sensor noise, which is why
    many working controllers leave kd at zero.

    `integral_limit` is anti-windup. A robot held against a wall accumulates
    integral the whole time it is stuck, then spins hard working it off once it
    comes free.

    Set `wrap=True` for an angular error, so 179 to -179 degrees is a difference
    of two degrees rather than 358.
    """

    kp: float
    ki: float = 0.0
    kd: float = 0.0
    dt: float = ROBOT.control_period
    integral_limit: float = 1.0
    output_limit: float = ROBOT.max_angular_velocity
    wrap: bool = True

    _integral: float = field(default=0.0, init=False)
    _previous: float | None = field(default=None, init=False)

    def reset(self) -> None:
        self._integral = 0.0
        self._previous = None

    def __call__(self, error: float) -> float:
        if self.wrap:
            error = wrap_angle(error)

        self._integral = float(np.clip(self._integral + error * self.dt,
                                       -self.integral_limit, self.integral_limit))

        # On the first call there is no previous error, so the derivative is
        # undefined. Using zero rather than the error itself avoids a large
        # spurious kick on the first control cycle, which on a real robot is a
        # visible lurch.
        derivative = 0.0 if self._previous is None else (error - self._previous) / self.dt
        self._previous = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return float(np.clip(output, -self.output_limit, self.output_limit))


def make_heading_controller(kp: float = 2.5, ki: float = 0.0, kd: float = 0.15,
                            speed: float = 0.35, slow_on_turn: bool = True):
    """Drive towards a fixed point using PID on the heading error only.

    The simplest closed loop worth writing, and the one to tune first, because
    every intuition you build here transfers. Point the robot at the target,
    drive forwards, keep pointing.

    `slow_on_turn` is worth understanding rather than copying. Without it the
    robot drives at full speed while still turning hard, so it swings wide and
    arrives from the wrong side. Scaling the forward speed by how well aligned
    the robot is costs one line and removes the behaviour entirely. Nav2's
    Regulated Pure Pursuit does the same thing and calls it a regulation
    heuristic.
    """
    pid = PID(kp=kp, ki=ki, kd=kd)

    def controller(pose: Pose, target: tuple[float, float]):
        dx, dy = target[0] - pose.x, target[1] - pose.y
        desired = math.atan2(dy, dx)
        error = wrap_angle(desired - pose.theta)
        w = pid(error)
        v = speed
        if slow_on_turn:
            v *= max(0.1, 1.0 - abs(error) / (math.pi / 2))
        return v, w

    controller.pid = pid          # exposed so tests and figures can reset it
    return controller


# ---------------------------------------------------------------------------
# Pure Pursuit
# ---------------------------------------------------------------------------

def _to_robot_frame(pose: Pose, point: np.ndarray) -> tuple[float, float]:
    """Express a world point in the robot's own frame.

    Forward is +x, left is +y. This is the same rotation Lab 2 asks you to
    write by hand, and it is the reason Pure Pursuit is so short: once the
    lookahead point is in the robot's frame, the geometry is two lines.
    """
    dx = point[0] - pose.x
    dy = point[1] - pose.y
    c, s = math.cos(-pose.theta), math.sin(-pose.theta)
    return dx * c - dy * s, dx * s + dy * c


def find_lookahead(path: np.ndarray, pose: Pose, lookahead: float,
                   start_index: int = 0) -> tuple[np.ndarray, int]:
    """The first point on the path at least `lookahead` metres ahead.

    Searching forward from `start_index` rather than from the beginning is not
    an optimisation. It stops the robot latching onto an earlier part of the
    path that happens to pass nearby, which is what makes a naive Pure Pursuit
    drive a figure of eight on a path that loops back on itself.

    If no point is far enough away, the robot is near the end, so the last point
    is returned and the controller simply drives at it.
    """
    if len(path) == 0:
        raise ValueError("empty path")
    for i in range(start_index, len(path)):
        if math.hypot(path[i][0] - pose.x, path[i][1] - pose.y) >= lookahead:
            return path[i], i
    return path[-1], len(path) - 1


def make_pure_pursuit(lookahead: float = 0.45, speed: float = 0.35,
                      regulate: bool = True, min_speed: float = 0.08):
    """Pure Pursuit: steer along the arc that reaches a point ahead on the path.

    With the lookahead point at (x, y) in the robot's frame and lookahead
    distance L:

        curvature = 2 * y / L^2
        w         = v * curvature

    That comes from the circle through the origin, tangent to the current
    heading, passing through the lookahead point. No error signal and no gains.

    Lookahead distance is the only parameter. Short tracks tightly and
    oscillates, long cuts corners smoothly. See `fig_lookahead.png`.

    `regulate` slows down as curvature rises, which is what Nav2's Regulated
    Pure Pursuit adds. Without it the robot takes hairpins at full speed.
    """

    def controller(pose: Pose, path: np.ndarray, index: int = 0):
        target, index = find_lookahead(path, pose, lookahead, index)
        x, y = _to_robot_frame(pose, target)

        # A lookahead point behind the robot means it has overshot or is facing
        # away. Chasing it with the arc formula would drive forwards while
        # turning slowly; turning on the spot is both faster and safer.
        if x <= 0.0:
            return 0.0, math.copysign(ROBOT.max_angular_velocity * 0.6, y or 1.0), index

        distance_squared = x * x + y * y
        curvature = 2.0 * y / distance_squared
        v = speed
        if regulate:
            # Halve the speed once the turning radius drops below about a metre,
            # and keep going down from there, with a floor so it never stalls.
            v = max(min_speed, speed / (1.0 + 1.5 * abs(curvature)))
        w = v * curvature
        return v, w, index

    return controller


def follow_path(path: np.ndarray, pose: Pose, controller=None,
                goal_tolerance: float = 0.15, max_steps: int = 2000,
                dt: float = ROBOT.control_period,
                pose_noise: float = 0.0, seed: int = 0):
    """Drive a whole path and return the trajectory, for testing and figures.

    Stops within `goal_tolerance` of the final waypoint, or when steps run out,
    and returns the reason so a test can assert that it arrived.

    `pose_noise` models a noisy pose estimate, which is what a real controller
    acts on. Measured on the corner path, mean of fifteen seeds at 5 cm noise:

        lookahead    cross track error    control jitter
          0.10 m          0.017 m              0.46
          0.20 m          0.020 m              0.20
          0.45 m          0.035 m              0.11
          1.00 m          0.076 m              0.08

    Tracking error picks the shortest lookahead. Control jitter picks the
    longest, by nearly six to one. The trade is accuracy against smoothness,
    and which one to favour depends on the application.
    """
    from motion import step as motion_step

    controller = controller or make_pure_pursuit()
    rng = np.random.default_rng(seed)
    trajectory, index = [pose], 0
    goal = path[-1]

    for _ in range(max_steps):
        if math.hypot(goal[0] - pose.x, goal[1] - pose.y) <= goal_tolerance:
            return trajectory, True

        # The controller sees an estimate; the world moves the true pose. Keeping
        # those separate is the point. Fold the noise into `pose` instead and you
        # have modelled a robot that teleports, not one that is unsure.
        if pose_noise > 0.0:
            believed = Pose(pose.x + rng.normal(0.0, pose_noise),
                            pose.y + rng.normal(0.0, pose_noise),
                            wrap_angle(pose.theta + rng.normal(0.0, pose_noise)))
        else:
            believed = pose

        v, w, index = controller(believed, path, index)
        pose = motion_step(pose, v, w, dt)
        trajectory.append(pose)

    return trajectory, False


def cross_track_error(path: np.ndarray, trajectory) -> np.ndarray:
    """Distance from each pose to the nearest point ON the path.

    Note "on the path", not "on a path vertex". Measuring to the nearest
    waypoint sounds equivalent and is not: a path sampled every 5 cm reports up
    to 2.5 cm of error for a robot sitting exactly on the line, because the
    nearest vertex is up to half a sample away. That is pure measurement
    artefact, it is the same order as the differences you are trying to detect,
    and it would make every comparison in Exercise 4.7 meaningless.

    So this projects each pose onto every path SEGMENT and takes the smallest
    perpendicular distance, clamped to the segment ends.
    """
    if len(path) == 0 or len(trajectory) == 0:
        return np.zeros(0)
    points = np.array([[p.x, p.y] for p in trajectory])
    if len(path) == 1:
        return np.linalg.norm(points - path[0], axis=1)

    a = path[:-1]                      # segment starts, (S, 2)
    b = path[1:]                       # segment ends
    ab = b - a                         # segment vectors
    ab_len2 = np.einsum("ij,ij->i", ab, ab)
    ab_len2 = np.where(ab_len2 == 0.0, 1e-12, ab_len2)

    ap = points[:, None, :] - a[None, :, :]            # (N, S, 2)
    t = np.einsum("nsi,si->ns", ap, ab) / ab_len2      # projection parameter
    t = np.clip(t, 0.0, 1.0)                           # stay on the segment
    closest = a[None, :, :] + t[:, :, None] * ab[None, :, :]
    return np.linalg.norm(points[:, None, :] - closest, axis=2).min(axis=1)


def control_jitter(trajectory, dt: float = ROBOT.control_period) -> float:
    """How violently the commanded turn rate changes from cycle to cycle.

    The standard deviation of the change in angular velocity. Cross track error
    tells you whether the robot was on the path; this tells you what it cost to
    keep it there.

    Report both. On the corner path with a realistic 5 cm pose uncertainty, a
    0.10 m lookahead tracks about twice as well as a 1.00 m one and jitters
    nearly six times as hard. Optimising the first number alone produces a robot
    that follows the line beautifully and shakes itself to pieces doing it.
    """
    if len(trajectory) < 3:
        return 0.0
    theta = np.array([p.theta for p in trajectory])
    # Unwrap before differencing, or every pass through pi registers as a
    # colossal, entirely fictional jerk.
    omega = np.diff(np.unwrap(theta)) / dt
    return float(np.std(np.diff(omega)))
