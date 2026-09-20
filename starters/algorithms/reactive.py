"""
Two reactive behaviours: emergency braking and follow the gap.

Reactive means no map, no plan and no memory. Scan in, velocity out, every
cycle. It keeps working when the planner is confused or the localiser is lost,
which is why the emergency stop must not share code or state with the
navigation stack.

Emergency braking is the safety layer. Its production equivalent is Nav2's
Collision Monitor, configured in Lab 7.

Follow the gap navigates with no map. It is also the classical baseline the
learned policies in Lab 9 are measured against.

Reference implementation. The version you fill in is `reactive_skeleton.py`.
"""

from __future__ import annotations

import math

import numpy as np

from motion import ROBOT
from sensing import LIDAR, LidarSpec, beam_angles, in_corridor


# ---------------------------------------------------------------------------
# Emergency braking
# ---------------------------------------------------------------------------

def time_to_collision(ranges: np.ndarray, v: float,
                      spec: LidarSpec = LIDAR,
                      half_width: float = 0.25) -> float:
    """Seconds until impact if the robot keeps its current forward speed.

    For a beam at angle `a` the range closes at `v * cos(a)`, so

        ttc = (range - stopping_margin) / (v * cos(a))

    counted only over beams that are actually closing, in the robot's corridor.

    A fixed distance threshold does not work here. Brake below 0.4 m and at
    0.05 m/s the robot can never pass through a doorway, while at 0.5 m/s it
    covers 0.4 m in under a second and cannot stop in time.

    Returns `inf` when nothing is closing.
    """
    if v <= 0.0:
        return math.inf                      # reversing and stopping are safe here
    angles = beam_angles(spec)
    mask = in_corridor(ranges, half_width, spec)
    if not mask.any():
        return math.inf

    closing = v * np.cos(angles[mask])       # rate at which each range shrinks
    usable = closing > 1e-6
    if not usable.any():
        return math.inf

    # Measure to the robot's skin, not to its centre. The LiDAR sits inside the
    # footprint, so a return at 0.25 m is already touching a 0.22 m robot.
    gap = np.maximum(ranges[mask][usable] - ROBOT.footprint_radius, 0.0)
    return float(np.min(gap / closing[usable]))


def make_emergency_brake(threshold_s: float = 1.2, half_width: float = 0.25,
                         spec: LidarSpec = LIDAR):
    """Wrap any controller in a safety layer that can only ever slow it down.

    Takes a controller and returns a controller, so it sits between the
    navigation stack and the wheels. It can veto motion and can never command
    it, which is what makes it a safety layer rather than another behaviour.

    Nav2's Collision Monitor does the same job with more configuration.

    `threshold_s` of 1.2 is a starting point. Exercise 3.5 asks you to find the
    value where the robot neither stops in open doorways nor hits the wall.
    """
    def wrap(controller):
        def guarded(*args, **kwargs):
            out = controller(*args, **kwargs)
            v, w = out[0], out[1]
            rest = out[2:]

            ranges = kwargs.get("ranges")
            if ranges is None:
                raise ValueError("the guarded controller needs ranges=")

            ttc = time_to_collision(ranges, v, spec, half_width)
            if ttc < threshold_s:
                # Stop translating but keep the turn. Rotating on the spot is
                # how the robot gets out; freezing both leaves it stuck.
                return (0.0, w, *rest)
            return out
        return guarded
    return wrap


# ---------------------------------------------------------------------------
# Follow the gap
# ---------------------------------------------------------------------------

def widest_gap(ranges: np.ndarray, threshold: float,
               spec: LidarSpec = LIDAR) -> tuple[int, int]:
    """The longest run of consecutive beams reading further than `threshold`.

    Returns (start, end) as an inclusive index range, or (-1, -1) if there is
    none. Infinite returns count as free, because nothing was hit.
    """
    free = (ranges > threshold) | ~np.isfinite(ranges)
    best = (-1, -1, 0)
    i = 0
    n = len(free)
    while i < n:
        if not free[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and free[j + 1]:
            j += 1
        if (j - i + 1) > best[2]:
            best = (i, j, j - i + 1)
        i = j + 1
    return best[0], best[1]


def safety_bubble(ranges: np.ndarray, radius: float = 0.35,
                  spec: LidarSpec = LIDAR) -> np.ndarray:
    """Zero out the beams around the closest return.

    This is the step that makes follow the gap work, and the step everybody
    leaves out first time.

    Without it the algorithm sees a gap of free beams immediately beside a very
    close obstacle and steers into it, because a single beam of angular width
    says nothing about whether a 0.44 m robot fits. Blanking a bubble around the
    nearest point expresses the robot's WIDTH in a representation that otherwise
    has no notion of it.

    The bubble is wider in beams when the obstacle is nearer, which is just
    trigonometry: at 0.3 m a 0.35 m radius subtends a much larger angle than it
    does at 3 m.
    """
    out = np.array(ranges, dtype=float)
    finite = np.isfinite(out)
    if not finite.any():
        return out

    nearest = int(np.nanargmin(np.where(finite, out, np.inf)))
    d = out[nearest]
    if not np.isfinite(d) or d <= 0.0:
        return out

    # Half angle subtended by the bubble at that distance.
    half = math.atan2(radius, max(d, 1e-3))
    step = (spec.angle_max - spec.angle_min) / spec.n_beams
    width = max(1, int(half / step))

    # The scan is CIRCULAR: beam 359 is adjacent to beam 0. Clipping the bubble
    # at the array ends instead of wrapping silently halves it whenever the
    # nearest obstacle is behind the robot, and the robot then treats an
    # obstacle it is about to reverse into as half blanked. Index modulo n.
    idx = (np.arange(nearest - width, nearest + width + 1) % spec.n_beams)
    out[idx] = 0.0
    return out


def make_gap_follower(bubble: float = 0.35, free_threshold: float = 1.0,
                      speed: float = 0.35, turn_gain: float = 1.2,
                      spec: LidarSpec = LIDAR):
    """Drive towards the middle of the widest free gap. No map, no plan.

    Four steps, in this order, and the order matters:

    1. Blank a safety bubble around the nearest obstacle, so the robot's width
       is represented.
    2. Find the widest run of beams reading beyond the free threshold.
    3. Aim at the middle of that run.
    4. Slow down in proportion to how far you have to turn.

    Run it and it looks remarkably competent in corridors and rooms. It has no
    memory at all, so it will also drive happily into a dead end, turn round,
    drive out, and drive straight back in. That failure is not a bug to fix
    here; it is the argument for the mapping and planning you did in Lab 4, and
    it is worth seeing rather than being told.
    """
    angles = beam_angles(spec)

    def controller(pose=None, path=None, index: int = 0, *, ranges: np.ndarray):
        clear = safety_bubble(ranges, bubble, spec)

        # Only consider beams the robot could actually drive towards. Searching
        # the full circle lets the widest gap be the empty room BEHIND the
        # robot, so it turns round and drives back the way it came, repeatedly.
        # It also sidesteps the wrap at +/- pi: a gap spanning that boundary
        # would be reported as two, which is only a problem directly astern.
        behind = np.abs(angles) >= (math.pi / 2)
        searchable = np.array(clear, dtype=float)
        searchable[behind] = 0.0

        start, end = widest_gap(searchable, free_threshold, spec)

        if start < 0:
            # Boxed in: nothing is beyond the threshold anywhere. Rotate towards
            # whichever side has more room rather than stopping dead.
            left = np.nanmean(np.where(np.isfinite(clear[angles > 0]),
                                       clear[angles > 0], spec.max_range))
            right = np.nanmean(np.where(np.isfinite(clear[angles < 0]),
                                        clear[angles < 0], spec.max_range))
            return 0.0, math.copysign(1.0, left - right) * 0.8, index

        target = angles[(start + end) // 2]
        w = turn_gain * target
        v = speed * max(0.15, 1.0 - abs(target) / (math.pi / 2))
        return (float(np.clip(v, 0.0, ROBOT.max_linear_velocity)),
                float(np.clip(w, -ROBOT.max_angular_velocity,
                              ROBOT.max_angular_velocity)),
                index)

    return controller
