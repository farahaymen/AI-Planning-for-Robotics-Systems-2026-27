"""
Two reactive behaviours: emergency braking, and follow the gap.

Reactive means no map, no plan, no memory. Scan in, velocity out, every cycle.
That sounds primitive and it is the most important layer in the robot, for one
reason: **it is the only part that still works when everything above it is
wrong.**

A planner can be confused. A localiser can be lost. A behaviour tree can be
stuck in a branch nobody tested. None of that matters to a controller that only
ever asks "is there something in front of me, and how fast am I approaching
it". That independence is the whole design argument, and it is why the emergency
stop must NOT share code, state or assumptions with the navigation stack.

**Emergency braking** is the safety layer. You will write it in Lab 3 and
configure its production equivalent, Nav2's Collision Monitor, in Lab 7.

**Follow the gap** is a complete navigator with no map at all. It is also the
classical baseline that the reinforcement learning policies in Lab 9 are
measured against, so writing it here means that comparison is against something
you understand rather than a number you were handed.

REFERENCE IMPLEMENTATION. The version you fill in is `reactive_skeleton.py`.
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

    For a beam at angle `a`, the range shrinks at a rate of `v * cos(a)` as the
    robot drives forwards. Time to collision for that beam is therefore

        ttc = (range - stopping_margin) / (v * cos(a))

    and only for beams where the range is actually closing. The answer is the
    smallest such time over every beam in the robot's corridor.

    **Why time and not distance.** The obvious emergency stop is "brake if
    anything is nearer than 0.4 m", and it is wrong in both directions at once.
    At 0.05 m/s that stops the robot from ever docking or passing through a
    doorway. At 0.5 m/s it triggers far too late, because the robot travels
    0.4 m in less than a second and cannot stop in that distance. Distance is
    the wrong quantity; the question is always how much TIME you have.

    Returns `inf` when nothing is closing, which is the correct answer for a
    stationary robot or a clear corridor.
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

    The shape of this matters as much as the arithmetic. It takes a controller
    and returns a controller, so it sits BETWEEN the navigation stack and the
    wheels and cannot be bypassed by a planner having a bad day. It can veto and
    it can never command motion, which is the property that makes it a safety
    layer rather than another behaviour.

    Nav2's Collision Monitor is the same idea with more configuration: it
    watches the velocity command, projects the footprint forward along it, and
    slows or stops if that projection hits anything. You configure it in Lab 7.
    This is its thirty line ancestor.

    `threshold_s` of 1.2 is a starting point, not a truth. Exercise 3.5 asks you
    to find the value where the robot neither stops in open doorways nor hits
    the wall, and to say what it depends on.
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
                # Stop translating. Keep the turn: rotating on the spot is how
                # the robot gets out of the situation, and freezing both would
                # leave it stuck against the wall forever.
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
