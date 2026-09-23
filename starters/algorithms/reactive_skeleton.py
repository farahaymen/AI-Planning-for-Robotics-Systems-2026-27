"""
Lab 3, Exercise 3.5 and Lab 4, Exercise 4.8. The reactive behaviours.

This is the file you edit.

    ARC_REACTIVE=reactive_skeleton python3 -m pytest tests/test_reactive.py -x -q

Reactive means no map, no plan, no memory. Scan in, velocity out, every cycle.

That sounds primitive and it is the most important layer in the robot, because
it is the only part that still works when everything above it is wrong. A
planner can be confused, a localiser can be lost, a behaviour tree can be stuck
in a branch nobody tested. None of that reaches a controller whose only question
is "is something in front of me, and how fast am I closing on it".

That independence is the design argument, and it is why the emergency stop must
not share code, state or assumptions with the navigation stack.
"""

from __future__ import annotations

import math

import numpy as np

try:
    from .motion import ROBOT
except ImportError:  # Standalone exercise.
    from motion import ROBOT
from sensing import LIDAR, LidarSpec, beam_angles, in_corridor


# ---------------------------------------------------------------------------
# Exercise 3.5a. Time to collision
# ---------------------------------------------------------------------------

def time_to_collision(ranges: np.ndarray, v: float,
                      spec: LidarSpec = LIDAR,
                      half_width: float = 0.25) -> float:
    """Seconds until impact if the robot keeps its current forward speed.

    A beam at angle `a` sees its range shrink at `v * cos(a)` as the robot
    drives forwards, so for that beam

        ttc = (range - footprint_radius) / (v * cos(a))

    and the answer is the smallest such time over the beams in the corridor.

    **Why time rather than distance.** The obvious emergency stop is "brake if
    anything is nearer than 0.4 m". It is wrong in both directions at once. At
    0.05 m/s it stops the robot from ever docking or passing through a doorway.
    At 0.5 m/s it triggers far too late, because the robot covers 0.4 m in under
    a second. Distance is the wrong quantity. The question is how much TIME.
    """
    # TODO 1: a robot that is stopped or reversing cannot drive into anything
    #         ahead of it. Return math.inf for v <= 0.
    #
    # TODO 2: use in_corridor(ranges, half_width, spec) to select the beams that
    #         are actually in the robot's way. If none are, return math.inf.
    #
    # TODO 3: the closing rate for each selected beam is v * cos(angle). Keep
    #         only the beams where that is positive; a beam pointing sideways or
    #         backwards is not closing and dividing by it is meaningless.
    #
    # TODO 4: subtract ROBOT.footprint_radius from each range before dividing,
    #         and clamp at zero. The LiDAR sits INSIDE the robot, so a return at
    #         0.22 m is already touching. Measuring to the centre tells you that
    #         you have time left when you have already hit the wall.
    #
    #         Return the smallest (gap / closing_rate) as a float.
    raise NotImplementedError("TODO: implement time_to_collision")


# ---------------------------------------------------------------------------
# Exercise 3.5b. The safety layer
# ---------------------------------------------------------------------------

def make_emergency_brake(threshold_s: float = 1.2, half_width: float = 0.25,
                         spec: LidarSpec = LIDAR):
    """Wrap any controller in a layer that can only ever slow it down.

    The shape matters as much as the arithmetic. This takes a controller and
    returns a controller, so it sits BETWEEN the navigation stack and the wheels
    and cannot be bypassed by a planner having a bad day.

    It may veto. It may never command motion. That is the property that makes it
    a safety layer rather than just another behaviour, and the tests check it.

    Nav2's Collision Monitor, which you configure in Lab 7, is the same idea
    with more configuration. This is its thirty line ancestor.
    """
    def wrap(controller):
        def guarded(*args, **kwargs):
            out = controller(*args, **kwargs)
            v, w = out[0], out[1]
            rest = out[2:]

            ranges = kwargs.get("ranges")
            if ranges is None:
                raise ValueError("the guarded controller needs ranges=")

            # TODO 5: compute the time to collision for the commanded v.
            #         If it is below threshold_s, return (0.0, w, *rest).
            #
            #         Keep the TURN. Zeroing both leaves a robot pinned against
            #         a wall with no way out, and rotating on the spot is exactly
            #         how it escapes.
            #
            #         Otherwise return the controller's output unchanged.
            raise NotImplementedError("TODO: implement the brake")
        return guarded
    return wrap


# ---------------------------------------------------------------------------
# Exercise 4.8. Follow the gap
# ---------------------------------------------------------------------------

def widest_gap(ranges: np.ndarray, threshold: float,
               spec: LidarSpec = LIDAR) -> tuple[int, int]:
    """The longest run of consecutive beams reading beyond `threshold`.

    Returns (start, end) inclusive, or (-1, -1) if there is no such run.
    """
    # TODO 6: build a boolean array of "free" beams. A beam is free if its range
    #         exceeds the threshold OR is infinite, because infinite means the
    #         beam hit nothing at all.
    #
    # TODO 7: find the longest consecutive run of True and return its first and
    #         last indices. Return (-1, -1) if there is none.
    raise NotImplementedError("TODO: implement widest_gap")


def safety_bubble(ranges: np.ndarray, radius: float = 0.35,
                  spec: LidarSpec = LIDAR) -> np.ndarray:
    """Zero the beams around the closest return.

    This is the step that makes follow the gap work and the step everyone omits
    the first time.

    Without it the algorithm finds free beams immediately beside a very close
    obstacle and steers into them, because a single beam says nothing about
    whether a 0.44 m robot fits. Blanking a bubble around the nearest point is
    how the robot's WIDTH gets represented in a signal that has no notion of it.
    """
    out = np.array(ranges, dtype=float)
    finite = np.isfinite(out)
    if not finite.any():
        return out

    nearest = int(np.nanargmin(np.where(finite, out, np.inf)))
    d = out[nearest]
    if not np.isfinite(d) or d <= 0.0:
        return out

    # TODO 8: the bubble's half angle at distance d is atan2(radius, d). Convert
    #         that to a number of BEAMS using the angular step, which is
    #         (angle_max - angle_min) / n_beams.
    #
    # TODO 9: zero the beams within that many either side of `nearest`.
    #
    #         Index MODULO n_beams. The scan is circular: beam 359 is adjacent
    #         to beam 0. Slicing instead of wrapping silently halves the bubble
    #         whenever the nearest obstacle is behind the robot, and there is a
    #         test for exactly that.
    raise NotImplementedError("TODO: implement safety_bubble")


def make_gap_follower(bubble: float = 0.35, free_threshold: float = 1.0,
                      speed: float = 0.35, turn_gain: float = 1.2,
                      spec: LidarSpec = LIDAR):
    """Drive towards the middle of the widest free gap. No map, no plan.

    Four steps, and the order matters:

    1. Blank a safety bubble around the nearest obstacle.
    2. Ignore everything behind the robot.
    3. Find the widest run of free beams and aim at its middle.
    4. Slow down in proportion to how far you have to turn.

    It looks remarkably competent in corridors and rooms, and it has no memory
    at all, so it will drive into a dead end, turn round, drive out, and drive
    straight back in. That failure is not a bug to fix here. It is the argument
    for the mapping and planning you did in Lab 4, and it is worth seeing rather
    than being told about.
    """
    angles = beam_angles(spec)

    def controller(pose=None, path=None, index: int = 0, *, ranges: np.ndarray):
        # TODO 10: apply the safety bubble.
        #
        # TODO 11: zero every beam at |angle| >= pi/2. Searching the full circle
        #          lets the widest gap be the empty room BEHIND the robot, so it
        #          turns round and drives back the way it came, repeatedly.
        #
        # TODO 12: find the widest gap. If there is none, the robot is boxed in:
        #          return v = 0 and a turn towards whichever side has more room,
        #          rather than stopping dead and needing rescuing by hand.
        #
        # TODO 13: aim at the middle beam of the gap. Set w = turn_gain * angle,
        #          and scale v down as |angle| grows, with a floor.
        #          Clip both to the robot's limits and return (v, w, index).
        raise NotImplementedError("TODO: implement make_gap_follower")

    return controller
