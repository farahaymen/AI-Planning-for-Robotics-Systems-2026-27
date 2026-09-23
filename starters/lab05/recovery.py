"""
Lab 5: detecting failure and escalating recovery.

Nav2's behaviour tree already does this, in XML, and you will configure that
today. These two classes are the same logic in about eighty readable lines, so
that when you edit the tree you know what the nodes are actually doing.

Both are used again: the stuck detector defines "stuck" for the Project 2
competition rules, and the escalator is the pattern your mission node needs.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from enum import Enum


class StuckDetector:
    """Fires when the robot has not moved far enough over a time window.

    Deliberately based on displacement from the OLDEST pose in the window rather
    than on total distance travelled. A robot spinning on the spot or oscillating
    in a doorway covers plenty of distance and gets nowhere, and a total-distance
    test calls that healthy.
    """

    def __init__(self, window_s: float = 5.0, min_displacement_m: float = 0.15,
                 control_period: float = 0.05):
        self.min_displacement = min_displacement_m
        self.capacity = max(2, int(round(window_s / control_period)))
        self.history: deque[tuple[float, float]] = deque(maxlen=self.capacity)

    def update(self, x: float, y: float) -> bool:
        self.history.append((x, y))
        if len(self.history) < self.capacity:
            return False            # not enough history to judge yet
        ox, oy = self.history[0]
        return math.hypot(x - ox, y - oy) < self.min_displacement

    def reset(self) -> None:
        self.history.clear()

    @property
    def displacement(self) -> float:
        if len(self.history) < 2:
            return 0.0
        ox, oy = self.history[0]
        x, y = self.history[-1]
        return math.hypot(x - ox, y - oy)


class Recovery(Enum):
    NONE = "none"
    CLEAR_COSTMAP = "clear_costmap"
    SPIN = "spin"
    BACK_UP = "back_up"
    WAIT = "wait"
    ABORT = "abort"


@dataclass
class RecoveryEscalator:
    """Try cheap recoveries first, escalate, then give up.

    The ordering is not arbitrary. Clearing the costmap is free and fixes the
    most common cause, which is a stale obstacle that has already moved away.
    Spinning costs a few seconds and refreshes the sensor's view. Backing up
    physically moves the robot and is the first action that can make things
    worse. Waiting is for dynamic obstacles that will leave on their own.

    Aborting is a legitimate outcome and is better engineering than a robot that
    thrashes forever. A system that reports it cannot reach a goal and moves to
    the next one scores better in Project 1 than one that retries indefinitely.
    """

    sequence: list[Recovery] = field(default_factory=lambda: [
        Recovery.CLEAR_COSTMAP, Recovery.SPIN, Recovery.WAIT, Recovery.BACK_UP])
    max_cycles: int = 2
    index: int = 0
    cycles: int = 0
    history: list[Recovery] = field(default_factory=list)

    def next_action(self) -> Recovery:
        if self.cycles >= self.max_cycles:
            self.history.append(Recovery.ABORT)
            return Recovery.ABORT
        action = self.sequence[self.index]
        self.index += 1
        if self.index >= len(self.sequence):
            self.index = 0
            self.cycles += 1
        self.history.append(action)
        return action

    def on_progress(self) -> None:
        """Call when the robot starts moving again. Resets the escalation.

        Without this a robot that recovers successfully three times over a long
        mission still aborts, because the counter never cleared.
        """
        self.index = 0
        self.cycles = 0

    @property
    def attempts(self) -> int:
        return len([h for h in self.history if h is not Recovery.ABORT])

    @property
    def exhausted(self) -> bool:
        return self.cycles >= self.max_cycles
