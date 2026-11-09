"""Convert a Lab 6 greedy action to a one-cell motion without depending on ROS."""

from math import atan2, cos, hypot, sin

import numpy as np

from teaching.tabular import GridRobot, MOVES


DIRECTIONS = ('east', 'north', 'west', 'south')


class GridRun:
    """Use the fresh Gazebo spawn as (0, 0) in odom. Each cell is one metre."""

    def __init__(self, q):
        self.grid = GridRobot(slip=0)
        if (not isinstance(q, np.ndarray) or q.shape != (self.grid.n_states, 4)
                or q.dtype.kind not in 'fiu' or not np.isfinite(q).all()):
            raise ValueError('expected a finite numeric Lab 6 Q-table with shape (36, 4)')
        self.q = q
        self.cell = self.grid.start
        self.target = None
        self.action = None
        self.moves = 0
        self.visited = set()

    @property
    def done(self):
        return self.cell == self.grid.goal

    def command(self, x, y, yaw):
        if self.done:
            return 0., 0.
        if self.target is None:
            if self.cell in self.visited:
                raise RuntimeError(f'Greedy policy revisits {self.cell}; this policy loops on the grid.')
            self.visited.add(self.cell)
            self.action = int(self.q[self.grid.state(self.cell)].argmax())
            row, col = self.cell
            dr, dc = MOVES[self.action]
            nr, nc = row + dr, col + dc
            if not (0 <= nr < 6 and 0 <= nc < 6) or self.grid.grid[nr, nc]:
                raise RuntimeError(f'Policy chose {DIRECTIONS[self.action]} from {self.cell}, a blocked move.')
            self.target = (nr, nc)
            self.moves += 1

        # Grid uses (row, col); ROS odometry uses (x, y) = (col, row).
        row, col = self.target
        dx, dy = col - x, row - y
        distance = hypot(dx, dy)
        if distance <= .05:
            self.cell, self.target = self.target, None
            return 0., 0.
        angle = atan2(dy, dx) - yaw
        error = atan2(sin(angle), cos(angle))
        turn = max(-.9, min(.9, 1.8 * error))
        forward = min(.25, .8 * distance) if abs(error) < .15 else 0.
        return forward, turn
