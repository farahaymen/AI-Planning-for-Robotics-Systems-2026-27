# The offline algorithms notebook

Pure Python. No ROS, no Gazebo, no simulator. Everything here runs on any
machine with Python, NumPy and Matplotlib, which is deliberate: the week the
simulator has a bad day should not be the week nobody learns A*.

This folder spans Labs 3 to 5 and it is what Project 1 is built from.

| File | Lab | What it is |
| --- | --- | --- |
| `motion.py` | 4 | the robot, the unicycle model, actuator lag |
| `gridmap.py` | 4 | the six maps, and the helpers that measure a path |
| `planners_skeleton.py` | 4 | **you edit.** BFS, DFS, Dijkstra, A*, nine TODOs |
| `control_skeleton.py` | 4 | **you edit.** PID and Pure Pursuit, eleven TODOs |
| `planners.py`, `control.py` | | reference implementations to compare against |
| `tests/` | | 126 tests, and the specification |
| `compare.py` | | every planner on every map |
| `figures.py`, `figures_control.py` | | regenerate every figure in the manuals |

Run the tests against your own work:

    ARC_PLANNERS=planners_skeleton python3 -m pytest tests/test_planners.py -x -q
    ARC_CONTROL=control_skeleton  python3 -m pytest tests/test_control.py  -x -q

## The controllers

A planner returns a list of cells. Something has to turn that into wheel
velocities, and without it you have a drawing rather than a robot. Project 1
needs both of these working.

**PID** is an error controller: measure how wrong you are, multiply by a gain,
drive the error to zero. **Pure Pursuit** is a geometric controller: pick a
point on the path ahead and drive the arc that reaches it. It computes no error
at all.

Three things in this folder are worth more than the code itself.

**The actuator lag matters.** `motion.Actuator` models motors taking time to
reach a commanded speed. Without it a proportional controller physically cannot
overshoot, every gain looks stable, and the derivative term appears to do
nothing. With a realistic 0.15 s lag, raising kp from 2.5 to 6.0 makes the robot
overshoot 6.1 percent and settle *later*, and adding kd at the same gain fixes
both. That is the entire argument for the derivative term and you cannot see it
in a model without inertia.

**Two metrics disagree, and one of them lies.** Sweep the Pure Pursuit lookahead
distance and cross track error says shorter is always better, at every noise
level. Control jitter says the opposite, by nearly six to one. Optimise the
first alone and you build a robot that follows the line beautifully and shakes
itself to pieces. Which one you optimise is an engineering decision about the
robot's job, not something the data settles.

**Cross track error is measured to the path, not to a waypoint.** Measuring to
the nearest vertex reports 2.5 cm of error for a robot sitting exactly on a path
sampled every 5 cm. That artefact is the same size as the differences you are
trying to detect.


## The six maps and why each one exists

A comparison run on a map where every algorithm ties teaches nothing. Each of
these was chosen, and in two cases searched for, because it makes a specific
pair of algorithms disagree.

**`empty_room`** is the control. Every optimal planner returns 35.36. What
differs is effort: Dijkstra expands 777 cells, A* with the octile heuristic
expands 26. Thirty times less work for an identical answer. This is the whole
argument for A* in one number.

**`corridors`** is the counterweight. Three walls with alternating doorways mean
the straight line to the goal points at a wall for most of the search, so the
estimate is badly wrong nearly everywhere. A* still wins, 434 against 594, but
the margin has collapsed from thirty times to a quarter. A heuristic is worth
what the geometry lets it be worth.

**`greedy_trap`** is a three-sided box, open at the top, sitting across the line
from start to goal. Every cell inside it is nearer the goal in straight-line
terms than the cells outside, so a search ordered on the heuristic alone
descends into it and has to climb back out. Greedy best first returns a path 33
percent longer than the shortest. A* with an inadmissible heuristic returns one
11 percent longer. Both are visible in the drawn path: run
`python3 compare.py --map greedy_trap` and look at the line.

**`cluttered`** (seed 7) is the quiet one, and the one most worth your attention.
A* with the Manhattan heuristic returns a path 1.5 percent longer than optimal
while expanding 38 cells against the correct heuristic's 182. Nothing looks
wrong. The path is smooth, the robot drives it, the planner is five times
faster. This is what an inadmissible heuristic actually looks like in a system
that ships, and it is the reason the difference between Manhattan and octile is
worth caring about rather than memorising.

**`steps_vs_cost`** (seed 0) exists because BFS is optimal in the wrong currency.
It returns 30 steps measuring 39.94; Dijkstra returns 32 steps measuring 39.46.
BFS wins on the thing it optimises and loses on the thing you want, because it
counts a diagonal as one move and the robot has to drive sqrt(2) metres. This
map was found by searching 200 random seeds, and only four of them separate the
two planners at all. That is the real warning. BFS on a diagonal grid is usually
right, and you will not notice the day it is not.

**`sealed_goal`** has no path. Every planner must terminate and say so. One that
hangs, or returns a path not reaching the goal, fails. "Unreachable" is not an
edge case; it is what a real robot faces the moment a door closes behind it.

## What you should be able to say afterwards

Exercise 4.3 asks for a short write-up, not a table. Four claims, each backed by
a number you measured yourself:

1. Why A* and Dijkstra return the same distance on every solvable map, and what
   the heuristic changes instead.
2. Why BFS stops being optimal when you add diagonals, and what you would have
   to change about the grid, not the algorithm, to make it optimal again.
3. What an inadmissible heuristic bought and what it cost, on `cluttered` and on
   `greedy_trap`, in percent.
4. Which of these you would actually put on the robot in Week 9, and why. There
   is a defensible answer that is not "A* with octile", and if you can argue for
   it you have understood the trade.

Do not write that one algorithm is better than another. Say what each one
optimises and what it gives up, because that is the only form of the statement
that survives contact with a different map.

## Where this goes next

Lab 6 hands you Nav2's global planner on a real costmap. It is A* with a few
engineering details you will now recognise: a downsampled grid, a tolerance on
the goal, and a cost term for driving close to walls. Nothing in it will be new.
The point of writing these four by hand is that when the planner in Week 6
behaves strangely, you will be debugging a thing you have built rather than a
black box you have configured.
