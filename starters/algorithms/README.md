# The offline algorithms notebook

Pure Python. No ROS, no Gazebo, no simulator. Everything here runs on any
machine with Python, NumPy and Matplotlib, which is deliberate: the week the
simulator has a bad day should not be the week nobody learns A*.

This folder spans Labs 3 to 5 and it is what Project 1 is built from.

| File | Lab | What it is |
| --- | --- | --- |
| `motion.py` | 4 | the robot, the unicycle model, actuator lag |
| `gridmap.py` | 4 | the six maps, and the helpers that measure a path |
| `planners_skeleton.py` | 4 | **you edit.** BFS, DFS, Dijkstra, A*, sixteen TODO markers |
| `control_skeleton.py` | 4 | **you edit.** PID and Pure Pursuit, sixteen TODO markers |
| `reactive_skeleton.py` | 3, 4 | **you edit.** Emergency braking and follow the gap, eighteen TODO markers |
| `coverage_skeleton.py` | 4 | **you edit.** Boustrophedon coverage, thirteen TODO markers |
| `sensing.py` | 3 | a LiDAR simulated on a grid map |
| `planners.py`, `control.py` | | reference implementations to compare against |
| `tests/` | | 168 tests, and the specification |
| `compare.py` | | every planner on every map |
| `figures.py`, `figures_control.py` | | regenerate every figure in the manuals |

Run the tests against your own work:

    ARC_PLANNERS=planners_skeleton python3 -m pytest tests/test_planners.py -x -q
    ARC_CONTROL=control_skeleton   python3 -m pytest tests/test_control.py  -x -q
    ARC_REACTIVE=reactive_skeleton python3 -m pytest tests/test_reactive.py -x -q
    ARC_COVERAGE=coverage_skeleton python3 -m pytest tests/test_coverage.py -x -q

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


## The reactive behaviours

Reactive means no map, no plan, no memory: scan in, velocity out, every cycle.
That sounds primitive and it is the most important layer in the robot, because
it is the only part that still works when everything above it is wrong.

**Emergency braking** must be a TIME, not a distance. "Brake below 0.40 m"
leaves the same clearance at every speed, because it is not looking at speed at
all: too cautious to creep through a doorway, too late at 0.5 m/s. The time rule
scales by itself. Measured clearance when stopped:

    speed     time rule    distance rule
    0.10 m/s     0.10 m        0.15 m
    0.30 m/s     0.35 m        0.15 m
    0.50 m/s     0.56 m        0.13 m

It is also shaped as a wrapper: it takes a controller and returns a controller,
so it sits between the navigation stack and the wheels, can only ever veto, and
can never command motion. That is what makes it a safety layer rather than
another behaviour. Nav2's Collision Monitor, configured in Lab 7, is the same
idea with more knobs.

**Follow the gap** navigates a cluttered room for sixty seconds without touching
anything, using no map at all. Two details are load bearing. The safety bubble
around the nearest obstacle is how the robot's WIDTH gets represented in a
signal that has no notion of it; without it the robot steers confidently into
gaps it does not fit through. And the bubble must WRAP, because beam 359 is
adjacent to beam 0 and slicing instead of wrapping silently halves it whenever
the nearest obstacle is behind.

It is also the classical baseline that Lab 9 measures the reinforcement learning
policies against, so that comparison is now against something you wrote.


## Coverage planning

Every other planner here answers "how do I get from A to B". This one answers
"how do I visit all of it", which is a different problem with a different
objective, and it is the question a large part of the robotics market actually
asks: vacuum cleaners, lawn mowers, floor scrubbers, agricultural sprayers.

**Turns are the cost, not distance.** Every turn is decelerate, rotate,
accelerate, and on most machines disengage and re-engage the tool. So the
headline result is about turns, not metres. On a 6 m by 4 m furnished room with
a 0.45 m tool:

    sweep along rows      covered 100.0%   40 turns   13 passes
    sweep along columns   covered  98.1%   62 turns   20 passes

Same room, same tool, same floor cleaned. Sweeping along the long axis saves 22
turns, because turns happen at the ENDS of passes and longer passes means fewer
ends. Nobody guesses this reliably, which is why it is worth measuring.

**The spacing and the tool width are different numbers.** Conflating them is the
mistake that makes a coverage planner look perfect at every setting: widen the
gap between passes, widen the imaginary brush to match, and the uncovered
stripes vanish from the measurement while remaining on the floor. Measured with
a fixed 0.45 m tool:

    spacing    covered   turns
    0.25 m      100.0%      99      re-cleaning what the last pass did
    0.45 m      100.0%      52      the knee
    0.70 m       69.2%      34      stripes, and the robot has no idea

**Coverage is scored against REACHABLE space.** A map with a sealed cupboard
contains free cells no robot can enter; counting those as failures caps every
planner below 100 percent for reasons that have nothing to do with the planner.

The coverage planner calls the A* from Exercise 4.2 to join one pass to the
next. It is a CLIENT of a point to point planner, not a replacement for one,
and Nav2's coverage server has the same relationship to the navigation stack.
