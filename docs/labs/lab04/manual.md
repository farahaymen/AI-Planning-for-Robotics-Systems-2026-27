---
title: "Lab 4: Occupancy Grid Mapping"
subtitle: "Autonomous Robotics with ROS 2: Mapping, Navigation and Reinforcement Learning"
author: "British University in Egypt"
date: "Duration 2 hours | ARC VM 2026.1"
---

# Lab 4: Occupancy Grid Mapping

**Course** Autonomous Robotics with ROS 2: Mapping, Navigation and Reinforcement Learning
**Duration** 2 hours
**Environment** ARC VM 2026.1. This lab runs entirely in NumPy against a recording, so it works in every graphics tier including one where Gazebo will not start.
**Packages** `rosbag2_py`, `numpy`, `matplotlib`, `arc_lab4`

**Prerequisites**

Lab 3, and specifically the bag you recorded there. If your recording is
unusable, tell your demonstrator at the start of the session. Your demonstrator
holds a recording made on the reference VM and will give you a copy, and using
it costs you nothing. Do not spend the session collecting data again.
The planning notebook is due today; see the appendix.

---

## Before the session

### Why this lab matters

A LiDAR gives you a few hundred distance readings, ten times a second, each valid
only from where the robot happened to be standing. A map is a single persistent
statement about a whole building. Getting from the first to the second is the
core inference problem in mobile robotics, and today you implement it.

You will not use a library. The algorithm is about forty lines and it is worth
writing yourself, because from Lab 5 onwards you will configure SLAM Toolbox and
Nav2, and configuring something you have never built is how people end up
adjusting parameters at random.

There is also a practical point about method. This lab runs against a recording
rather than a live simulation. Replaying a bag means the same input produces the
same map every time, so when your map looks wrong you can change one thing and
see exactly what it did. Debugging against a live simulator, where the robot
takes a slightly different path each run, is much harder and teaches worse
habits.

### The problem, stated carefully

You have a sequence of scans. For each, you know roughly where the sensor was,
from odometry. You want to estimate, for every cell in a grid, the probability
that it is occupied.

Two simplifying assumptions make this tractable, and both are worth naming
because both are false.

The first is that cells are independent. In reality walls are continuous and
knowing one cell is occupied tells you a lot about its neighbour. Occupancy grid
mapping ignores this, which is why the maps have ragged edges.

The second is that the pose is known. Today it comes from odometry, which drifts.
By the end of a long run your map will smear, and that smearing is not a bug in
your code. It is the reason SLAM exists, and Lab 5 addresses it.

### Log odds, and why not probabilities

The Bayesian update for a single cell given a new measurement multiplies
probabilities. Doing that hundreds of times per cell underflows to zero and is
slow. Instead store the log odds:

$$
l = \log \frac{p}{1 - p}
\qquad \Longleftrightarrow \qquad
p = \frac{1}{1 + e^{-l}}
$$

Under this transformation the Bayesian update becomes addition:

$$
l_{t} = l_{t-1} + l_{\text{sensor}} - l_{\text{prior}}
$$

With a prior of 0.5, meaning complete ignorance, $l_{\text{prior}} = 0$ and the
update is simply $l_t = l_{t-1} + l_{\text{sensor}}$. Every observation adds a
constant. That is the entire algorithm.

![Left: the log odds transformation. Right: why the clamp matters.](docs/figures/lab04_log_odds.png){width=95%}


### The inverse sensor model

The model answers: given that a beam reported distance $r$, what does that say
about the cells it passed through?

It says two things. Cells along the beam before $r$ are probably free, because
the beam travelled through them. The cell at $r$ is probably occupied, because
something stopped the beam.

So the model is two numbers, $l_{\text{free}}$ and $l_{\text{occ}}$, added to
cells along the ray and at its endpoint respectively. In the supplied parameters
$l_{\text{occ}} = 0.85$ and $l_{\text{free}} = -0.40$.

The asymmetry is deliberate and worth understanding. One beam passes through
perhaps forty cells and terminates on one. If free and occupied evidence were
weighted equally, free space evidence would arrive forty times faster and walls
would slowly erode.

**The case that catches everyone.** A beam that reports its maximum range did not
hit anything. It means the cells along it are free and the endpoint tells you
nothing at all. If you mark the endpoint occupied, you draw a ring of phantom
obstacles at exactly the sensor's range limit, in every direction the robot ever
faced. It is the most recognisable failure in a student occupancy map, and it is
one line of code.

![One beam, two constants. Cells beyond the hit are left untouched.](docs/figures/lab04_inverse_sensor_model.png){width=82%}


### Clamping

Without a limit, a cell observed a thousand times reaches a log odds of 850 and
no amount of contrary evidence will move it. The map becomes permanently certain,
and a chair that someone moves stays in the map forever.

Clamping the log odds to a range such as $[-5, 5]$ bounds how confident the map
is allowed to become. This is not a numerical convenience. It is what makes the
map correctable, and it is the same reason Nav2 costmap layers clear as well as
mark.

### Read before the session

`starters/lab04/occupancy_skeleton.py` is the file you edit. It contains the data
structures, the Bresenham line tracer and the coordinate handling. The sensor
model is removed from `integrate_scan` and `_update_cell`, and what belongs there
is marked with six TODOs. Read the whole file, including the tests in
`starters/lab04/tests`, which describe the behaviour your implementation must
produce.

`starters/lab04/occupancy.py` is the reference implementation. Do not open it
until your own tests pass. It is there to compare against afterwards.

---

> ### Engineering Practice: developing against recordings
>
> The reason this lab uses a bag is the same reason professional robotics teams
> log everything.
>
> A robot failure usually depends on circumstances you cannot recreate on
> demand. Somebody was standing in the doorway, the floor was wet, the battery
> was at 15 percent. Asking the robot to fail again so you can watch is not a
> plan. If the run was recorded, the failure can be replayed a hundred times
> while you instrument the code, and it becomes an ordinary debugging problem.
>
> This changes how teams work. A bug report at a robotics company is often a bag
> file and a timestamp rather than a description. Regression tests are recorded
> runs with expected outputs. When a perception change is proposed, it is
> evaluated by replaying an archive of past runs rather than by driving the robot
> around again.
>
> The habit worth taking from today is smaller than that and immediately useful.
> When something in your project misbehaves, your first move should be to capture
> it rather than to start changing code. A recording turns "it sometimes drifts"
> into a thing you can measure.

---

### Pre-lab quiz

Five questions covering the log odds transformation, why the update becomes
addition, why $l_{\text{free}}$ and $l_{\text{occ}}$ differ in magnitude, what a
maximum range return means, and what clamping is for.

---

## In the session

### Stage 0: health check and data (10 minutes)

```
course-check
ros2 bag info ~/arc_ws/bags/lab03_mapping_run
```

Confirm your bag has `/scan`, `/odom`, `/tf` and `/tf_static`, and that the scan
count is roughly ten times the duration. If it is not, use the reference run
rather than spending the session on data collection.

### Stage 1: demonstration (10 minutes)

Your demonstrator will show a correct map assembling from the reference bag, then
show the same run with one line of the sensor model changed, producing the
phantom ring described above. Look at both before you write anything.

### Exercise 4.1: get the data into NumPy (15 minutes)

The loader is supplied because parsing a bag is not the learning objective.

**Code 4.1: Extracting scans and poses from a recording**

```python
#!/usr/bin/env python3
"""Read a bag into plain NumPy arrays. No ROS node, no live simulation."""

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


def read_run(path, scan_topic="/scan", odom_topic="/odom"):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""))

    scans, odoms = [], []
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic == scan_topic:
            scans.append((stamp_ns, deserialize_message(data, LaserScan)))
        elif topic == odom_topic:
            odoms.append((stamp_ns, deserialize_message(data, Odometry)))
    return scans, odoms


def pose_from_odom(msg):
    """Extract (x, y, theta) from an Odometry message."""
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    # Yaw from a quaternion that is a pure z rotation.
    theta = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
    return float(p.x), float(p.y), float(theta)


def nearest_pose(scan_stamp_ns, odoms):
    """Match each scan to the odometry sample closest in time.

    Odometry runs at 50 Hz and the LiDAR at 10 Hz, so there is no sample at
    exactly the right instant. Nearest neighbour introduces up to 10 ms of error,
    which at 0.5 m/s is 5 mm and is acceptable here. Interpolating would be
    better and is a legitimate extension.
    """
    stamps = np.array([s for s, _ in odoms])
    return pose_from_odom(odoms[int(np.argmin(np.abs(stamps - scan_stamp_ns)))][1])
```

The timestamp matching is worth a moment. The two sensors run at different rates
and are not synchronised, so every scan has to be associated with a pose that was
measured at a slightly different time. Nearest neighbour is the crude answer and
it is good enough at these speeds. Nav2 and SLAM Toolbox interpolate, using TF,
which is one of the things TF is actually for.

Verify before proceeding:

```python
scans, odoms = read_run("bags/lab03_mapping_run")
print(f"{len(scans)} scans, {len(odoms)} odometry samples")
print(f"scan has {len(scans[0][1].ranges)} beams, "
      f"angle_min {scans[0][1].angle_min:.3f}, "
      f"increment {scans[0][1].angle_increment:.5f}")
```

You should see 360 beams. If you see a different number, your world file differed
from the standard, and the angle array you construct must come from the message
rather than from an assumption.

### Exercise 4.2: implement the sensor model (30 minutes)

Open `occupancy_skeleton.py` and complete `integrate_scan` and `_update_cell`.
The tests describe exactly what is expected, so run them as you go. The
`ARC_OCCUPANCY` variable is what points the tests at your file rather than at the
reference:

```
cd ~/arc_ws/src/arc-course
ARC_OCCUPANCY=occupancy_skeleton python3 -m pytest starters/lab04 -q
```

Without that variable the tests import `occupancy.py`, the reference
implementation, and they all pass without you having written anything. Once your
own version passes, read `occupancy.py` and compare it against what you wrote.

Work in this order, because each step makes the next one visible.

First, handle one beam: convert the endpoint to world coordinates, convert both
endpoints to cells, trace the line, and add $l_{\text{free}}$ to every cell
except the last.

Second, add $l_{\text{occ}}$ to the last cell, but only when the return is a
genuine hit rather than a maximum range miss.

Third, apply the clamp in `_update_cell`.

Then run it over the whole bag:

**Code 4.2: Building the map from the whole run**

```python
import numpy as np
import matplotlib.pyplot as plt
from occupancy_skeleton import MappingParams, OccupancyMap, sensor_pose_from_base

params = MappingParams(resolution=0.05, l_occ=0.85, l_free=-0.40,
                       max_range=12.0, min_range=0.12)
# The warehouse runs from (0, 0) to (12, 12) in world coordinates and the robot
# spawns at (1, 1), so the grid has to cover 0 to 12 with a margin. Anything
# outside the grid is dropped silently by the in-bounds guard rather than
# reported, so an origin of (-7, -7) loses the east wall, the north wall and two
# of the three pillars without any error.
grid = OccupancyMap(width_m=14.0, height_m=14.0, origin=(-1.0, -1.0), params=params)

scans, odoms = read_run("bags/lab03_mapping_run")
for stamp, scan in scans:
    base_pose = nearest_pose(stamp, odoms)
    # The LiDAR sits 0.10 m forward of base_link. Using the base pose instead
    # shifts the entire map by 10 cm along the heading, which reads as blur
    # rather than as an obvious error.
    pose = sensor_pose_from_base(base_pose, offset_x=0.10)

    angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
    grid.integrate_scan(pose, np.array(scan.ranges), angles)

plt.imshow(grid.probability(), origin="lower", cmap="gray_r", vmin=0, vmax=1)
plt.title(f"coverage {grid.coverage() * 100:.1f}%")
plt.savefig("map.png", dpi=150)
```

**[SCREENSHOT PLACEHOLDER]**
The completed occupancy map next to a top-down view of the Gazebo world it was
built from.
*Instructor note: capture the Gazebo view orthographically from directly above at
the same scale, so walls can be compared position by position. Mark the doorway
that Lab 6 later plans through.*

### Exercise 4.3: reproduce the phantom ring, then fix it (10 minutes)

Deliberately break the maximum range handling by treating every return as a hit,
and rebuild the map.

You will see a ring of obstacles at exactly 12 metres from wherever the robot
travelled. Save that image, because it is the clearest possible illustration of
what the sensor model actually asserts. Then restore the correct behaviour.

![Both maps built by `occupancy.py` from the same run. The only difference is whether a return at the sensor limit is treated as a hit.](docs/figures/lab04_mapping_correct_vs_ring.png){width=100%}


Record in your exit task: at what radius does the ring appear, and why is it a
ring rather than a filled disc?

### Experiment 4.4: what the parameters do (20 minutes)

Rebuild the map with each setting and record the result.

| Setting | Walls sharp or blurred? | Free space complete? | Unknown cells (%) | Notes |
|---------|-------------------------|----------------------|-------------------|-------|
| Baseline: `l_occ` 0.85, `l_free` -0.40 | | | | |
| `l_free` -0.85 (equal weighting) | | | | |
| `l_free` -0.05 | | | | |
| `l_max` 50.0 (effectively unclamped) | | | | |
| `resolution` 0.10 | | | | |
| `resolution` 0.02 | | | | |

Then answer two questions. Why does equal weighting erode walls rather than
sharpening them? And what does halving the resolution cost you, in memory and in
the time your loop takes, and what does it buy?

The resolution question matters beyond this lab. Nav2 costmaps default to 0.05 m
and the choice is the same trade you are measuring here.

### Exercise 4.5: export a map ROS can use (15 minutes)

A map is only useful if the rest of the stack can load it. The ROS format is a
PGM image plus a YAML file giving the resolution and origin.

**Code 4.3: Writing a ROS compatible map**

```python
import numpy as np
import yaml
from PIL import Image

def save_ros_map(grid, stem="my_map"):
    occ = grid.to_ros_occupancy(occupied_threshold=0.65, free_threshold=0.25)

    # ROS map images use 0 for occupied and 254 for free, which is the opposite
    # of the intuitive direction, and unknown is 205. Getting this inverted
    # produces a map where the robot believes the walls are the only free space.
    image = np.full(occ.shape, 205, dtype=np.uint8)
    image[occ == 0] = 254
    image[occ == 100] = 0

    # The image origin is top left, the grid origin is bottom left.
    Image.fromarray(np.flipud(image)).save(f"{stem}.pgm")

    with open(f"{stem}.yaml", "w") as f:
        yaml.safe_dump({
            "image": f"{stem}.pgm",
            "resolution": grid.params.resolution,
            "origin": [grid.origin_x, grid.origin_y, 0.0],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.25,
        }, f, default_flow_style=False)
```

Two conventions in that function will catch you if you skip them. The greyscale
encoding runs opposite to intuition, with 0 meaning occupied. And the image row
order is flipped relative to the grid, because images start at the top left and
maps start at the bottom left.

Load it and look at it:

```
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=my_map.yaml
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
rviz2
```

The lifecycle transitions should be familiar from Lab 1. `map_server` is a
managed node, and a configured but inactive map server publishes nothing.

### Exit task (10 minutes)

Commit and push, then submit:

1. Your completed `occupancy_skeleton.py` with all tests passing under
   `ARC_OCCUPANCY=occupancy_skeleton`.
2. Your map image, and the phantom ring image from Exercise 4.3.
3. Your completed parameter table with the two written answers.
4. Your exported `.pgm` and `.yaml`, loading correctly in RViz.
5. One sentence on where in your map the odometry drift is visible, and how you
   can tell it is drift rather than a mapping error.

---

## Appendix: the assessed planning notebook

Due today, 10 percent of the module mark. This is independent work rather than a
lab exercise, because it needs no ROS, no simulator and no virtual machine, and
it is the closest content in this course to your Machine Learning and Deep
Learning modules.

**Task.** Implement breadth first search, depth first search, Dijkstra and A* on
a 2D occupancy grid, then compare them properly.

**Requirements.**

1. All four algorithms, in plain Python with `heapq` and `collections`. No
   planning libraries.
2. Each returns the path, the number of nodes expanded, and the runtime.
3. Run all four on at least five maps of differing structure, including one
   where the greedy direction is a trap and one with no valid path.
4. Compare A* with at least two heuristics, one of which is inadmissible, and
   report what happens to optimality.
5. A results table with path length, nodes expanded and runtime, and a short
   written analysis.

**Questions your analysis must answer.**

Why does BFS find the shortest path on an unweighted grid while DFS does not?
Under what condition is A* guaranteed to return an optimal path, and what did you
observe when you violated it? Why does A* expand fewer nodes than Dijkstra when
both are optimal? What happens to all four when the goal is unreachable, and what
should a planner do about it?

**Marking.** Correctness 40, experimental method and the comparison table 25,
written analysis 20, code quality and tests 15.

> ### Research Frontier: is search the right primitive?
>
> A* on a grid is sixty years old and still the default global planner in Nav2,
> which is worth pausing on. It is optimal, complete, predictable and fast enough,
> and those four properties are hard to beat.
>
> Research nevertheless pushes on it from two directions. The first asks whether
> a learned model can predict where the path will go and restrict the search to
> that region, giving large speedups on maps resembling the training set at the
> cost of guarantees when they do not. The second asks whether planning should be
> differentiable, so that a planner can be trained end to end with the perception
> feeding it, rather than the two being tuned separately.
>
> Both are worth knowing about and neither has displaced A* in production. When
> you read a paper claiming a learned planner is faster, the questions to ask are
> what happens on a map unlike the training distribution, and whether the method
> can report that no path exists. Completeness is unglamorous and it is why the
> old algorithm is still there.

---

## Troubleshooting

**The map is empty.**

Check that scans are being read at all, and that your grid bounds and origin
actually contain the robot's trajectory. Print the minimum and maximum odometry
x and y before you build.

**The map is a smear with no recognisable walls.**

Almost always the pose. Confirm you are converting the quaternion correctly, that
you are applying the sensor offset, and that the scan angles come from
`angle_min` and `angle_increment` in the message rather than assumed values.

**Walls appear but are doubled or ghosted.**

Odometry drift, which is expected on a long run and is the subject of Lab 5. If
the doubling appears within the first ten seconds it is not drift, it is a pose
error.

**The map is mirrored or rotated 90 degrees.**

Row and column are swapped somewhere. The grid is indexed `[row, col]` where row
comes from y and col from x. This is the same transposition trap as the
`OccupancyGrid` reshape.

**The map has a ring of obstacles at a constant radius.**

Exercise 4.3, arriving unintentionally.

---

## Connection to Lab 5

Your map has a defect you cannot fix with a better sensor model. Over a long run
the walls thicken and double, because every scan was placed using odometry and
odometry drifts without bound.

Next week you attack that directly. You will measure the drift, then watch a
particle filter correct it by matching scans against a map. Then you will run
SLAM Toolbox, which solves mapping and localisation simultaneously by treating
the poses themselves as unknowns and optimising them, which is precisely the
assumption you were told today to be suspicious of.

The map you built today is also a useful comparison. When SLAM Toolbox produces
its version of the same room from the same bag, the difference between the two is
exactly the value that pose optimisation adds.

---

## References

**Textbooks**

Thrun, S., Burgard, W. and Fox, D. (2005). *Probabilistic Robotics*. MIT Press.
Chapter 9 is the source for everything in this lab. Section 9.2 derives the log
odds update, and Table 9.1 is the algorithm you implemented.

Moravec, H. and Elfes, A. (1985). High resolution maps from wide angle sonar.
*IEEE International Conference on Robotics and Automation*. The original
occupancy grid paper, short and readable, and worth seeing that the idea predates
almost everything else in this course.

**Official documentation**

`nav2_map_server` and the map file format:
https://docs.nav2.org/configuration/packages/configuring-map-server.html
The authoritative description of the PGM and YAML conventions used in Exercise
4.5.

rosbag2 Python API: https://github.com/ros2/rosbag2

**Repositories**

`ros-navigation/navigation2`, package `nav2_costmap_2d`. The `StaticLayer` and
`ObstacleLayer` sources are a production implementation of what you wrote today,
including the marking and clearing behaviour that clamping makes possible.

---

## Further reading

`docs/references.md` has a fuller list under **Lab 4. Occupancy grid mapping and
classical path planning**, with papers, industry write-ups and the documentation
worth keeping open. Every entry says what you get from it and which part of the
lab it connects to.

If you read one thing, read *Grid-Centric Traffic Scenario Perception for
Autonomous Driving* (Shi and others, 2023). Its appendix sets out the binary
Bayes filter and a worked inverse sensor model for a single LiDAR return, which
is the log-odds update you implemented today.
