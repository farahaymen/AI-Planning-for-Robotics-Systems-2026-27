---
title: "Lab 6: Autonomous Navigation with Nav2"
subtitle: "Autonomous Robotics with ROS 2: Mapping, Navigation and Reinforcement Learning"
author: "British University in Egypt"
date: "Duration 2 hours | ARC VM 2026.1"
---

# Lab 6: Autonomous Navigation with Nav2

**Course** Autonomous Robotics with ROS 2: Mapping, Navigation and Reinforcement Learning
**Duration** 2 hours
**Environment** ARC VM 2026.1 (Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, Nav2 Jazzy)
**Packages** `nav2_bringup`, `nav2_navfn_planner`, `dwb_core`, `nav2_mppi_controller`, `nav2_costmap_2d`, `arc_nav`, `arc_eval`

**Prerequisites**

You will need the map you saved in Lab 5, your A* implementation from the
planning notebook, and a working `course-check`. If you did not save a map, a
reference map is provided in `arc_nav/maps/arc_warehouse.yaml`, and using it
costs you nothing in this lab.

---

## Before the session

The reading below is the concept delivery for this lab. It replaces the lecture
portion of the session, so the two hours in the laboratory can be spent building
rather than listening. A short quiz on this material closes one hour before your
session.

### Why this lab matters

Everything you have built so far has been a component. You wrote a planner that
finds paths on a grid. You built a map of an environment and localised the robot
within it. Neither of those is a navigation system, and the gap between the two
is larger than it looks.

A robot that can plan a path still has to follow it, and the world will not
cooperate. The map is out of date the moment somebody leaves a trolley in a
corridor. The path passes within two centimetres of a wall because the planner
treats the robot as a point. The plan was computed once and the robot has since
drifted. The controller commands a velocity the wheels cannot achieve. Handling
all of this reliably, thousands of times a day, in a building full of people, is
what separates a navigation demonstration from a navigation product.

Nav2 is the system that handles it, and it is the piece of software you are most
likely to encounter directly if you work on mobile robots after you graduate.

### Where we are

In the planning notebook you ran breadth first search, Dijkstra and A* on a grid
you invented. That grid was a NumPy array of zeros and ones, and it was
convenient precisely because it was fake. In Lab 5 you produced a real map with
SLAM Toolbox and localised against it with AMCL, which gave you a
`nav_msgs/OccupancyGrid` and a `map` to `base_footprint` transform.

Those two things fit together. The occupancy grid is a NumPy array once you
reshape it, and the transform tells you where the robot sits inside it. Today you
join them, first by running your own A* on the real map, and then by handing the
same job to Nav2 and looking carefully at what Nav2 does that your implementation
does not.

### Learning outcomes

By the end of this session you should be able to:

1. Describe the Nav2 architecture in terms of its servers, its lifecycle
   management and its behaviour tree, and explain what each one is responsible
   for.
2. Convert a ROS `OccupancyGrid` into a NumPy array and plan on it with your own
   A*, then account for the differences between your path and the one Nav2
   produces.
3. Explain what a costmap layer does, and predict the effect of changing the
   inflation radius on both path shape and clearance.
4. Configure and run two different local controllers, and compare them using a
   seeded benchmark rather than a single run.
5. Diagnose a navigation stack that comes up but does not navigate.

### The architecture

![The Nav2 stack. Dashed grey: the behaviour tree sequencing the servers. Dashed red: the collision monitor's independent sensor subscription.](docs/figures/diagram_nav2_architecture.png){width=88%}

### 1.1 Nav2 is a set of servers, not a library

The first thing to understand about Nav2 is that it is not a function you call.
It is a collection of independent ROS 2 nodes, each owning one responsibility,
communicating over topics and actions. `planner_server` produces paths.
`controller_server` produces velocities. `behavior_server` performs recovery
actions such as spinning in place or backing up. `bt_navigator` decides the order
in which all of that happens.

This separation is what allows you to replace the local controller today without
touching anything else, and it is the same property that allows a company to
replace the global planner with something proprietary while keeping the rest of
the stack.

The second thing to understand is that these nodes are managed lifecycle nodes.
You met the lifecycle concept in Lab 1, and this is where it starts to matter. A
Nav2 node that has been launched is not necessarily doing anything. It has to be
configured and then activated before it will accept work. When navigation
mysteriously does nothing at all, the first question is not what is wrong with
your goal, it is whether the servers are actually active:

```
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
```

An answer of `unconfigured` or `inactive` tells you the lifecycle manager did not
finish its transitions, which usually means one of the nodes upstream failed and
the manager stopped. That is a much more useful place to start looking than the
goal pose.

The third thing is that you talk to Nav2 through an action, not a topic. Sending
a goal on a topic gives you no feedback and no way to know whether it worked. The
`navigate_to_pose` action gives you acceptance, continuous feedback including the
number of recovery behaviours that have been triggered, and a final result code.
You will use all three today.

### 1.2 Costmaps and layers

Your A* implementation treats the map as a binary array. Free or blocked, nothing
in between. That works in a notebook and it fails on a real robot for a simple
reason: the robot is not a point.

A costmap is a grid where each cell holds a value from 0 to 254 rather than a
boolean, and it is built from stacked layers, each of which is allowed to modify
the values written by the layer below it.

The **static layer** copies the map you saved in Lab 5. This is the layer that
knows about walls.

The **obstacle layer** marks cells where the LiDAR currently sees something, and
clears cells that the LiDAR has raytraced through and found empty. This is the
layer that knows about the trolley somebody left in the corridor five minutes
ago.

The **inflation layer** is the interesting one. It takes every lethal cell and
writes a decaying cost into the cells around it, out to a configured radius. The
cost at distance $d$ from the nearest obstacle is

$$
c(d) = (\text{cost}_{\text{lethal}} - 1) \cdot e^{-\alpha (d - r_{\text{inscribed}})}
$$

where $\alpha$ is the cost scaling factor and $r_{\text{inscribed}}$ is the
inscribed radius of the robot footprint. Every cell within the inscribed radius
of an obstacle is marked lethal outright, because the robot cannot possibly be
there.

Two things follow from this and both matter today. First, inflating obstacles by
the robot radius converts a robot shaped collision problem into a point collision
problem, which is what makes a point based planner such as your A* usable at all.
Second, because the cost decays smoothly rather than dropping to zero at the
boundary, the planner does not merely avoid collisions, it prefers to stay away
from walls. The strength of that preference is a parameter you control, and you
will measure its effect in Experiment 6.3.

Choosing an inflation radius is a genuine engineering trade. Too small and the
robot cuts corners tightly, clips door frames and triggers the collision monitor.
Too large and narrow doorways fill in completely, the planner reports no valid
path through a gap the robot would physically fit through, and the robot refuses
to go somewhere it could easily go. Almost every deployment tuning session
includes an argument about this number.

![Inflation on a real arena, computed by `grid_tools.inflate`. Black is the obstacle, blue the inflated region.](docs/figures/lab06_inflation.png){width=100%}


### 1.3 The local controller

The global planner gives you a path. The path is a sequence of poses, it assumes
a point robot, and it says nothing about velocity. Converting it into wheel
commands that respect the robot's acceleration limits while avoiding an obstacle
that appeared after the plan was made is the local controller's job.

**DWB** is the Nav2 implementation of the dynamic window approach. On each cycle
it samples a set of constant velocity commands that are reachable within the
robot's acceleration limits, rolls each forward for a short simulated time, and
scores the resulting trajectories with a set of weighted critics. The critics in
your configuration reward staying close to the path, making progress towards the
goal and finishing with the correct heading, while penalising proximity to
obstacles and oscillation. The command that scores best is sent.

DWB is transparent, cheap and easy to reason about, which is why it remains the
default in a great many deployments. Its weakness follows from its assumption:
because every sampled trajectory holds a constant velocity, it cannot represent a
manoeuvre that requires changing speed partway through, so it can be poor at
threading through tight or cluttered spaces.

**MPPI** relaxes that assumption. Model predictive path integral control samples
whole control sequences rather than single constant commands, rolls each one
forward through a motion model, scores the trajectories using plugin critics, and
takes a softmax weighted average of the sampled sequences as its output. Because
the average is weighted by cost, good samples dominate, and because it samples
sequences rather than constants it can produce genuinely time varying manoeuvres.

The Nav2 MPPI controller was created by Aleksei Budyakov and adapted and developed for Nav2 by Steve Macenski. It implements the `nav2_core::Controller` interface, which is what allows it to drop into the controller server in place of DWB without any other change. It is worth knowing that the implementation runs on CPU only, using AVX2 vectorisation available on essentially any machine from 2013 onwards. That is why you can run it inside the course virtual machine without a GPU, and it is also a good illustration of how much a well optimised implementation changes what is deployable.

---

> ### Industry Perspective
>
> Nav2 is not a teaching tool that happens to work. It is the navigation stack
> running on commercial autonomous mobile robots in warehouses, hospitals and
> factories today, and the design decisions you are looking at were made under
> commercial pressure.
>
> The lifecycle management that seems like ceremony exists because a robot that
> starts navigating before its localisation has converged is dangerous. The
> behaviour tree exists because the sequence of what to try when navigation fails
> is application specific and needs to be editable without recompiling. The
> collision monitor exists as a separate node with its own sensor subscription
> because a safety stop that depends on the autonomy stack being healthy is not
> a safety stop.
>
> The plugin architecture matters commercially too. A company can ship a
> proprietary planner as a `nav2_core::GlobalPlanner` plugin and keep every other
> part of the stack, including the tooling, the visualisation and the community
> maintenance. That is a large part of why Nav2 won.
>
> One thing to keep in perspective. The Nav2 Collision Monitor is a software
> safeguard inside the autonomy stack. It is not a safety rated protective device
> in the sense meant by ISO 3691-4, the standard covering driverless industrial
> trucks, which requires certified sensing and certified stopping performance
> independent of the application software. Deployed AMRs carry both. Confusing
> the two is a mistake with consequences, and you will see it in Lab 7.

> ### Research Frontier
>
> The architecture you are configuring today splits navigation cleanly into a
> global planner that is optimal on a known map and a local controller that is
> reactive but short sighted. That split has a known failure mode. Global
> planners are brittle when the world contains obstacles that were not in the
> map, and local controllers handle those well but cannot reason about a goal
> across a building.
>
> A current line of work asks whether the local half should be learned. Chandra
> and colleagues (2024) propose a hybrid planner that detects when the global
> plan has been obstructed by an unexpected obstacle and switches to a
> reinforcement learning planner for that stretch, falling back to the classical
> controller otherwise, and report a 26 percent improvement over either planner
> used alone on a physical robot. The interesting part is not the number. It is
> that the switching criterion is a simple geometric test rather than a learned
> one, which the authors argue explicitly on the grounds that a learned switch
> inherits the generalisation problems of the thing it is meant to guard.
>
> Kolomeytsev and colleagues (2025) take the complementary approach, keeping a
> graph based global planner but feeding its path into a deep reinforcement
> learning local policy as a sequence of checkpoints encoded in both the
> observation and the reward, so that the policy retains long range context it
> would otherwise lack.
>
> Both papers describe architectures you will be able to build by Lab 10, and one
> of them is a legitimate choice for your Grand Challenge entry. Read at least
> the first one before then. Notice while you configure Nav2 today which parts of
> the stack these approaches keep and which they replace, because that choice is
> the actual research contribution.

---

### Pre-lab quiz

Five questions on the VLE, closing one hour before your session. They cover the
lifecycle states, what each costmap layer contributes, the effect of the
inflation radius, the difference between DWB and MPPI sampling, and why Nav2 uses
an action rather than a topic.

---

## In the session

### Stage 0: health check and recap (10 minutes)

```
course-check
```

If any required check fails, note the incident ID and tell your demonstrator
before you start. Marks are not lost for failures of the official environment,
but the incident has to be recorded at the time.

Then bring up the simulator and the navigation stack:

```
ros2 launch arc_gazebo simulation.launch.py world:=arc_warehouse
ros2 launch arc_nav navigation.launch.py map:=$HOME/arc_ws/maps/lab05_map.yaml params:=dwb
```

**[SCREENSHOT PLACEHOLDER]**
RViz2 immediately after Nav2 activation, showing the static map, the inflated
global costmap and the robot model, with the particle cloud from AMCL still
spread out before the first pose estimate.
*Instructor note: capture at 1280x720 with the RViz displays panel visible so
students can see which displays are enabled. Take this before setting the initial
pose, so the uncertainty in the particle cloud is obvious.*

### Stage 1: demonstration and the fault (15 minutes)

Your demonstrator will run a complete navigation to a goal so that you have seen
the target behaviour before you try to produce it. Watch the global path appear,
the local costmap update as the LiDAR sweeps, and the robot follow the path.

Then the demonstrator will break one thing in the configuration and hand the
system back. Diagnosing it is part of today's exit task. Do not read the
troubleshooting section yet; try the systematic workflow first.

### Exercise 6.1: sending a goal properly (20 minutes)

You can set a goal by clicking in RViz, and you should do that once to confirm
the stack works. But clicking is not an interface you can benchmark, script or
build a mission on, so the rest of the course sends goals through the action.

Create the file below in `~/arc_ws/src/arc_nav/arc_nav/goal_client.py`.

**Code 6.1: Sending a navigation goal through the NavigateToPose action**

```python
#!/usr/bin/env python3
"""Send a single navigation goal to Nav2 and report what happened."""

import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose


class GoalClient(Node):
    def __init__(self):
        super().__init__("arc_goal_client")
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.recoveries = 0

    def send(self, x, y, yaw=0.0):
        # The action server does not exist until bt_navigator has been
        # activated, so waiting here rather than assuming saves a confusing
        # failure later.
        if not self.client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error("navigate_to_pose not available")
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        # A yaw of theta is the quaternion (0, 0, sin(theta/2), cos(theta/2)).
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        send_future = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if not handle.accepted:
            self.get_logger().error("goal rejected: is it inside the map?")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status
        self.get_logger().info(f"finished with status {status} "
                               f"after {self.recoveries} recovery behaviours")
        return status == 4  # STATUS_SUCCEEDED

    def on_feedback(self, msg):
        self.recoveries = msg.feedback.number_of_recoveries


def main():
    rclpy.init()
    node = GoalClient()
    x, y = float(sys.argv[1]), float(sys.argv[2])
    ok = node.send(x, y)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
```

Three parts of this are worth attention.

The wait for the action server is not defensive programming for its own sake. The
server genuinely does not exist until `bt_navigator` reaches the active state, so
a script that assumes it is there will fail in a way that looks like a Nav2
problem rather than a startup ordering problem.

The feedback callback captures `number_of_recoveries`. This single number is the
most useful diagnostic Nav2 gives you for free. A run that succeeded after four
recoveries and a run that succeeded after none are not the same result, and if
you only look at success you will never see the difference. It is one of the
metrics the evaluation harness records.

The exit code matters because it makes the script usable from a test or a
benchmark rather than only from a terminal.

Run it:

```
ros2 run arc_nav goal_client 4.5 2.0
```

**[GIF PLACEHOLDER]**
The robot navigating from its start pose to (4.5, 2.0), with the global path in
green and the local trajectory candidates visible.
*Instructor note: record roughly 15 seconds at 10 fps. Enable the DWB trajectory
visualisation display in RViz first, since the fan of candidate trajectories is
the clearest available picture of what a sampling controller does.*

### Exercise 6.2: your A* on the real map (20 minutes)

Nav2's `NavfnPlanner` implements Dijkstra by default, and setting `use_astar` to
true selects the A* variant. You have written both. Now compare them on the same
map.

The costmap is published as a `nav_msgs/OccupancyGrid` on
`/global_costmap/costmap`, and `starters/lab06/grid_tools.py` gives you the
conversion.

**Code 6.2: Converting the published global costmap into a NumPy grid**

```python
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

from grid_tools import occupancy_to_numpy, to_binary_obstacle_map, world_to_grid

# The costmap is latched, published once with transient local durability. A
# subscriber using the default volatile QoS connects successfully and then waits
# forever for a message that was already sent.
MAP_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)


class CostmapGrabber(Node):
    def __init__(self):
        super().__init__("arc_costmap_grabber")
        self.grid = None
        self.info = None
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                                 self.on_map, MAP_QOS)

    def on_map(self, msg):
        self.grid, self.info = occupancy_to_numpy(msg)
        self.get_logger().info(
            f"received {self.info.width} x {self.info.height} cells "
            f"at {self.info.resolution} m/cell")


def fetch_costmap(timeout_s=10.0):
    node = CostmapGrabber()
    deadline = node.get_clock().now().nanoseconds + int(timeout_s * 1e9)
    while node.grid is None and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    grid, info = node.grid, node.info
    node.destroy_node()
    if grid is None:
        raise RuntimeError("no costmap received; check the QoS and that Nav2 is active")
    return grid, info
```

The QoS profile in this cell is the point of the exercise. The costmap is
published once with transient local durability so that late subscribers still
receive it. A subscriber using the default settings will connect, report no
errors, and receive nothing at all. This is the same class of mistake as the
LaserScan reliability problem from Lab 1, and it will happen to you again in your
project, so it is worth recognising the symptom: a subscription that exists,
reports a publisher, and never fires its callback.

Now plan on it with your own implementation.

**Code 6.3: Planning with your A* and comparing against Nav2's path**

```python
import numpy as np
from grid_tools import (fetch_costmap, to_binary_obstacle_map, inflate,
                        world_to_grid, grid_to_world, path_length_m)
from my_planner import astar          # your planning notebook implementation

grid, info = fetch_costmap()

# Nav2 has already inflated this costmap, so values above the threshold include
# both real obstacles and the inflated region around them. Planning on the
# inflated map is what makes a point based planner safe for a robot with size.
obstacles = to_binary_obstacle_map(grid, occupied_threshold=253,
                                   unknown_is_obstacle=False)

start = world_to_grid(0.0, 0.0, info)
goal = world_to_grid(4.5, 2.0, info)

path, expanded = astar(obstacles, start, goal)
print(f"A*: {len(path)} cells, {expanded} nodes expanded, "
      f"{path_length_m(path, info):.2f} m")
```

Compare three numbers against the path Nav2 produced for the same goal, which you
can measure by echoing `/plan`:

| Planner | Path length (m) | Nodes expanded | Minimum distance to an obstacle (m) |
|---------|-----------------|----------------|--------------------------------------|
| Your A*, uninflated map | | | |
| Your A*, Nav2 costmap | | | |
| NavFn (Dijkstra) | | not reported | |
| NavFn (`use_astar: true`) | | not reported | |

Your A* on the raw binary map will produce the shortest path and it will run
along the walls, because nothing in your cost function discourages that. Planning
on the inflated costmap moves it away from the walls without you changing a line
of your planner. That is the whole idea of the inflation layer, and seeing it
happen to your own code is more convincing than reading about it.

**[SCREENSHOT PLACEHOLDER]**
RViz showing your A* path and the Nav2 `/plan` overlaid on the inflated global
costmap.
*Instructor note: publish the student path as a `nav_msgs/Path` on `/my_plan` and
add both Path displays in different colours. The corner cutting difference near
walls should be clearly visible, so choose a goal that requires a corner.*

### Experiment 6.3: what the inflation radius actually does (15 minutes)

Change `inflation_radius` in `arc_nav/config/nav2_dwb.yaml` and re-run the same
goal three times, once at each value.

| `inflation_radius` | Path length (m) | Minimum clearance (m) | Navigation time (s) | Path found? |
|--------------------|-----------------|------------------------|---------------------|-------------|
| 0.25 | | | | |
| 0.55 | | | | |
| 0.90 | | | | |

Then answer two questions in your exit task. At which value does the robot stop
being able to plan through the narrow doorway, and why does that happen at a
radius smaller than the doorway width? What would you set for a robot delivering
medication in a hospital corridor, and what would you set for a robot moving
pallets in a warehouse aisle at night, and why are those answers different?

### Exercise 6.4: a seeded controller comparison (20 minutes)

You now have a working navigation system, so the interesting question is no
longer whether it works but how well, and compared to what.

Run the benchmark against both controller configurations. The harness runs the
same mission from the same seeded start conditions for each system and reports
aggregate statistics.

**Code 6.4: Benchmark configuration for the DWB baseline**

```yaml
# arc_eval/configs/lab06_dwb.yaml
system_label: nav2_dwb
scenario: lab05_map_missions
env_factory: arc_eval.ros_nav2_env:make_nav2_env
env_args:
  goals: [[4.5, 2.0, 0.0], [1.0, 5.5, 1.57], [-2.0, 3.0, 3.14]]
  system_label: nav2_dwb
  time_limit_s: 120.0
policy_factory: arc_eval.ros_nav2_env:make_nav2_policy
seeds: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
max_steps: 4000
out: results/lab06_dwb.json
```

```
python3 -m arc_eval.runner --config arc_eval/configs/lab06_dwb.yaml
# restart Nav2 with params:=mppi, then
python3 -m arc_eval.runner --config arc_eval/configs/lab06_mppi.yaml
python3 -m arc_eval.runner --compare results/lab06_dwb.json results/lab06_mppi.json
```

Record the comparison:

| Metric | DWB | MPPI |
|--------|-----|------|
| Success rate (with 95% interval) | | |
| Collision free rate | | |
| Navigation time, mean and SD (s) | | |
| Path length, mean and SD (m) | | |
| Minimum clearance, mean (m) | | |
| Recovery behaviours per run | | |

One rule applies to every comparison you make for the rest of this course. If the
difference between two systems is smaller than the standard deviation of either,
you have not measured a difference, you have measured noise. Reporting that
honestly earns full marks for method. Reporting a 4 percent improvement from ten
runs as though it were a result does not.

Ten seeds is enough to see a large effect and not enough to see a small one. For
your project claims you will use thirty.

### Exit task (8 minutes)

Commit and push, then submit on the VLE:

1. `goal_client.py`, working.
2. Your filled planner comparison table and inflation table.
3. The two benchmark JSON files and the comparison output.
4. Two or three sentences identifying the fault your demonstrator introduced in
   Stage 1, the command that revealed it, and what the symptom was.

Nothing counts as submitted until it appears on your remote. The virtual machine
does not survive the end of the session.

---

## Troubleshooting

Work through this in order rather than jumping to the item that sounds like your
problem. The order is the diagnostic workflow, and the point of the workflow is
that it works even when you have no idea what is wrong.

**Nothing happens when I send a goal.**

Check the lifecycle state before anything else.

```
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /controller_server
```

If these are not `active`, the lifecycle manager did not complete. Look at the
launch output for the first node that failed, not the last error printed.

**The action server is not available.**

```
ros2 action list
ros2 action info /navigate_to_pose -t
```

If the action is absent, `bt_navigator` is not active. If it is present but your
client times out, you probably have a `ROS_DOMAIN_ID` mismatch between terminals.

**The robot plans a path but does not move.**

Follow the velocity chain. Nav2 publishes to `cmd_vel_smoothed`, the collision
monitor forwards it to `cmd_vel`, and the controller consumes that.

```
ros2 topic hz /cmd_vel_smoothed
ros2 topic hz /cmd_vel
ros2 control list_controllers
```

If `cmd_vel_smoothed` is publishing and `cmd_vel` is not, the collision monitor
is stopping you, which means it believes something is inside its stop polygon.
If both are publishing and the robot is still stationary, the controller is not
active.

**The costmap is empty or the robot sees nothing.**

```
ros2 topic hz /scan
ros2 topic info /scan --verbose
ros2 run tf2_ros tf2_echo base_footprint laser_link
```

An empty costmap with a healthy `/scan` is nearly always a frame problem. The
`frame_id` in the scan message has to match a frame in the TF tree, and the
costmap has to be able to transform from that frame to its own global frame.

**Planning fails with "no valid path".**

Either the goal is inside inflated space, or the corridor between you and it has
been inflated closed. Reduce `inflation_radius` temporarily to test which. If a
smaller radius finds a path, the geometry was the problem rather than the goal.

**The TF tree looks wrong.**

```
ros2 run tf2_tools view_frames
```

You should see `map` to `odom` published by AMCL, `odom` to `base_footprint`
published by the diff drive controller, and the fixed sensor joints published by
`robot_state_publisher`. Two publishers for the same transform is a common and
confusing failure, and it usually means something was launched twice.

---

## Connection to Lab 7

You now have a navigation system that works. Today it worked because the
environment cooperated, and that is the part worth being suspicious about.

Try this before you leave if you have time. Send a goal, then place an obstacle
directly on the path once the robot has committed to it. Nav2 will replan, and
depending on where you put the obstacle it may also spin, back up or give up
entirely. The behaviour you see is not hard coded. It is a behaviour tree, and
`bt_navigator` is executing it.

Next week you take that tree apart. You will look at why navigation fails, what
the recovery behaviours actually do, how the collision monitor decides to stop,
and how to modify the tree so that the robot handles a blocked corridor the way
your application needs rather than the way the default assumes. You will be given
a deliberately fragile robot and asked to make it robust, which is a fair
description of most of the work in deployed robotics.

---

## References

**Textbooks**

Thrun, S., Burgard, W. and Fox, D. (2005). *Probabilistic Robotics*. MIT Press.
Chapter 9 covers occupancy grid mapping and gives the probabilistic account of
what a costmap is approximating.

Lynch, K. M. and Park, F. C. (2017). *Modern Robotics: Mechanics, Planning, and
Control*. Cambridge University Press. Chapter 10 covers motion planning,
including the configuration space argument that underlies obstacle inflation.

Macenski, S., Martín, F. and Ginés, J. (2022). *A Concise Introduction to Robot
Programming with ROS 2*. CRC Press. Written by Nav2 maintainers and the most
direct treatment of the material in this lab.

**Official documentation**

Nav2 documentation, Jazzy: https://docs.nav2.org
Configuration guides for `nav2_costmap_2d`, `dwb_core` and `nav2_mppi_controller`
are the authoritative parameter references and should be your first stop before
searching elsewhere.

ROS 2 Jazzy documentation: https://docs.ros.org/en/jazzy
Managed node lifecycle and QoS settings.

**Repositories**

`ros-navigation/navigation2` on GitHub. The `nav2_mppi_controller` README
documents every critic and its weight, and reading the critic source is the
fastest way to understand what the controller is actually optimising.

**Papers**

Macenski, S., Martín, F., White, R. and Clavero, J. G. (2020). The Marathon 2: A
Navigation System. *IEEE/RSJ International Conference on Intelligent Robots and
Systems (IROS)*. arXiv:2003.00368.
The paper that introduced Nav2 and explains the architectural decisions behind
the server split and the behaviour tree. Read it to understand why the system is
shaped the way it is rather than what its parameters do.

Macenski, S., Moore, T., Lu, D. V., Merzlyakov, A. and Ferguson, M. (2023). From
the desks of ROS maintainers: A survey of modern and capable mobile robotics
algorithms in the field of Robotics and Autonomous Systems. *Robotics and
Autonomous Systems*, 168.
A maintainer's comparison of the available planners and controllers with
recommendations about which to use in which application. The most useful single
document for choosing a Nav2 configuration.

Williams, G., Aldrich, A. and Theodorou, E. A. (2017). Model Predictive Path
Integral Control: From Theory to Parallel Computation. *Journal of Guidance,
Control, and Dynamics*, 40(2).
The theoretical basis of the MPPI controller. Section 2 is enough to understand
where the softmax weighting comes from.

Chandra, R. et al. (2024). Hybrid Classical/RL Local Planner for Ground Robot
Navigation. arXiv:2410.03066.
The hybrid architecture discussed in the Research Frontier box. Relevant to your
Grand Challenge entry and short enough to read properly.

Kolomeytsev, Y. et al. (2025). Hybrid Motion Planning with Deep Reinforcement
Learning for Mobile Robot Navigation. arXiv:2512.24651.
Global graph planner feeding checkpoints into a DRL local policy. The
complementary approach to the previous paper.

**Video**

Macenski, S. Nav2 design and architecture, ROSCon FR 2023. Motivates the Nav2
architecture from ROS 2 and mobile robotics design principles, and covers in
thirty minutes what would otherwise take a long time to assemble from
documentation.
