---
title: "Lab 7: Robust Autonomy, Behaviour Trees, Recovery and Safety"
subtitle: "Autonomous Robotics with ROS 2: Mapping, Navigation and Reinforcement Learning"
author: "British University in Egypt"
date: "Duration 2 hours | ARC VM 2026.1"
---

# Lab 7: Robust Autonomy, Behaviour Trees, Recovery and Safety

**Course** Autonomous Robotics with ROS 2: Mapping, Navigation and Reinforcement Learning
**Duration** 2 hours
**Environment** ARC VM 2026.1 (Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, Nav2 Jazzy)
**Packages** `nav2_bt_navigator`, `nav2_behaviors`, `nav2_collision_monitor`, `behaviortree_cpp`, `arc_nav`, `arc_lab7`

**Prerequisites**

Lab 6 and Project 1. You should have a navigation stack that works when the
environment cooperates. Read `starters/lab07/recovery.py` before the session.

---

## Before the session

### Why this lab matters

Last week your robot navigated. This week you find out what it does when it
cannot.

That distinction is most of the difference between a demonstration and a product.
A robot that reaches its goal nine times out of ten is not 90 percent of a
working robot, because the tenth case is the one that blocks a corridor in a
hospital at three in the morning. What the robot does in that case is a design
decision, and today you make it deliberately rather than accepting the default.

There is also a specific reason this lab sits here. In three weeks your robot
will run in a competition arena you have not seen, containing an obstacle placed
where your plan wants to go. Everything today is directly applicable to that.

### Why navigation fails

It is worth having a taxonomy, because the recovery that helps depends on which
kind of failure you have.

**The plan is impossible.** No path exists to the goal on the current costmap.
Either the goal is genuinely unreachable, or, far more often, inflation has
closed a gap the robot would physically fit through. Recovery: clear the costmap
and replan, then relax the goal tolerance.

**The plan is stale.** A path existed when it was computed and something has
since moved onto it. Recovery: replan. Nav2 does this continuously, which is why
most obstacles never become visible failures.

**The controller cannot follow the plan.** The path exists and is valid, and the
robot cannot execute it, usually because it is in a tight space where the
sampled trajectories all score badly. Recovery: back up to somewhere with more
room, then try again.

**The robot is stuck.** It is issuing velocity commands and not moving. Wheels
slipping, a caster jammed against a lip, or oscillating in a doorway. Recovery:
back up, then a different approach.

**Localisation is wrong.** The robot is confidently somewhere it is not, so the
plan is correct for a place the robot is not in. This is the worst case, because
every recovery behaviour makes it worse. Recovery: relocalise, which usually
means human intervention.

Notice that "replan" fixes two of these, "clear the costmap" fixes one cheaply,
and one cannot be fixed by the navigation stack at all. A recovery strategy is a
choice about which of these you expect and in what order to try.

### Detecting failure at all

Before recovering you have to notice, and noticing is less obvious than it looks.

The naive test is distance travelled over a window. It is wrong, and the reason
is instructive. A robot oscillating in a doorway travels plenty of distance and
gets nowhere. A robot spinning in place travels none and may be recovering
correctly.

The test that works is **displacement** from the oldest pose in the window, not
the length of the path taken to get there. `StuckDetector` in the starter
implements exactly this, and its tests include the tight-circle case that a
distance based test misclassifies. This same definition is used in the Project 2
competition rules, so it is worth understanding now.

### Behaviour trees

Nav2 does not hard code what to do when navigation fails. It runs a behaviour
tree, which is an XML file you can edit without recompiling anything.

A behaviour tree is a tree of nodes ticked at a fixed rate. Each node returns
SUCCESS, FAILURE or RUNNING. The structure comes from a small number of control
nodes:

- **Sequence** runs children in order and fails if any child fails. Logical AND.
- **Fallback** runs children in order until one succeeds. Logical OR, and this is
  where recovery lives: try to navigate, and if that fails, try recoveries.
- **Decorator** modifies a child, for example by retrying it a number of times or
  rate limiting it.

The default Nav2 tree is roughly: a fallback whose first child is the pipeline
that computes a path and follows it, and whose second child is a round robin of
recovery behaviours. When navigation returns FAILURE, the fallback moves to the
recoveries, runs one, and returns to try navigating again.

Behaviour trees are used here rather than state machines because they compose.
Adding a recovery to a state machine means adding transitions from every state
that might need it, and the number of transitions grows quadratically. Adding a
node to a fallback is one line.

![The recovery tree from Code 7.1. RoundRobin escalates rather than repeating the cheapest action.](docs/figures/diagram_recovery_tree.png){width=100%}


### Recovery ordering

The order in the default tree is not arbitrary, and `RecoveryEscalator` in the
starter makes the reasoning explicit.

Clear the costmap first. It is free, it takes no time, and it fixes the most
common cause, which is a stale obstacle that has already moved away.

Spin next. A few seconds, refreshes the sensor view of the surroundings, and
cannot make the situation physically worse.

Wait after that, for dynamic obstacles that will leave on their own. A person
standing in a doorway will move; backing away from them achieves nothing.

Back up last, because it is the first action that physically moves the robot and
therefore the first that can make things worse.

Then abort. Aborting is a legitimate engineering outcome and is better than a
robot that thrashes forever. A system that reports it cannot reach a goal and
moves to the next one is better engineered than one that retries indefinitely,
and it scores better in Project 2 where interventions carry a penalty.

One subtlety worth noting because it is a real bug. The escalation counter must
reset when the robot starts making progress again. Without that, a robot that
recovers successfully three times over a long mission still aborts, because the
counter never cleared. The supplied tests check this.

### The collision monitor

Everything above lives inside the autonomy stack, which means all of it depends
on the autonomy stack being healthy. That is not an acceptable basis for not
hitting things.

The Nav2 Collision Monitor is a separate node with its own subscription to the
LiDAR, sitting between the velocity smoother and the robot. It defines polygons
around the robot and takes an action when a sensor return falls inside one:
`stop`, `slowdown`, `limit` or `approach`. It does not care what the planner
thinks or whether the behaviour tree is running.

This is defence in depth, and the reason for it is that a bug in the controller
should not be able to drive the robot into a wall.

---

> ### Industry Perspective: what a safety system actually is
>
> The Collision Monitor is a software safeguard inside the application. It is not
> a safety rated protective device, and the distinction matters commercially and
> legally.
>
> ISO 3691-4 covers driverless industrial trucks and their systems. It requires,
> among much else, that protective stopping be achieved by means whose failure
> behaviour is characterised and certified, using sensors rated for safety use,
> independent of the application software. A deployed AMR therefore carries a
> safety rated laser scanner wired to a certified safety controller that can cut
> motor power without asking the navigation stack for permission, alongside the
> Collision Monitor, which handles the ordinary case of slowing down near people.
>
> The two coexist because they solve different problems. The certified system
> answers "can this robot be permitted to operate near humans". The Collision
> Monitor answers "can we avoid triggering the certified system, because a
> protective stop means somebody has to come and reset the robot".
>
> Confusing the two is a mistake with consequences. If you write in a report that
> your Collision Monitor configuration makes the robot safe, an engineer reading
> it will stop trusting the rest of the document.

> ### Research Frontier: verifying what the robot will do
>
> A behaviour tree is editable, readable and testable, and that is a large part
> of why it displaced state machines in this domain. What it is not, in general,
> is verifiable. There is no guarantee that some sequence of sensor readings and
> failures cannot drive the tree into a loop of recoveries that never terminates,
> and in practice this is prevented by a retry counter rather than by proof.
>
> Current work approaches this from several directions. Formal methods research
> asks whether behaviour trees can be given semantics against which temporal
> logic properties are checkable, so that "the robot always eventually either
> reaches the goal or reports failure" becomes a theorem rather than a hope.
> Runtime monitoring research asks whether properties too expensive to prove can
> at least be checked continuously while the robot runs, with a fallback when a
> violation is detected.
>
> A third direction is the one most likely to reach you: using language models to
> synthesise or repair behaviour trees from a natural language description of the
> task. The results are impressive in demonstrations and the verification
> question becomes sharper rather than softer, because a tree nobody wrote is a
> tree nobody has read.
>
> The question to carry into Project 2 is simpler than any of that. For your own
> recovery logic, can you state a bound on how long the robot can spend
> recovering before it gives up? If you cannot, you have written a system whose
> worst case behaviour you do not know.

---

### Pre-lab quiz

Five questions covering the five failure categories and which recovery addresses
each, why displacement rather than distance detects a stall, what a Fallback node
does, why costmap clearing comes before backing up, and the difference between
the Collision Monitor and a safety rated protective device.

---

## In the session

### Stage 0: health check (5 minutes)

```
course-check
ros2 launch arc_nav navigation.launch.py map:=$HOME/arc_ws/maps/slam_map.yaml params:=dwb
```

### Stage 1: watch it fail (15 minutes)

Your demonstrator will run four scripted failures and let you watch what the
default tree does with each: a blocked corridor, a stale obstacle, a robot
wedged against a wall, and a goal inside inflated space.

For each, note which recovery ran and whether it helped. You will need this for
Exercise 7.3.

**[GIF PLACEHOLDER]**
The robot encountering a blocked corridor, attempting recoveries in sequence, and
either recovering or aborting.
*Instructor note: record with the RViz behaviour tree display active if
available, otherwise with the `bt_navigator` log visible alongside. About 45
seconds. The moment the fallback switches from navigation to recovery is the
frame that matters.*

### Exercise 7.1: read the tree (15 minutes)

```
ros2 param get /bt_navigator default_nav_to_pose_bt_xml
cat $(ros2 pkg prefix nav2_bt_navigator)/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml
```

Draw the tree by hand. It is small enough. Identify the top level Fallback, the
navigation pipeline, and the recovery branch, and answer in your exit task: what
condition causes the tree to move from the navigation branch to the recovery
branch, and what causes it to move back?

Then instrument it:

```
ros2 topic echo /behavior_tree_log
```

Send a goal and watch the node transitions. This topic is the single most useful
diagnostic for a robot that is doing something inexplicable, and most people
never discover it.

### Exercise 7.2: write a recovery policy (25 minutes)

Replace the default recovery branch with your own. Start from the supplied
`arc_nav/behavior_trees/arc_recovery.xml`.

**Code 7.1: A custom recovery branch**

```xml
<!-- arc_nav/behavior_trees/arc_recovery.xml -->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">

      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
        </RateController>
        <FollowPath path="{path}" controller_id="FollowPath"/>
      </PipelineSequence>

      <!-- Cheapest first, and each one is given a chance to succeed before the
           next is tried. RoundRobin cycles rather than always starting over,
           so a repeated failure escalates instead of retrying costmap clearing
           forever. -->
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <RoundRobin name="RecoveryActions">
          <Sequence name="ClearingActions">
            <ClearEntireCostmap name="ClearLocal" service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap name="ClearGlobal" service_name="global_costmap/clear_entirely_global_costmap"/>
          </Sequence>
          <Spin spin_dist="1.57"/>
          <Wait wait_duration="4.0"/>
          <BackUp backup_dist="0.30" backup_speed="0.10"/>
        </RoundRobin>
      </ReactiveFallback>

    </RecoveryNode>
  </BehaviorTree>
</root>
```

Three parts deserve comment.

`RateController` at 1 Hz limits replanning. Replanning every tick sounds safer
and is not: the planner is the most expensive thing in the stack, and at 20 Hz it
starves the controller. One replan per second is the usual compromise.

`GoalUpdated` as the first child of the ReactiveFallback is a condition, not an
action. If a new goal arrives while recovering, the tree abandons recovery
immediately rather than finishing a spin nobody wants any more.

`RoundRobin` rather than `Sequence` for the recoveries means each failure tries
the *next* recovery rather than starting again from the cheapest. That is the
escalation behaviour `RecoveryEscalator` models in Python.

Point Nav2 at your tree and test it:

```
ros2 param set /bt_navigator default_nav_to_pose_bt_xml \
  $(ros2 pkg prefix arc_nav)/share/arc_nav/behavior_trees/arc_recovery.xml
ros2 lifecycle set /bt_navigator deactivate
ros2 lifecycle set /bt_navigator activate
```

The deactivate and activate cycle is required because the tree is loaded on
configuration. Setting the parameter alone changes nothing, and this catches
people out every time.

### Exercise 7.3: the difficult robot (25 minutes)

You are given a navigation stack with three faults injected. Fix them, using the
behaviour tree log and the diagnostic workflow.

```
ros2 launch arc_lab7 difficult_robot.launch.py scenario:=1
```

| Scenario | Symptom you observe | Diagnosis | What you changed |
|----------|---------------------|-----------|-------------------|
| 1 | | | |
| 2 | | | |
| 3 | | | |

The faults come from this set: a recovery loop that never terminates because the
retry count is too high, an inflation radius that closes the only doorway, a
collision monitor polygon large enough that the robot stops before it can move at
all, a goal checker tolerance tighter than the controller can achieve, and a
progress checker whose movement allowance is shorter than a spin recovery takes.

The last one is worth flagging in advance because it is genuinely subtle. The
progress checker declares failure while a legitimate recovery is in progress, so
the robot appears to abandon recoveries halfway through for no reason.

### Exercise 7.4: the collision monitor (15 minutes)

Configure the monitor and then verify it does what you configured, rather than
assuming.

```yaml
# arc_nav/config/nav2_dwb.yaml, collision_monitor section
polygons: ["PolygonSlow", "PolygonStop"]
PolygonSlow:
  type: "circle"
  radius: 0.55
  action_type: "slowdown"
  slowdown_ratio: 0.4
  min_points: 4
PolygonStop:
  type: "circle"
  radius: 0.32
  action_type: "stop"
  min_points: 4
```

Drive the robot towards a wall and watch the velocity chain:

```
ros2 topic hz /cmd_vel_smoothed     # what Nav2 wants
ros2 topic hz /cmd_vel              # what the robot gets
ros2 topic echo /collision_monitor_state
```

| Distance to obstacle (m) | `/cmd_vel_smoothed` linear.x | `/cmd_vel` linear.x | Monitor state |
|--------------------------|------------------------------|---------------------|---------------|
| 1.5 | | | |
| 0.7 | | | |
| 0.4 | | | |
| 0.25 | | | |

The gap between the two velocity columns is the monitor doing its job, and being
able to point at it is a much better answer in your Project 2 viva than saying
the monitor is enabled.

### Exit task (10 minutes)

Commit and push, then submit:

1. Your hand drawn behaviour tree and the answer about the fallback condition.
2. Your custom recovery tree, working, with a `/behavior_tree_log` excerpt
   showing it escalating through at least two recoveries.
3. Your completed fault table with diagnoses.
4. Your completed collision monitor table.
5. One sentence stating the maximum time your recovery policy can spend before
   aborting, and how you know.

Run the self check first:

```
pytest starters/lab07/tests -v
```

---

## Troubleshooting

**The tree change had no effect.** You set the parameter without cycling the
lifecycle. Deactivate and reactivate `bt_navigator`.

**The XML fails to load.** `bt_navigator` logs the parse error but it scrolls
past quickly. Check node names against the Nav2 BT node list; a misspelled node
name is the usual cause and BehaviorTree.CPP is case sensitive.

**The robot recovers forever.** `number_of_retries` too high, or the escalation
counter is resetting when it should not. This is the failure the abort case
exists to prevent.

**The robot refuses to move at all.**

```
ros2 topic echo /collision_monitor_state
```

If the state is stopped with nothing nearby, a polygon is too large or is
configured in the wrong frame.

**Recoveries are abandoned partway through.** The progress checker's
`movement_time_allowance` is shorter than the recovery takes. A spin of 1.57
radians at 1.0 rad/s takes about 1.6 seconds plus acceleration, and an allowance
of 10 seconds is usually right.

**`/behavior_tree_log` is empty.** The topic only publishes while a goal is
active.

---

## Connection to Lab 8

You have now built the classical autonomy stack in full: perception, mapping,
localisation, planning, control, recovery and safety. Everything in it was
designed by a person and every parameter has a meaning you can state.

That is worth pausing on before next week, because next week the approach
changes completely. Instead of specifying how the robot should behave, you will
specify what counts as good behaviour and let an optimisation process find a
policy. No costmap, no planner, no behaviour tree. LiDAR in, velocity out.

The first half of that is engineering an environment the learning algorithm can
work in, which is what Lab 8 is about and which is harder and more interesting
than it sounds. The reward function you write there is the direct equivalent of
everything you configured today, and it is a great deal less explicit about it.

---

## References

**Textbooks**

Colledanchise, M. and Ögren, P. (2018). *Behavior Trees in Robotics and AI: An
Introduction*. CRC Press. Freely available from the authors. Chapters 1 to 3
cover the node semantics and the comparison with finite state machines properly.

**Papers**

Macenski, S., Martín, F., White, R. and Clavero, J. G. (2020). The Marathon 2: A
Navigation System. IROS. arXiv:2003.00368. Section 3 explains why Nav2 chose
behaviour trees, which is the design argument behind everything in this lab.

**Official documentation**

Nav2 behaviour trees and the BT node reference:
https://docs.nav2.org/behavior_trees/index.html
The node list is what you check when your XML fails to parse.

Nav2 Collision Monitor:
https://docs.nav2.org/configuration/packages/collision_monitor/index.html
Polygon types, action types and the frame configuration.

BehaviorTree.CPP: https://www.behaviortree.dev
Groot2, the graphical editor and live monitor, is worth knowing about even though
this lab edits the XML directly.

**Standards**

ISO 3691-4, Industrial trucks: safety requirements and verification, part 4,
driverless industrial trucks and their systems. You are not expected to read the
standard, which is not freely available. You are expected to know it exists and
what category of requirement it imposes.
