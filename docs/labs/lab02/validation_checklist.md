---
title: "Lab 2 Validation Checklist"
---

# Lab 2 validation checklist

Exercises 2.1 to 2.3 need no physics and must pass in Tier C. Exercise 2.4
requires the simulator and must pass in Tier B.

| ID | Check | How | Status |
|----|-------|-----|--------|
| V2.1 | Xacro expands without error | `xacro arc_bot.urdf.xacro > /tmp/o.urdf` | |
| V2.2 | `check_urdf` reports a single connected tree | | |
| V2.3 | `display.launch.py` shows the robot correctly in RViz | | |
| V2.4 | `view_frames` produces a tree with no orphans | | |
| V2.5 | `tf2_echo base_link laser_link` matches the URDF origin exactly | 0.10, 0, 0.12 | |
| V2.6 | Camera link and optical link from Code 2.1 expand and appear | | |
| V2.7 | The optical frame rotation is correct (z forward, x right) | `tf2_echo` | |
| V2.8 | `broken_tf.urdf.xacro` fails in the documented way | | |
| V2.9 | The documented diagnostic command actually reveals it | | |
| V2.10 | `ros2 control list_controllers` shows both controllers active | | |
| V2.11 | `ros2 control list_hardware_interfaces` names both wheel joints | | |
| V2.12 | `odom_ruler` completes and reports a ratio near 1.000 at r=0.050 | | |
| V2.13 | Ratio is 1.10 +/- 0.02 at r=0.055 | the exercise's central claim | |
| V2.14 | Ratio is 0.90 +/- 0.02 at r=0.045 | | |
| V2.15 | `arc_lab2_cpp` builds from clean in under 90 s on the weakest PC | | |
| V2.16 | The parameter change in Exercise 2.5 is visible via `ros2 param get` | | |
| V2.17 | `pytest starters/lab02/tests` passes, 8 tests | | |
| V2.18 | Wheel command limit of 20.0 rad/s exceeds the 16.3 rad/s demand | guarded by a test | |

## Timing

| ID | Stage | Target | Actual |
|----|-------|--------|--------|
| V2.19 | Exercise 2.1 | 15 min | |
| V2.20 | Exercise 2.2 | 20 min | |
| V2.21 | Exercise 2.3 | 15 min | |
| V2.22 | Exercise 2.4, three runs at 10 s each plus rebuilds | 20 min | |
| V2.23 | Exercise 2.5, including two C++ builds | 15 min | |
| V2.24 | Whole session | 120 min | |

V2.23 is the risk. Two clean C++ builds on a slow VM can exceed the slot. If
V2.15 measures above 90 seconds, reduce the task to a single rebuild and drop the
topic rename.

## Deliberate faults for Stage 1

| Fault | Expected symptom | Revealing command |
|-------|------------------|-------------------|
| `laser_joint` parent set to a non-existent link | two disconnected trees | `view_frames` |
| `laser_joint` origin z negated | scan appears below the floor in RViz | `tf2_echo` |
| Wheel separation halved in the controller YAML | robot turns at twice the commanded rate | `odom_ruler` with angular velocity |
| `robot_state_publisher` launched twice | frames flicker between two poses | `ros2 node list` |

Sign-off: ____________  Date: ____________  VM release: ____________
