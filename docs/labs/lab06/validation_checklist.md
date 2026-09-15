---
title: "Lab 6 Validation Checklist"
subtitle: "Nothing in the manual is released to students until every item below passes in the Golden VM"
---

# Lab 6 validation checklist

Run this on the weakest available lab PC, in graphics Tier B (headless Gazebo
server with RViz only), because Tier B is what the fairness guarantee in the
design baseline commits to. Record the date, the VM release and the machine.

Status codes: **PASS**, **FAIL**, **BLOCKED** (depends on a failed earlier item).

## Environment

| ID | Check | How | Status |
|----|-------|-----|--------|
| V6.1 | `course-check` exits 0 | `course-check; echo $?` | |
| V6.2 | Graphics tier detected correctly | compare against `glxinfo -B` by hand | |
| V6.3 | Simulation launches and the robot spawns | `ros2 launch arc_gazebo simulation.launch.py` | |
| V6.4 | `/scan` publishes at 10 Hz +/- 1 | `ros2 topic hz /scan` | |
| V6.5 | `/odom` publishes at 50 Hz +/- 5 | `ros2 topic hz /odom` | |
| V6.6 | TF resolves `map` to `base_footprint` | `ros2 run tf2_ros tf2_echo map base_footprint` | |
| V6.7 | No duplicate TF publishers | `ros2 run tf2_tools view_frames` | |

## Nav2 bring-up

| ID | Check | How | Status |
|----|-------|-----|--------|
| V6.8 | All lifecycle nodes reach `active` within 30 s | `ros2 lifecycle get /bt_navigator` and each server | |
| V6.9 | `navigate_to_pose` action is advertised | `ros2 action info /navigate_to_pose -t` | |
| V6.10 | `goal_client.py` reaches a goal and exits 0 | `ros2 run arc_nav goal_client 4.5 2.0` | |
| V6.11 | World reset hook returns the robot to a known pose | `Nav2EvalEnv._reset_world`, currently NotImplemented | |
| V6.12 | Ten seeded runs complete without manual intervention | `python3 -m arc_eval.runner --config arc_eval/configs/lab06_dwb.yaml` | |
| V6.13 | Metrics JSON contains every required info key | inspect `results/lab06_dwb.json` | |

## Exercise correctness

| ID | Check | How | Status |
|----|-------|-----|--------|
| V6.14 | Costmap subscriber receives a message with transient local QoS | Code 6.2 as printed | |
| V6.15 | Costmap subscriber receives nothing with default QoS | deliberately break it; the manual claims this | |
| V6.16 | `pytest starters/lab06/tests` passes, 7 tests (includes the shape check) | | |
| V6.17 | Student A* on the raw map hugs walls; on the costmap it does not | visual check in RViz | |
| V6.18 | `use_astar: true` changes the NavFn path measurably | compare `/plan` lengths | |
| V6.19 | Inflation radius 0.90 blocks the narrow doorway | the manual asserts this; confirm the doorway width supports it | |
| V6.20 | Inflation radius 0.25 succeeds where 0.90 fails | | |
| V6.21 | MPPI config loads and the controller runs at the configured rate | `ros2 topic hz /cmd_vel_smoothed` | |
| V6.22 | MPPI runs in real time on the weakest PC with `batch_size: 1000` | measure controller frequency under load | |

## Timing

| ID | Check | Target | Actual |
|----|-------|--------|--------|
| V6.23 | Stage 0 health check and bring-up | under 10 min | |
| V6.24 | Exercise 6.1 for a median student | under 20 min | |
| V6.25 | Exercise 6.2 | under 20 min | |
| V6.26 | Experiment 6.3, three runs | under 15 min | |
| V6.27 | Exercise 6.4, twenty seeded runs total | under 20 min | |
| V6.28 | Whole session with a demonstrator present | under 120 min | |

V6.27 is the item most likely to fail. Twenty navigation runs at up to 120
seconds each is 40 minutes of wall clock in the worst case. Before release,
either reduce the time limit, shorten the mission, or run the two controller
configurations concurrently in separate domains. Decide this from a measurement
rather than an estimate.

## Deliberate faults for Stage 1

Each of these must be verifiable through the diagnostic workflow in the
troubleshooting section, and each must be recoverable within five minutes.

| Fault | Injected by | Expected symptom | Revealing command |
|-------|-------------|------------------|-------------------|
| Wrong `sensor_frame` in the obstacle layer | edit `nav2_dwb.yaml` | costmap ignores the LiDAR | `ros2 run tf2_ros tf2_echo` |
| `controller_server` left inactive | remove from lifecycle manager list | path plans, robot still | `ros2 lifecycle get` |
| Collision monitor polygon radius set to 1.5 m | edit `nav2_dwb.yaml` | robot refuses to move at all | `ros2 topic hz /cmd_vel` |
| `wheel_radius` in the controller set to 0.055 | edit `arc_bot_controllers.yaml` | odometry drifts, AMCL diverges | compare `/odom` with ground truth |
| Costmap subscriber QoS set to volatile | edit the starter | subscription exists, never fires | `ros2 topic info --verbose` |

Rotate the fault between sessions so that the answer does not travel between
groups.

## Sign-off

Lab 6 is releasable when every item above is PASS, or when a FAIL has been
resolved by changing the manual rather than by noting an exception. A manual that
documents behaviour the environment does not produce is worse than no manual.

Validated by: ____________  Date: ____________  VM release: ____________
Machine: ____________  Graphics tier: ____________
