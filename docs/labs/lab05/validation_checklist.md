---
title: "Lab 5 Validation Checklist"
---

# Lab 5 validation checklist

Exercise 5.2 is offline and must pass in Tier C. Exercises 5.1, 5.3 and 5.4
require the simulator and must pass in Tier B.

| ID | Check | How | Status |
|----|-------|-----|--------|
| V5.1 | Ground truth pose is available from Gazebo for `drift_meter` | | |
| V5.2 | Drift error is roughly proportional to distance over 5, 15 and 30 m | | |
| V5.3 | Drift over 30 m is large enough to be obvious, target above 0.3 m | | |
| V5.4 | Likelihood field builds from a student map in under 10 s | | |
| V5.5 | Global localisation with 2000 particles converges on the reference bag | | |
| V5.6 | Convergence happens at a distinctive feature, not in open corridor | | |
| V5.7 | 200 particles converges less reliably than 2000 | the exercise's claim | |
| V5.8 | Zero motion noise reliably FAILS to converge | particle deprivation | |
| V5.9 | SLAM Toolbox runs from the bag and produces a map | | |
| V5.10 | A loop closure triggers and is visible in the pose graph | | |
| V5.11 | SLAM map has visibly thinner walls than the Lab 4 map | | |
| V5.12 | `map_saver_cli` writes a loadable map | | |
| V5.13 | AMCL converges from a 1 m initial pose error while driving | | |
| V5.14 | AMCL final error is smaller than raw odometry error over the same loop | | |
| V5.15 | `map` to `odom` is published by AMCL and by SLAM Toolbox, never both | | |
| V5.16 | Kidnapped robot does NOT recover with default parameters | | |
| V5.17 | `pytest starters/lab05/tests` passes, 13 tests | | |

## Timing

| ID | Stage | Target | Actual |
|----|-------|--------|--------|
| V5.18 | Exercise 5.1, three drives | 15 min | |
| V5.19 | Exercise 5.2, four filter configurations | 20 min | |
| V5.20 | Exercise 5.3, SLAM run and save | 25 min | |
| V5.21 | Exercise 5.4, relocalise and compare | 20 min | |
| V5.22 | Whole session | 120 min | |

V5.10 is the highest risk item in the lab and the highest value one. Loop closure
must trigger reliably on the reference bag or the best teaching moment in the
course is lost. If it does not, record a purpose-made bag in which the robot
traverses a loop twice at low speed and make that the reference. Do this before
release rather than discovering it during a session.

V5.20 depends on the real time factor measured in Lab 3. If bag replay at 1x
exceeds the slot, replay at 2x and confirm the scan matcher still converges.

## Deliberate faults for Stage 1

| Fault | Expected symptom | Revealing command |
|-------|------------------|-------------------|
| AMCL `laser_max_range` set to 3.0 | particles never converge | compare with `/scan` range_max |
| SLAM Toolbox and AMCL both active | frames flicker, pose jumps | `ros2 run tf2_tools view_frames` |
| `use_sim_time` false on AMCL only | extrapolation errors | `ros2 param get /amcl use_sim_time` |
| Wrong map loaded for the environment | converges confidently on the wrong pose | visual in RViz |

Sign-off: ____________  Date: ____________  VM release: ____________
