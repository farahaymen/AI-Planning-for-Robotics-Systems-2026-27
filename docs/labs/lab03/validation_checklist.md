---
title: "Lab 3 Validation Checklist"
---

# Lab 3 validation checklist

This is the first lab that hard-depends on Gazebo, so Tier B validation is not
optional. Every exercise must complete headless.

| ID | Check | How | Status |
|----|-------|-----|--------|
| V3.1 | `course-check` reports Tier B with 3D acceleration off, as shipped | `course-check` | |
| V3.2 | Simulation launches headless in Tier B | `headless:=true` | |
| V3.3 | Robot spawns at the expected pose | | |
| V3.4 | `/scan` at 10 Hz +/- 1, `frame_id` is `laser_link` | | |
| V3.5 | `/odom` at 50 Hz +/- 5, `frame_id` is `odom` | | |
| V3.6 | `/imu` at 100 Hz +/- 10 | | |
| V3.7 | `/clock` published and advancing | | |
| V3.8 | `/cmd_vel` moves the robot and odometry follows | | |
| V3.9 | `sensor_doctor` with `use_sim_time:=true` shows lag under 0.1 s | | |
| V3.10 | With `use_sim_time:=false` it shows lag near the Unix epoch and errors | the teaching moment | |
| V3.11 | `broken_a` reproduces its documented symptom | | |
| V3.12 | `broken_b` reproduces its documented symptom | | |
| V3.13 | `broken_c` reproduces its documented symptom | | |
| V3.14 | Each fault is diagnosable by the stated command alone | | |
| V3.15 | `ros2 bag record` with MCAP produces a readable bag | | |
| V3.16 | `ros2 bag info` lists all six recorded topics | | |
| V3.17 | Scan count is approximately 10x the duration in seconds | | |
| V3.18 | `ros2 bag play --clock` drives simulation time for consumers | | |
| V3.19 | TF resolves during replay with the simulator stopped | requires `/tf_static` | |
| V3.20 | A 3-minute bag is under the repository size limit, or LFS is configured | | |
| V3.21 | Real time factor measured and recorded per graphics tier | | |
| V3.22 | `pytest starters/lab03/tests` passes, 7 tests | | |

## Timing

| ID | Stage | Target | Actual |
|----|-------|--------|--------|
| V3.23 | Launch and verification | 15 min | |
| V3.24 | Exercise 3.2, three broken worlds | 25 min | |
| V3.25 | Exercise 3.3, drive and record | 20 min | |
| V3.26 | Whole session | 120 min | |

V3.25 is the risk and it compounds with V3.21. A real time factor of 0.4 means a
three minute recording takes seven and a half minutes of wall clock. Measure the
factor on the weakest PC first, then set the required bag duration from it rather
than the reverse.

V3.20 needs a decision before release: bags of a useful length will exceed a
normal Git repository limit. Either configure LFS on the course organisation or
collect bags through the VLE. Decide before week three, not during it.

Sign-off: ____________  Date: ____________  VM release: ____________
