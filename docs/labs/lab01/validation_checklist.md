---
title: "Lab 1 Validation Checklist"
---

# Lab 1 validation checklist

Run on the weakest lab PC. Lab 1 has no simulator dependency, so it must pass in
all three graphics tiers. Status: **PASS**, **FAIL**, **BLOCKED**.

| ID | Check | How | Status |
|----|-------|-----|--------|
| V1.1 | `course-check` exits 0 | `course-check; echo $?` | |
| V1.2 | `colcon build --symlink-install` succeeds from a clean workspace | | |
| V1.3 | `range_source` publishes at 10 Hz +/- 1 | `ros2 topic hz /range` | |
| V1.4 | `range_monitor` reports a matching rate | read its log | |
| V1.5 | `ros2 param set rate_hz 2.0` changes the rate without a restart | | |
| V1.6 | Best effort publisher plus reliable subscriber gives 0 Hz and NO error | the core claim of Exercise 1.2 | |
| V1.7 | `ros2 topic info /range --verbose` shows the differing reliability lines | | |
| V1.8 | Matching the QoS restores delivery | | |
| V1.9 | `qos_rules.py` predictions agree with observed behaviour in all four cases | | |
| V1.10 | `managed_beacon` publishes nothing before `configure` | | |
| V1.11 | Topic EXISTS after `configure` but carries no data | this distinction is the lesson | |
| V1.12 | `activate` starts publishing; `deactivate` stops it without exit | | |
| V1.13 | Changing `ROS_DOMAIN_ID` hides the running node | `ros2 node list` | |
| V1.14 | `pytest starters/lab01/tests` passes, 7 tests | | |
| V1.15 | Ctrl-C leaves no orphaned processes | `ros2 node list` after | |

## Timing

| ID | Stage | Target | Actual |
|----|-------|--------|--------|
| V1.16 | Stage 0, health check and Git setup | 10 min | |
| V1.17 | Stage 1, demonstration | 15 min | |
| V1.18 | Exercise 1.1 for a median student | 20 min | |
| V1.19 | Exercise 1.2 | 20 min | |
| V1.20 | Exercise 1.3 | 15 min | |
| V1.21 | Exercise 1.4 | 10 min | |
| V1.22 | Exit task | 10 min | |
| V1.23 | Sum of the stages above | 100 min | |
| V1.24 | Whole session | 120 min | |

The stages account for 100 minutes of the 120 minute session. The remaining 20
minutes are contingency, for late arrivals, slow builds and questions.

V1.18 is the risk. This is the students' first `colcon build` and first
`package.xml`, and build errors here consume time unpredictably. Consider
shipping the package skeleton pre-created with only the node bodies left as
TODO, and measure before deciding.

## Deliberate faults for Stage 1

| Fault | Expected symptom | Revealing command |
|-------|------------------|-------------------|
| Subscriber topic name misspelled | node runs, 0 Hz, both topics in the list | `ros2 topic list` |
| Publisher QoS set best effort | 0 Hz, no error | `ros2 topic info --verbose` |
| `while True` instead of a timer | publishes, ignores parameters and Ctrl-C | `ros2 param set` fails to take effect |
| Terminal launched with a different domain | node invisible | `echo $ROS_DOMAIN_ID` |

Sign-off: ____________  Date: ____________  VM release: ____________
