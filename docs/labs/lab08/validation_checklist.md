---
title: "Lab 08 Validation Checklist"
---

# Lab 08 validation checklist

No simulator dependency. Must pass in Tier C.

| ID | Check | How | Status |
|----|-------|-----|--------|
| V8.1 | Reference solution passes `gymnasium` check_env | | |
| V8.2 | Reference solution passes `stable_baselines3` check_env | | |
| V8.3 | Observation shape is 29 and lies inside the declared space | | |
| V8.4 | Same seed twice gives identical rollouts | | |
| V8.5 | Goal-seeking policy scores clearly above random | | |
| V8.6 | Throughput at or above 3,000 steps/s on the weakest PC | measured 3,827 on reference | |
| V8.7 | With a scenario factory and no seed, each reset draws a NEW arena | | |
| V8.8 | With an explicit seed, the arena is reproducible | | |
| V8.9 | Bug injection 1 (prev-distance ordering) gives all-zero rewards | | |
| V8.10 | Bug injection 2 (terminated on step limit) raises no error | | |
| V8.11 | Bug injection 3 (mean downsampling) erases a thin obstacle | | |
| V8.12 | Bug injection 4 (raw goal angle) passes check_env but trains worse | | |
| V8.13 | Bug injection 5 (path_length not reset) accumulates across episodes | | |
| V8.14 | Reference passes the suite, 14 tests | `python3 -m pytest starters/lab08 -q` from `~/arc_ws/src/arc-course` | |
| V8.15 | All 14 fail on the untouched skeleton, so no test passes by accident | `ARC_NAV_CORE=nav_core_skeleton python3 -m pytest starters/lab08 -q` | |
| V8.16 | A correctly completed skeleton passes all 14 | fill it in from `arc_rl/nav_core.py` and re-run | |
| V8.17 | TODO markers in the skeleton match the tests. 11 TODOs, 10 covered by at least one test; TODO 5, the domain randomisation branch, is NOT covered and the manual says so | re-check whenever either file changes | |

## Timing

| ID | Stage | Target | Actual |
|----|-------|--------|--------|
| V8.18 | Exercise 8.1, implementation | 35 min | |
| V8.19 | Exercise 8.2, four checks | 20 min | |
| V8.20 | Exercise 8.3, five injections | 15 min | |
| V8.21 | Exercise 8.4, reading `arc_eval/ros_nav2_env.py` | 10 min | |
| V8.22 | Whole session | 120 min | |

V8.7 records a defect found during development. Without it, a training run sees
only as many arenas as there are parallel environments, because SB3 seeds each
one once and never again, and students would be taught to overfit while the
curves looked healthy. Keep the test.

Sign-off: ____________  Date: ____________  VM release: ____________
