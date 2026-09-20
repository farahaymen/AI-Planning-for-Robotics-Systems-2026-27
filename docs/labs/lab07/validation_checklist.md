---
title: "Lab 07 Validation Checklist"
---

# Lab 07 validation checklist

Requires Nav2 and the simulator. Must pass in Tier B.

| ID | Check | How | Status |
|----|-------|-----|--------|
| V7.1 | Default BT XML path resolves and the file is readable | `ros2 param get /bt_navigator default_nav_to_pose_bt_xml` | |
| V7.2 | `/behavior_tree_log` publishes while a goal is active | | |
| V7.3 | `arc_recovery.xml` parses and loads after a lifecycle cycle | | |
| V7.4 | Setting the param WITHOUT cycling has no effect | the manual claims this | |
| V7.5 | RoundRobin escalates rather than repeating the cheapest recovery | read the log | |
| V7.6 | `GoalUpdated` aborts an in-progress recovery on a new goal | | |
| V7.7 | Scenario 1 reproduces its documented symptom | | |
| V7.8 | Scenario 2 reproduces its documented symptom | | |
| V7.9 | Scenario 3 reproduces its documented symptom | | |
| V7.10 | Each fault is diagnosable from `/behavior_tree_log` plus the workflow | | |
| V7.11 | Progress-checker fault genuinely aborts a legitimate spin | subtle, verify | |
| V7.12 | `PolygonSlow`, added by the student in Exercise 7.4, measurably reduces `/cmd_vel` below `/cmd_vel_smoothed`. The shipped config declares only `PolygonStop` | apply the Exercise 7.4 edit first | |
| V7.13 | `stop` polygon zeroes `/cmd_vel` while Nav2 still commands motion | | |
| V7.14 | `/collision_monitor_state` reflects the active polygon | | |
| V7.15 | `pytest starters/lab07/tests` passes, 12 tests | | |

## Timing

| ID | Stage | Target | Actual |
|----|-------|--------|--------|
| V7.16 | Exercise 7.1 | 15 min | |
| V7.17 | Exercise 7.2 | 25 min | |
| V7.18 | Exercise 7.3, three scenarios | 25 min | |
| V7.19 | Exercise 7.4 | 15 min | |
| V7.20 | Whole session | 120 min | |

V7.18 is the risk. Three fault scenarios each requiring a full Nav2 restart is
three bring-ups inside 25 minutes. Measure bring-up time on the weakest PC; if it
exceeds 45 seconds, reduce to two scenarios rather than rushing the diagnosis,
which is the actual learning objective.

Sign-off: ____________  Date: ____________  VM release: ____________
