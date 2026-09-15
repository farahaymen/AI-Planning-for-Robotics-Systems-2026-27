---
title: "Lab 4 Validation Checklist"
---

# Lab 4 validation checklist

Lab 4 has no live simulator dependency and must pass in graphics Tier C.

| ID | Check | How | Status |
|----|-------|-----|--------|
| V4.1 | Reference bag reads without ROS running | `read_run` on `reference_run.mcap` | |
| V4.2 | Scan messages report 360 beams and the expected angle increment | | |
| V4.3 | Quaternion to yaw conversion matches `tf2_echo` on the same bag | | |
| V4.4 | Nearest pose matching produces monotonic timestamps | | |
| V4.5 | Reference solution builds a recognisable map of the warehouse | visual | |
| V4.6 | The map matches the Gazebo world geometry within one cell at the walls | overlay | |
| V4.7 | Removing the max-range guard reproduces the phantom ring | Exercise 4.3 | |
| V4.8 | The ring appears at the configured max range, not elsewhere | | |
| V4.9 | Omitting the sensor offset shifts the map by 0.10 m, visibly | | |
| V4.10 | `l_free = -0.85` visibly erodes walls | Experiment 4.4 row 2 | |
| V4.11 | `l_max = 50` prevents a moved obstacle from clearing | Experiment 4.4 row 4 | |
| V4.12 | Exported PGM and YAML load in `nav2_map_server` and render in RViz | | |
| V4.13 | The exported map has correct orientation, not flipped or mirrored | | |
| V4.14 | `pytest starters/lab04/tests` passes, 12 tests | | |
| V4.15 | Full bag integration completes in under 3 minutes on the weakest PC | | |

## Timing

| ID | Stage | Target | Actual |
|----|-------|--------|--------|
| V4.16 | Exercise 4.1 | 15 min | |
| V4.17 | Exercise 4.2, implementation | 30 min | |
| V4.18 | Experiment 4.4, six map rebuilds | 20 min | |
| V4.19 | Exercise 4.5 | 15 min | |
| V4.20 | Whole session | 120 min | |

V4.15 and V4.18 compound. Six rebuilds at three minutes each is eighteen minutes
of pure compute inside a twenty minute slot. Measure V4.15 first; if it exceeds
90 seconds, subsample the bag for the parameter study and say so in the manual.
The per-beam Python loop is the bottleneck and vectorising it is a legitimate
extension task for stronger students.

## Deliberate fault for Stage 1

| Fault | Expected symptom | Revealing check |
|-------|------------------|-----------------|
| Max range treated as a hit | ring of obstacles at 12 m | look at the map |
| Row and column swapped | map transposed | compare against the world |
| Sensor offset omitted | 10 cm blur along headings | overlay with ground truth |
| Prior set to non-zero | whole map biased before any data | `coverage()` on a fresh map |

Sign-off: ____________  Date: ____________  VM release: ____________
