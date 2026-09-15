---
title: "Lab 10 Validation Checklist"
---

# Lab 10 validation checklist

Exercises 10.2 and 10.3 run in Tier C. Exercise 10.1 needs Gazebo, Tier B.

| ID | Check | How | Status |
|----|-------|-----|--------|
| V10.1 | `RosNavEnv` produces observations matching the surrogate element by element | in an identical pose | |
| V10.2 | A surrogate-trained policy loads and drives the Gazebo robot | | |
| V10.3 | A measurable gap exists between surrogate and Gazebo success | | |
| V10.4 | The gap is large enough to discuss but not total | target 10 to 40 points | |
| V10.5 | Domain randomisation costs surrogate performance | the honest expectation | |
| V10.6 | Domain randomisation narrows the surrogate-to-Gazebo gap | may be inconclusive | |
| V10.7 | Competition grammar collapses all reactive systems | measured 6.7 / 0.0 / 6.7 | |
| V10.8 | The supplied hybrid gives near-zero handovers | measured 0 in 9 of 10 | |
| V10.9 | Classical failures are collisions, not stalls | the root cause | |
| V10.10 | At least two of the four suggested fixes raise handover frequency | | |
| V10.11 | At least one fix beats both components on some metric | may be inconclusive | |
| V10.12 | A short commitment window reproduces the oscillation failure | | |
| V10.13 | `pytest starters/lab10/tests` passes, 9 tests | | |

## Timing

| ID | Stage | Target | Actual |
|----|-------|--------|--------|
| V10.14 | Exercise 10.1, Gazebo evaluation | 25 min | |
| V10.15 | Exercise 10.2, retrain plus evaluate | 20 min | |
| V10.16 | Exercise 10.3, diagnose and fix | 25 min | |
| V10.17 | Whole session | 120 min | |

V10.14 is the binding constraint in the whole course. Gazebo evaluation at 120 s
per episode is 60 minutes for 30 seeds and does not fit. Before release, decide
and document the seed count for the in-session run, and require the full 30 seed
evaluation as coursework rather than in-session work.

V10.4 needs a decision. If the measured gap turns out to be near zero, the
surrogate is too close to Gazebo for the lesson to land and the exercise needs a
harder physics setting. If it is total, the policy does not transfer at all and
there is nothing to analyse. Measure before release.

Sign-off: ____________  Date: ____________  VM release: ____________
