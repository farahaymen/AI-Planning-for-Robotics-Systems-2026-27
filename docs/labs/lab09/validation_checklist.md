---
title: "Lab 09 Validation Checklist"
---

# Lab 09 validation checklist

CPU only. Must pass in Tier C.

| ID | Check | How | Status |
|----|-------|-----|--------|
| V9.1 | PPO 200k steps completes in under 4 min on the weakest PC | measured 1.9 min on reference | |
| V9.2 | Two runs in parallel do not exceed the VM's memory | | |
| V9.3 | TensorBoard serves and shows all three named curves | | |
| V9.4 | `dense_progress` reaches at least 35 % success on held-out seeds | measured 40.0 | |
| V9.5 | `sparse_safe` reliably produces the freezing failure | measured 0.0 % | |
| V9.6 | The freeze diagnostic shows mean path under 2 m over all episodes | measured 1.07 m | |
| V9.7 | Timeout terminations dominate for `sparse_safe` | measured 19 of 30 | |
| V9.8 | Classical floor reproduces at about 40 % | | |
| V9.9 | Evaluation seeds 500 to 529 were never generated during training | | |
| V9.10 | At least two of the four suggested fixes raise success above 0 | | |
| V9.11 | `deterministic=True` evaluation beats stochastic evaluation | | |
| V9.12 | Results are reproducible from a fixed seed | | |

## Timing

| ID | Stage | Target | Actual |
|----|-------|--------|--------|
| V9.13 | Exercise 9.1, two runs in background | 25 min | |
| V9.14 | Exercise 9.2, evaluation | 25 min | |
| V9.15 | Exercise 9.3, diagnosis plus one retrain | 20 min | |
| V9.16 | Whole session | 120 min | |

V9.5 is the item the whole lab depends on. If the sparse reward happens to
succeed on your machine, the central teaching moment disappears. Verify across at
least three training seeds before release, and if it is not reliable, adjust the
constants so the arithmetic in the pre-lab reading matches the observed outcome.

V9.15 requires a full retrain inside 20 minutes. At 1.9 min it fits with room to
spare, but confirm on the weakest PC rather than assuming.

Sign-off: ____________  Date: ____________  VM release: ____________
