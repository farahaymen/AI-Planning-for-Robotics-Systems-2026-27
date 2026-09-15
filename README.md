# Autonomous Robotics with ROS 2

Course materials for a ten week autonomous robotics laboratory: mapping,
navigation and reinforcement learning. Built for ARC VM 2026.1 (Ubuntu 24.04,
ROS 2 Jazzy, Gazebo Harmonic).

## Read these first

| Document | Who for |
|----------|---------|
| `docs/proposal.md` | Reviewers. The case for the redesign, in ten pages. |
| `docs/golden_vm_build_runbook.md` | You, first. How to build and validate the VM, stage by stage. |
| `docs/00_design_baseline.md` | You. Thirty numbered decisions everything else depends on. |
| `docs/labs/lab*/manual.md` | Students. One manual per laboratory. |
| `docs/labs/lab*/validation_checklist.md` | Demonstrators. Sign off before release. |
| `docs/projects/` | Students. Project 1 and the Grand Challenge. |

## Layout

```
arc_description/     reference robot: URDF, Xacro, controllers
arc_nav/             Nav2 configuration (DWB and MPPI), launch
arc_rl/              nav_core.py (the observation contract), arenas, baselines
arc_eval/            the evaluation harness, the Nav2 adapter, competition scoring
starters/lab01..10/  student starting points and their self-check tests
docs/                manuals, checklists, projects, proposal
scripts/             course-check, build-manuals.sh
```

## Build the documents

```bash
./scripts/build-manuals.sh          # DOCX for all, plus PDF where there is maths
```

Manuals are authored in Markdown and generated with pandoc. Edit the Markdown,
never the DOCX. Maths-bearing manuals also emit a LaTeX PDF, because LibreOffice
silently drops equations when exporting a Word file to PDF.

## Run the tests

```bash
python3 -m pytest starters arc_eval -q     # 87 tests, no ROS required
```

Everything under `starters/` and `arc_eval/` is pure Python and runs without ROS
or Gazebo, which is also why every graded task has a fallback for machines where
Gazebo will not start.

## Train and evaluate

```bash
export PYTHONPATH=.
python3 starters/lab09/train.py --reward dense_progress --steps 200000
python3 -m arc_eval.runner --config arc_eval/configs/gap_follow.yaml
python3 -m arc_eval.runner --compare results/a.json results/b.json
```

Measured on the reference machine: 200,000 PPO steps in 1.9 minutes on one CPU
thread. Training happens in the NumPy surrogate; evaluation and deployment happen
through ROS against Gazebo, sharing one frozen observation contract.

## Build the Golden VM

```bash
cd scripts/provision
sudo ./provision.sh          # stages 00 to 50, each idempotent
course-check                 # gate: is everything present
course-smoke-test            # gate: does the robot actually move
sudo ./60-finalise.sh        # only after every gate passes
```

Follow `docs/golden_vm_build_runbook.md` rather than these five lines. It has the
validation gates, the snapshot points and the two decisions in stage 40 that
prevent NumPy from breaking ROS.

## Before releasing anything to students

Nothing requiring ROS 2 or Gazebo has been validated. Work through each
laboratory's validation checklist on the weakest laboratory PC, in graphics
Tier B, and sign it off. A manual that documents behaviour the environment does
not produce is worse than no manual.
