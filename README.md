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
| `docs/labs/lab01..lab10/manual.md` | Students. One manual per laboratory, ten in all. |
| `docs/labs/lab*/validation_checklist.md` | Demonstrators. Sign off before release. |
| `docs/projects/project1_specification.md` | Students. Mapping and navigation system, 25 percent, weeks 3 to 6. |
| `docs/projects/project2_specification.md` | Students. The Grand Challenge, 40 percent, weeks 9 to 12. |
| `docs/references.md` | Everyone. Further reading, one section per lab plus Applications. |

## Layout

```
arc_description/     reference robot: URDF, Xacro, controllers
arc_gazebo/          worlds, simulation launch, the warehouse arena
arc_nav/             Nav2 configuration (DWB and MPPI), launch, behaviour trees
arc_lab4/5/7/        per-lab ROS packages
arc_rl/              nav_core.py (the observation contract), arenas, baselines
arc_eval/            the evaluation harness, the Nav2 adapter, scoring, configs/
starters/lab01..10/  per-lab starting points and their self-check tests
starters/algorithms/ the offline notebook, Labs 3 to 5, what Project 1 is built from
docs/                manuals, checklists, projects, references, proposal
scripts/             the installed commands, plus provisioning and figure generation
```

`arc_rl`, `arc_eval` and `starters` are plain Python packages, not ROS packages.
Each carries a `COLCON_IGNORE` and no `package.xml`, so `colcon` skips them and
`ros2 run` will not find them. Read `starters/README.md` for how to run a suite
against your own work.

## Installed commands

`scripts/` is on the path in the Golden VM.

| Command | What it does |
|---------|--------------|
| `arc-setup` | Clone or update the course workspace, build it, verify it. |
| `arc-clean` | Stop every process a course launch starts. |
| `course-check` | Is everything present. The gate that decides whether a VM is usable. |
| `course-smoke-test` | Does the robot actually move. The gate `course-check` cannot be. |
| `arc-map-view` | Watch a SLAM map build without OpenGL, for machines RViz will not serve. |
| `arc-drive` | Drive the robot around a building on its own, so it can be mapped. |

## Build the documents

```bash
./scripts/build-manuals.sh          # DOCX for all, plus PDF where there is maths
```

Manuals are authored in Markdown and generated with pandoc. Edit the Markdown,
never the DOCX. Maths-bearing manuals also emit a LaTeX PDF, because LibreOffice
silently drops equations when exporting a Word file to PDF.

## Run the tests

```bash
python3 -m pytest starters arc_eval -q     # 282 pass, 1 skipped, no ROS required
```

The skip is a Nav2 integration test that needs a live stack; it runs with
`ARC_INTEGRATION=1` and a navigation stack up.

Everything under `starters/` and `arc_eval/` is pure Python and runs without ROS
or Gazebo, which is also why every graded task has a fallback for machines where
Gazebo will not start.

## Train and evaluate

```bash
export PYTHONPATH=.
python3 starters/lab09/train.py --reward dense_progress --steps 200000
python3 -m arc_eval.runner --config arc_eval/configs/gap_follow.yaml
python3 -m arc_eval.runner --config arc_eval/configs/lab09_dense.yaml
python3 -m arc_eval.runner --compare results/gap_follow.json results/ppo_dense_progress.json
```

Training writes TensorBoard logs to `runs/` and the policy to `models/`.
`arc_eval/configs/` holds one YAML per benchmarked system.

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
