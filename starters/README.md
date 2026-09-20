# Starter code

Pure Python. Nothing here is a ROS package: `starters/` carries a
`COLCON_IGNORE`, so `colcon` skips it and you run everything with `python3` from
the repository root.

```
cd ~/arc_ws/src/arc-course
python3 -m pytest starters -q
```

That runs every suite against the reference implementations, which should always
pass. To run one against your own work, set the lab's environment variable so
the tests import your file instead.

## Per-lab folders

| Folder | Lab | You edit | Variable | Tests |
| --- | --- | --- | --- | --- |
| `lab01/` | 1 | reference only (`qos_rules.py`) | | 7 |
| `lab02/` | 2 | reference only (`diffdrive.py`) | | 8 |
| `lab03/` | 3 | reference only (`rate_stats.py`) | | 7 |
| `lab04/` | 4 | `occupancy_skeleton.py` | `ARC_OCCUPANCY` | 12 |
| `lab05/` | 5 | `particle_filter_skeleton.py` | `ARC_PARTICLE_FILTER` | 13 |
| `lab06/` | 6 | `grid_tools_skeleton.py` | `ARC_GRID_TOOLS` | 7 |
| `lab07/` | 7 | reference only (`recovery.py`) | | 12 |
| `lab08/` | 8 | `nav_core_skeleton.py` | `ARC_NAV_CORE` | 14 |
| `lab09/` | 9 | `train.py`, the PPO training script | | |
| `lab10/` | 10 | `hybrid.py`, the hybrid policy | | 9 |

The Lab 8 reference is `arc_rl/nav_core.py` in the repository root rather than a
file in `lab08/`, because the same module is what Labs 9 and 10 train against.

Run one suite against your own work:

```
ARC_OCCUPANCY=occupancy_skeleton        python3 -m pytest starters/lab04 -q
ARC_PARTICLE_FILTER=particle_filter_skeleton python3 -m pytest starters/lab05 -q
ARC_GRID_TOOLS=grid_tools_skeleton      python3 -m pytest starters/lab06 -q
ARC_NAV_CORE=nav_core_skeleton          python3 -m pytest starters/lab08 -q
```

## `algorithms/`

The offline notebook. It spans Labs 3 to 5 and it is what Project 1 is built
from. No ROS, no Gazebo, no simulator, so it still works on the week the
simulator has a bad day. Four skeletons, 168 tests, and its own `README.md` with
the details.

```
ARC_PLANNERS=planners_skeleton python3 -m pytest starters/algorithms/tests/test_planners.py -q
ARC_CONTROL=control_skeleton   python3 -m pytest starters/algorithms/tests/test_control.py  -q
ARC_REACTIVE=reactive_skeleton python3 -m pytest starters/algorithms/tests/test_reactive.py -q
ARC_COVERAGE=coverage_skeleton python3 -m pytest starters/algorithms/tests/test_coverage.py -q
```

A skeleton fails its whole suite until you fill it in. That is the intended
starting point, not a broken checkout.
