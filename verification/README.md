# Verification and VM acceptance

The release report and offline test log record checks actually executed in the authoring environment. ROS 2 and Gazebo are not installed there. Consequently live launch, SLAM, Nav2 and ROS policy execution are **pending verification on the course VM**. Source inspection and numerical simulation do not establish those runtime results.

## Automated checks

Run from the updated repository inside the provisioned VM:

```bash
bash scripts/course-lab-check
```

This runs the offline suite, checks the environment and runs the existing Gazebo movement/sensor smoke test. A previous simulator must be stopped first. `--offline` runs only the numerical and source checks. A zero exit from the offline mode is not a ROS acceptance result.

## Complete the integration acceptance record

Use each lab's integrated experiment and retain the actual output beside this table. Mark a row passed only after observing its stated result.

| Lab | Runtime evidence required | Status at packaging |
|---|---|---|
| 1 | Original publisher/subscriber exchange and student package build | Existing content preserved; not rerun under ROS here |
| 2 | One active simulation; both controllers active; valid scans; drive_distance reports roughly 0.6 m displacement and stops | Pending VM |
| 3 | Map grows during a driven route; map saver writes a readable YAML/PGM pair; student mapper consumes the bag | Pending VM; numerical mapper tested |
| 4 | Search returns valid free-space segments; robot replay approaches its goal; repeat on the map actually saved in Lab 3 | Numerical demo tested; VM-produced map pending |
| 5 | AMCL aligns scans to the saved map; NavigateToPose reaches a successful final result; failed goal is reported correctly | Pending VM |
| 6 | Tabular training writes arrays and a replay; independent evaluation reports its outcome | Tested; see report |
| 7 | DQN and Double DQN targets checked; training, save/load and rollout execute | Tested with a short development budget; no trained-success claim |
| 8 | REINFORCE update and PPO training/save/load/replay execute | Tested with development budgets; no trained-success claim |
| 9 | Fresh Gazebo episode moves under policy_driver; stale inputs or a competing command publisher suppress motion; CSV contains actual statuses | ROS pending; pure observation contract and surrogate replay tested |

For Lab 9, first use the gap baseline, then a trained model. Stop other velocity sources. Restart the simulator for each manually controlled reset and record the initial pose. A proximity stop is not a physical collision measurement. The driver uses a steady wall-clock timer so its timeout and zero-command checks continue when the simulation clock is paused. The controller also has a command timeout.

## Development results versus learning results

The recorded smoke training runs used budgets only long enough to exercise code paths. Their weights are not included as student solutions. Their failure or timeout in a rollout is retained in the report. Do not distribute them as trained navigation solutions. Students train their own models and report measured success over held-out layouts and independent training seeds.

`demos` contains numerical outputs supporting the figures and checks. These files are generated examples, not files for students to implement. The static illustrations in the reader are explicitly labelled where they are analytical examples rather than observed performance.
