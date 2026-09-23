# Coursework 1: from messages to a planned robot route

**Submission: week 6. Assessed preparation: Labs 1–4 only.** The precise date, group arrangements, weighting and institutional submission format are set by the module team.

Build an explainable pipeline: ROS messages and inspection; a feedback-controlled movement in Gazebo; occupancy mapping and a saved SLAM map; planning and tracking through free space. Your report must connect an observed failure to a tested repair.

## Required evidence

- Your publisher/subscriber or clearly identified modification from Lab 1, with message/interface evidence.
- Your movement rule, requested and measured displacement, and sensible scan/odometry data from Lab 2.
- Your occupancy mapper, its tests, one comparison with SLAM, and a saved map image plus YAML from Lab 3.
- Your A* implementation, a valid route and a comparison of PID and Pure Pursuit tracking from Lab 4. Report path validity, goal error and what the robot actually did.
- Source files, exact commands/settings and a concise account of limitations. Attribute reused reference code and identify the functions you wrote.

The simulated movement evidence can be a screen recording together with message logs. A screenshot alone does not prove a robot moved or reached a goal. Preserve data and code rather than editing generated plots to resemble a desired outcome.

## Core scope

Nav2, reinforcement learning, Theta*, full autonomous frontier exploration and an ICP-based SLAM implementation are not required. Optional extension work should not replace the required core. The supplied frontier and ICP demonstrations illustrate components; they are not complete exploration or SLAM systems.

## Suggested marking dimensions

Assess correctness of the implementation, explanation of the data/control flow, quality of controlled experiments, diagnosis and repair, and reproducibility. Assign numerical weights using the approved module assessment policy rather than treating this document as a new grading regulation.
