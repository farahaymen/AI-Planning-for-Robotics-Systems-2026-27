# Progressive student package reference

Students create `robot_workshop` in their own workspace in Lab 1. This directory contains completed reference files and scaffolds. `COLCON_IGNORE` keeps it outside the course build.

| Current lab | Files used |
|---|---|
| 1 | distance_source.py, distance_alert.py |
| 2 | drive_distance.py, motion_rule.py; optional student_robot.urdf and model.launch.py |
| 3 extension | small_map.py; front_range.py for sensor practice |
| 5 | send_goal.py; optional scan_service.py and progress.py |
| 6–8 extensions | approach_env.py and train_approach.py |
| 9 extension | command_gate.py |

Only copy a scaffold when its lab introduces it; write the identified student functions yourself. Add executable entries and dependencies as needed, then build the student workspace. The main nine-lab RL experiments are in `teaching`, not this ROS reference package.

The one-dimensional approach environment is a smaller optional example and its models cannot replace the 29-input warehouse policy. The standalone command gate is also an extension; the live Lab 9 adapter is `arc_course.policy_driver`.
