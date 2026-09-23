# Current code and lab alignment

The authoritative sequence is `course.json`. Lab 1's HTML fragment is byte-preserved. Labs 2–9 render from their Markdown sources.

| Lab | Runtime entry | Core source |
|---|---|---|
| 2 | ros2 run arc_course drive_distance | arc_course/arc_course/drive_distance.py |
| 3 | python3 -m teaching.mapping_demo; teaching.map_bag; arc_nav slam launch | starters/lab03/occupancy.py |
| 4 | python3 -m teaching.navigation; teaching.extensions | teaching/navigation.py; starters/algorithms/control.py |
| 5 | arc_nav navigation launch; robot_workshop send_goal | arc_nav/config; student action client |
| 6 | python3 -m teaching.tabular | teaching/tabular.py |
| 7 | python3 -m teaching.deep_q; teaching.rollout | teaching/deep_q.py; arc_rl/nav_core.py |
| 8 | python3 -m teaching.policy_gradient; teaching.train_policy | return update and Stable-Baselines3 PPO |
| 9 | ros2 run arc_course policy_driver; teaching.rollout | teaching/ros_contract.py; starters/lab09/hybrid.py |

Documented custom command names, launch arguments and Python options are checked against source by the test suite. Numerical tests cover mapping, search, tracking, learning targets, observation conventions and evaluation isolation. Runtime ROS/Gazebo acceptance remains a separate gate.
