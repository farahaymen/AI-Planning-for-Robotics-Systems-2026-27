# ARC: nine connected ROS 2 laboratories

The student reader is [docs/ARC_Laboratories.html](docs/ARC_Laboratories.html). Download/open it in a browser; figures and equations are included. It starts with environment recovery and a beginner ROS guide. Lab 1 is preserved from the supplied reader.

| Lab | Main outcome |
|---|---|
| 1 | Send, receive and use ROS messages |
| 2 | Model, inspect and drive the robot in Gazebo |
| 3 | Occupancy mapping, SLAM and a saved map |
| 4 | Search and path tracking; connected navigation extensions |
| 5 | Nav2 localisation and goal-directed navigation |
| 6 | Dynamic programming and tabular Q-learning |
| 7 | DQN and Double DQN on robot observations |
| 8 | REINFORCE, actor-critic concepts and PPO |
| 9 | ROS policy deployment, hybrid control and evaluation |

Coursework is submitted in week 6 and uses Labs 1–4 only. See [coursework scope](docs/projects/coursework_week06.md). The course manifest is `course.json`.

## Source layout

`arc_description`, `arc_gazebo` and `arc_nav` contain robot, simulation and Nav2 integration. `arc_course` contains motion and policy-execution nodes. `arc_sensors`, `arc_mapping`, `arc_localization` and `arc_recovery` use functional names so lab renumbering does not change their purpose. `arc_lab1` is retained unchanged in purpose.

`teaching` contains runnable numerical experiments. `starters` contains student functions and reference tests. `teaching/building/robot_workshop` is the progressive student package reference; its `COLCON_IGNORE` keeps it out of the course workspace build. Students build their own copy in `~/student_ws`.

`docs/labs` contains the current explanations; `docs/reader/lab01.html` preserves the supplied Lab 1 exactly. `docs/archive` and `scripts/archive` are historical material, not current build instructions. Do not regenerate maintained packages from archived templates.

## Run in the course VM

Use ROS 2 Jazzy, Gazebo Harmonic and the supplied ARC VM. After applying this revision, run the new setup script directly once:

```bash
bash ~/arc_ws/src/arc-course/scripts/arc-setup --clean
source ~/arc_ws/install/setup.bash
```

A clean rebuild removes stale installed package names. It regenerates build/install/log, retaining source, maps, bags and student work. Setup installs a user-owned Python path file so ROS nodes can import the repository's shared teaching modules. The autostart helper supports `arc-setup --unattended`; missing system dependencies still require a manual setup.

## Verify without ROS

The numerical experiments run with Python 3.12. In an isolated development environment, install CPU PyTorch and the teaching requirements. On the course VM use the provisioned environment; do not replace ROS system dependencies casually.

```bash
python3 -m pip install 'torch>=2.8,<3' --index-url https://download.pytorch.org/whl/cpu
python3 -m pip install -r requirements-teaching.txt
python3 -m pytest starters arc_eval teaching/tests -q
```

GitHub Actions runs this suite. It cannot substitute for Gazebo acceptance on the actual VM. See [verification](verification/README.md) for recorded results and pending runtime checks.

## Build the student HTML

With Pandoc installed:

```bash
python3 scripts/build-teaching-reader.py --out docs/ARC_Laboratories.html
```

The reader embeds checked-in figures. To regenerate the new figures, run the mapping, navigation and tabular demos and pass their output directories to `scripts/make-nine-figures.py`. Do not edit generated HTML as the only source of a revision; edit the Markdown or preserved reader fragment and rebuild.

## Experimental evaluation adapter

`arc_eval/ros_nav2_env.py` is an unfinished reset adapter and is excluded from the nine-lab execution path. Its reset fails explicitly. Use Lab 5's navigation action client and Lab 9's single-episode recorder. A proximity measurement is not a physical contact sensor, and the fast simulator is not Gazebo.
