# arc-setup

One command to clone, build and verify the course workspace. Safe to re-run: it
clones on a fresh machine and pulls afterwards.

## First time on a machine

```bash
git clone https://github.com/YOUR-ORG/arc-course.git ~/arc_ws/src/arc-course
sudo install -m 0755 ~/arc_ws/src/arc-course/scripts/arc-setup /usr/local/bin/arc-setup
arc-setup
```

After that, from anywhere:

```bash
arc-setup
```

## Options

| Command | Use it when |
|---|---|
| `arc-setup` | Normal. Pull, build, install, verify. |
| `arc-setup --clean` | Builds are behaving strangely. Wipes build/install/log first. |
| `arc-setup --no-test` | You only want the build, verification comes later. |
| `arc-setup --repo URL` | Cloning from a different repository. |

Set `ARC_REPO` once in `~/.bashrc` to avoid passing `--repo`:

```bash
echo 'export ARC_REPO=https://github.com/YOUR-ORG/arc-course.git' >> ~/.bashrc
```

## What it does, and why each step is there

1. **Stops anything still running.** A leftover launch makes every later
   measurement wrong; that cost us an afternoon chasing a doubled sensor rate.
2. **Sources ROS** if the shell has not.
3. **Clones or pulls.**
4. **Repairs executable bits.** Git stores the executable bit as file mode, and a
   script authored on Windows is committed as non-executable. With
   `--symlink-install` the install tree links to that file, and `ros2 launch`
   reports a non-executable file as "not found".
5. **Normalises line endings** if `dos2unix` is present. A CRLF shebang fails
   with `bad interpreter: ^M`.
6. **Creates** `maps/ bags/ results/ .arc/`.
7. **rosdep**, then **colcon build --symlink-install**.
8. **Installs** `course-check`, `course-smoke-test`, `arc-clean` and `arc-setup`
   to `/usr/local/bin`.
9. **Verifies**: the offline test suite, then `course-check`, then
   `course-smoke-test --headless`.

Exit code 0 means the environment is ready. On failure it points at
`~/arc_ws/.arc/last-launch.log`.

## After it passes

```bash
# terminal 1
ros2 launch arc_gazebo simulation.launch.py headless:=true
# terminal 2
rviz2 -d $(ros2 pkg prefix arc_gazebo)/share/arc_gazebo/rviz/simulation.rviz
# terminal 3
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p use_sim_time:=true
# when finished
arc-clean
```

## If you prefer the raw commands

```bash
arc-clean
cd ~/arc_ws/src/arc-course && git pull
chmod +x scripts/* arc_gazebo/scripts/*
mkdir -p ~/arc_ws/{maps,bags,results,.arc}
cd ~/arc_ws
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash
sudo install -m 0755 src/arc-course/scripts/course-check      /usr/local/bin/
sudo install -m 0755 src/arc-course/scripts/course-smoke-test /usr/local/bin/
sudo install -m 0755 src/arc-course/scripts/arc-clean         /usr/local/bin/
cd src/arc-course && python3 -m pytest starters arc_eval -q
course-check && course-smoke-test --headless
```

`chmod +x` is the line people forget, and its failure mode looks nothing like
its cause.
