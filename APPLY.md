# Everything pending, one push

Your VM is at commit `1011a77`, so rounds 3 and 6 were never pushed. This zip
contains everything outstanding. One round trip clears the backlog.

## What is in here and why

**The LiDAR fix (round 6)** — `arc_description/urdf/arc_bot.urdf.xacro`
`laser_link` had a collision cylinder and the sensor sits at its centre, so every
beam hit the sensor's own body and the whole scan returned 0.12 m, the minimum
range. Collision removed, mount raised for clearance. **This is the one that
makes the robot able to see.**

**The cmd_vel relay (round 3)** — `arc_gazebo/scripts/cmd_vel_relay.py` plus
`CMakeLists.txt`, `simulation.launch.py`, `arc_bot_controllers.yaml`
`diff_drive_controller` subscribes to TwistStamped only in this version, while
teleop, Nav2 and every course node publish unstamped Twist. Included in case it
is also unpushed; harmless if it is already there.

**arc-setup, fixed** — `scripts/arc-setup`
It died right after a successful build on
`COLCON_TRACE: unbound variable`. colcon's generated `setup.bash` reads that
variable without a default, which is fatal under `set -u`. Now disabled around
both `source` calls. Also clears stale `AMENT_PREFIX_PATH` entries, which is
what produced that wall of "path doesn't exist" warnings.

**Package cleanup** — five `package.xml` and five `setup.py`
Removed `<buildtool_depend>ament_python</buildtool_depend>`: rosdep has no key
by that name, which caused `Cannot locate rosdep definition for [ament_python]`.
The `export build_type` is what selects the build. Also removed
`tests_require`, which setuptools no longer supports and warned about on every
build.

None of these four change behaviour you have already verified.

## Push

```
cd /d F:\arc-course
xcopy /E /Y "%USERPROFILE%\Desktop\arc-pending\arc_description" arc_description\
xcopy /E /Y "%USERPROFILE%\Desktop\arc-pending\arc_gazebo"      arc_gazebo\
xcopy /E /Y "%USERPROFILE%\Desktop\arc-pending\scripts"         scripts\
xcopy /E /Y "%USERPROFILE%\Desktop\arc-pending\arc_lab1"        arc_lab1\
xcopy /E /Y "%USERPROFILE%\Desktop\arc-pending\arc_lab3"        arc_lab3\
xcopy /E /Y "%USERPROFILE%\Desktop\arc-pending\arc_lab4"        arc_lab4\
xcopy /E /Y "%USERPROFILE%\Desktop\arc-pending\arc_lab5"        arc_lab5\
xcopy /E /Y "%USERPROFILE%\Desktop\arc-pending\arc_lab7"        arc_lab7\

git add -A
git update-index --chmod=+x arc_gazebo/scripts/cmd_vel_relay.py
git update-index --chmod=+x scripts/arc-setup
git update-index --chmod=+x scripts/arc-clean
git update-index --chmod=+x scripts/course-smoke-test
git update-index --chmod=+x scripts/course-check
git commit -m "LiDAR collision fix, cmd_vel relay, arc-setup nounset fix, package cleanup"
git push
```

The `update-index --chmod=+x` lines matter. Git stores the executable bit as
file mode and Windows has no such concept, so without them every fresh clone
gets non-executable scripts and `ros2 launch` reports "not found" for a file
that is plainly there.

## Then on the VM

```bash
arc-setup
```

That is all. It pulls, repairs modes, builds, installs and verifies.

## What to expect

`course-smoke-test --headless` should now include:

```
/scan sees the room    PASS   min 0.94 m, max 11.02 m
```

rather than the old "populated". If it reports
`every beam reads the same, the LiDAR is inside geometry`, the URDF fix did not
reach the VM.

Then look at it:

```bash
ros2 launch arc_gazebo simulation.launch.py headless:=true
# second terminal
rviz2 -d $(ros2 pkg prefix arc_gazebo)/share/arc_gazebo/rviz/simulation.rviz
```

The robot spawns at (1, 1) in a 12 m room, so expect an L-shaped arc of scan
points about 1 m out to the south and west, and longer returns north and east.
Not a closed ring hugging the robot.
