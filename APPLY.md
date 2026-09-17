# Fix round 6: the LiDAR was blind

## What was wrong

```
$ ros2 topic echo /scan --once --field ranges | ...
min: 0.11999999731779099   max: 0.11999999731779099
```

Every one of the 360 beams returned exactly 0.12 m, the configured minimum
range. The sensor origin was inside solid geometry.

`laser_link` carried a `<collision>` cylinder of radius 0.035 m, and the LiDAR
sensor sits at that link's centre. Every ray hit the sensor's own collision
cylinder before leaving it. In RViz this looks like a tight red ring locked
around the robot that never changes as it drives.

A sensor mount needs a visual so you can see it and an inertial so the solver is
happy. It does not need collision.

## The fix

Removed the collision element from `laser_link`, and raised the mount from
`base_height/2 + 0.02` to `base_height/2 + 0.06`:

```
base_link centre    0.110 m
chassis top         0.170 m
laser beam plane    0.230 m   (was 0.190 m)
clearance           0.060 m above the chassis   (was 0.020 m)
```

The old 0.02 m gap was the height of the laser body itself, so the beam plane
was grazing the chassis even once the collision was gone.

## The smoke test missed this, and now does not

The old check asked whether `/scan` carried values. An all-minimum-range scan is
full of values and completely blind, so it passed. `ros2 topic hz` was a perfect
10 Hz throughout.

It now checks the SPREAD of ranges and fails when every beam reads the same,
reporting `every beam reads the same, the LiDAR is inside geometry`. Verified
against the exact numbers from your VM.

## Apply

```
cd /d F:\arc-course
xcopy /E /Y "%USERPROFILE%\Desktop\arc-fixes-6\arc_description" arc_description\
xcopy /E /Y "%USERPROFILE%\Desktop\arc-fixes-6\scripts"         scripts\
git add -A
git commit -m "Remove laser collision geometry; smoke test checks scan spread"
git push
```

On the VM:

```bash
cd ~/arc_ws/src/arc-course && git pull
sudo install -m 0755 scripts/course-smoke-test /usr/local/bin/course-smoke-test
sudo install -m 0755 scripts/arc-clean         /usr/local/bin/arc-clean
cd ~/arc_ws && colcon build --packages-select arc_description && source install/setup.bash
```

## Verify

```bash
arc-clean
ros2 launch arc_gazebo simulation.launch.py headless:=true
```

Second terminal:

```bash
ros2 topic echo /scan --once --field ranges | tr ',' '\n' | grep -oE '[0-9]+\.[0-9]+' | sort -n | awk 'NR==1{min=$1} {max=$1} END {print "min:", min, " max:", max}'
```

The robot spawns 1 m from two walls in a 12 m room, so expect min around 0.9 and
max around 11. Then in RViz the scan should trace the room outline and change
shape as you drive.

```bash
arc-clean
course-smoke-test --headless
```
