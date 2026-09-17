# The LiDAR fix, corrected

## I removed the wrong element

Round 6 removed the `<collision>` from `laser_link`. That was wrong, and your
test proved it: the collision is gone from the generated SDF and the scan still
reads 0.12 m on every beam.

`gpu_lidar` raycasts against the **rendered** scene, which is built from
**visual** geometry. Collision geometry is what a CPU raycast sensor would use.
The visual cylinder was still centred exactly on the sensor origin, so every one
of the 360 beams hit the sensor's own housing.

Your `gz sdf -p` output showed both, side by side, and I misread which mattered:

```
<pose>0.1 0 0.23 ...</pose>     <- the laser VISUAL
<sensor name='arc_lidar' ...>
  <pose>0.1 0 0.23 ...</pose>   <- the sensor, same point
```

## The fix

The visual is offset 0.04 m downward, so it reads as a lidar puck sitting on the
chassis with the beam plane above it:

```
chassis top      0.170 m
housing bottom   0.175 m    (+0.005 above the chassis)
housing top      0.205 m
beam plane       0.230 m    (+0.025 above the housing)
```

Verified numerically rather than by eye this time.

## Also included

`scripts/arc-setup` with the `COLCON_TRACE: unbound variable` fix, so the script
completes and installs its own updated copy.

## Apply

```
cd /d F:\arc-course
xcopy /E /Y "%USERPROFILE%\Desktop\arc-lidar-fix\arc_description" arc_description\
xcopy /E /Y "%USERPROFILE%\Desktop\arc-lidar-fix\scripts"         scripts\
git add -A
git update-index --chmod=+x scripts/arc-setup
git commit -m "LiDAR: offset the visual below the sensor origin; gpu_lidar raycasts visuals"
git push
```

On the VM:

```bash
cd ~/arc_ws/src/arc-course && git pull
sudo install -m 0755 scripts/arc-setup /usr/local/bin/arc-setup
arc-setup
```

## Expected

```
/scan sees the room    PASS   min 0.94 m, max 11.02 m
```

If it still reports every beam identical, check the beam plane is really clear:

```bash
xacro arc_description/urdf/arc_bot.urdf.xacro fault:=none > /tmp/r.urdf
gz sdf -p /tmp/r.urdf 2>/dev/null | grep -E "<visual|<pose|sensor name" | head -20
```

The laser visual pose should now read `0.1 0 0.19`, and the sensor `0.1 0 0.23`.
Different z values. If they are equal again, the fix did not reach the VM.

## For the Lab 3 manual

This is the best fault in the course and it is real. The sensor published at the
correct 10 Hz, in the correct frame, with the correct message type, full of
plausible floats, and was useless. Diagnosing it needs
`ros2 topic echo`, not `ros2 topic hz`. And the first fix, removing collision,
looked reasonable and changed nothing, which is the more valuable half of the
lesson: GPU sensors see visuals, CPU sensors see collisions.
