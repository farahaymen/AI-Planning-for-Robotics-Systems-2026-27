---
title: "Golden VM Build Runbook"
subtitle: "ARC VM 2026.1 | Autonomous Robotics with ROS 2"
author: "British University in Egypt"
date: "Follow in order. Do not skip a validation gate."
---

# Golden VM build runbook

This is the procedure for building the virtual machine every student receives.
Follow it in order. Each stage ends in a **gate**: a command that must pass
before the next stage starts. Skipping a gate is how you end up at stage 9
debugging something that broke at stage 3.

Total time: **one full day** for the first build, about two hours for a rebuild
once you have done it once. Most of that is downloads and waiting.

Two rules apply throughout.

**Snapshot after every gate.** Snapshots are cheap and a failed stage then costs
you twenty minutes instead of the whole day.

**Build on the weakest machine you will deploy to, or test on it before you
finalise.** A VM that works on your development laptop and stutters in the
laboratory has not been validated.

---

## Stage 0: decide the hypervisor

Do this first, because it is the largest technical risk in the whole course and
it is cheap to test.

Gazebo Harmonic under virtualised OpenGL is the most likely thing to fail. Build
a throwaway Ubuntu 24.04 VM under each candidate and measure.

| Step | Command or action |
|------|-------------------|
| 1 | Install Ubuntu 24.04 Desktop in VirtualBox with 3D acceleration enabled |
| 2 | Install guest additions, reboot |
| 3 | `sudo apt install -y mesa-utils && glxinfo -B` |
| 4 | Record the `OpenGL renderer string` |
| 5 | Repeat in VMware Workstation Player |

**Gate 0.** At least one hypervisor reports a renderer that is not `llvmpipe`,
`softpipe` or `swrast`. If both report software rendering, you are building a
Tier B course, which is supported but changes how you brief demonstrators.

Historically VMware handles guest OpenGL more reliably than VirtualBox. Decide on
the measurement, not on that sentence.

Record the decision, with the renderer strings, in `docs/adr/`.

---

## Stage 1: the base machine

| Setting | Value | Why |
|---------|-------|-----|
| Guest OS | Ubuntu 24.04.x LTS Desktop, 64-bit | Jazzy's Tier 1 platform |
| vCPU | 4 minimum, 6 preferred | Never exceed host physical cores minus one |
| RAM | 8 GB minimum, 12 GB preferred | Host must retain at least 4 GB |
| Disk | 60 GB, dynamically allocated | Grows as used; check aggregate free space on shared machines |
| Video memory | 128 MB minimum, 256 MB preferred | With 3D acceleration enabled |
| Network | NAT | Outbound works, inbound does not, and students cannot reach each other |
| Shared clipboard | Bidirectional | Students will paste error messages to you |
| Shared folders | Disabled | Work belongs in Git, not in a folder that vanishes |

During the Ubuntu installer:

- Username `student`, host name `arc-vm`. Both are referenced by the scripts.
- Choose **Minimal installation**. Nobody needs LibreOffice Draw on a robotics VM,
  and the image is large enough already.
- Do **not** enable automatic updates. A distribution upgrade partway through the
  semester is exactly what the frozen baseline exists to prevent.

After first boot:

```bash
sudo apt update && sudo apt -y upgrade
sudo apt install -y git
sudo systemctl disable --now unattended-upgrades apt-daily.timer apt-daily-upgrade.timer
```

Install guest additions or VMware Tools now, and reboot.

**Gate 1.**

```bash
lsb_release -d              # Ubuntu 24.04
nproc                       # matches what you allocated
free -g                     # matches what you allocated
glxinfo -B | grep "OpenGL renderer"
```

**Snapshot: `01-base-os`.**

---

## Stage 2: provisioning

Clone the course repository and run the stages. Each is idempotent, so a failure
is fixed by re-running that stage rather than by starting over.

```bash
git clone https://github.com/YOUR-ORG/arc-course.git ~/arc_ws/src/arc-course
cd ~/arc_ws/src/arc-course/scripts/provision
sudo ./provision.sh
```

Or one stage at a time, which is what I would do on the first build:

```bash
sudo ./provision.sh 00      # base OS, locale, directories, environment profile
sudo ./provision.sh 10      # ROS 2 Jazzy
sudo ./provision.sh 20      # Gazebo Harmonic and ros_gz
sudo ./provision.sh 30      # Nav2, SLAM Toolbox, AMCL, ros2_control
sudo ./provision.sh 40      # PyTorch CPU, Gymnasium, Stable-Baselines3, Docker
sudo ./provision.sh 50      # course workspace, built
```

Set `ARC_REPO` before stage 50 if your repository URL differs from the default.

### What stage 40 is protecting you from

Two decisions inside it are worth knowing because both will look wrong until
they are explained.

**NumPy is pinned below 2.** ROS 2 Jazzy's Python packages on Noble are built
against NumPy 1.26. Letting pip pull NumPy 2 produces import errors in ROS
packages that surface days later and look unrelated. The constraint file at
`/etc/arc/pip-constraints.txt` is applied through `PIP_CONSTRAINT` in the
environment profile, so it holds even when a student runs `pip install`
themselves.

**PyTorch comes from the CPU wheel index.** The default PyPI wheel pulls several
gigabytes of CUDA libraries that a VM with no GPU can never use. On a 60 GB
image that is not a rounding error.

**Gate 2.** In a **new login shell**, so the profile is sourced:

```bash
course-check
echo $?                     # must be 0
```

Read the output rather than only the exit code. Confirm `ROS_DISTRO` is `jazzy`,
`ROS_AUTOMATIC_DISCOVERY_RANGE` is `LOCALHOST`, `ROS_DOMAIN_ID` is an integer,
and the reported graphics tier matches what you measured at Gate 0.

**Snapshot: `02-provisioned`.**

---

## Stage 3: does the robot actually work

`course-check` proves things are installed. It does not prove the robot moves.

```bash
course-smoke-test                 # with the Gazebo GUI
course-smoke-test --headless      # Tier B path, must also pass
```

The test launches the simulation, then asserts observable behaviour: the robot
spawns, `/scan` publishes at 10 Hz, `/odom` at 50 Hz, `/imu` at 100 Hz, `/clock`
advances and carries simulation time rather than a Unix timestamp, TF resolves
`odom` to `base_footprint` and `base_link` to `laser_link`, both controllers are
active, and a `/cmd_vel` command produces more than 0.30 m of measured
displacement.

That last check is the one that matters. Everything else can pass while the
robot sits still.

**Gate 3.** Both invocations exit 0. Keep the JSON artifacts from
`~/arc_ws/.arc/`; they are your evidence that the image was validated, and the
first thing to compare against when a laboratory machine misbehaves in week 6.

**Snapshot: `03-smoke-passed`.**

---

## Stage 4: the offline test suite

This needs no ROS and no simulator, so it also confirms the Tier C fallback.

```bash
cd ~/arc_ws/src/arc-course
python3 -m pytest starters arc_eval -q       # 100 tests
```

**Gate 4.** 100 passed, 1 skipped. The skip is the ROS-dependent adapter test and
is expected to run once ROS is sourced.

---

## Stage 5: walk the labs

Provisioning being correct is not the same as the manuals being correct. Work
through each laboratory's validation checklist, in order, on this machine.

| Laboratory | Checklist items | The item most likely to fail |
|-----------|-----------------|------------------------------|
| 1 | 15 | V1.17, first `colcon build` timing |
| 2 | 18 | V2.23, two C++ builds inside the slot |
| 3 | 22 | V3.25, recording time at the measured real time factor |
| 4 | 15 | V4.15, full bag integration under 90 s |
| 5 | 17 | V5.10, loop closure triggering reliably |
| 6 | 22 | V6.27, twenty seeded Nav2 runs inside 20 minutes |
| 7 | 15 | V7.18, three Nav2 restarts inside 25 minutes |
| 8 | 15 | V8.6, surrogate throughput on the weakest PC |
| 9 | 12 | V9.5, the sparse reward reliably failing |
| 10 | 13 | V10.14, Gazebo evaluation does not fit a session |

Record **measured** timings in the Actual column. Where an item fails, the fix is
to change the manual, not to note an exception. A manual that documents behaviour
the environment does not produce is worse than no manual.

**Gate 5.** Every checklist signed off, or a recorded decision for each failure.

**Snapshot: `05-labs-validated`.**

---

## Stage 6: the weakest machine

Take the snapshot to the worst laboratory PC and repeat Gates 2, 3 and 4 there.

Measure and record:

```bash
course-check --json | jq '.graphics_tier'
# real time factor: note /clock twice, ten wall-clock seconds apart
ros2 topic echo /clock --once; sleep 10; ros2 topic echo /clock --once
```

The real time factor governs several decisions you have already deferred: how
long a Lab 3 recording takes, whether Lab 6's twenty runs fit the session, and
whether Lab 10's evaluation must become coursework. Write the number down and
revisit those three checklist items with it.

**Gate 6.** The machine passes in whichever tier it reports, and every graded
task in the checklists is achievable in Tier B.

---

## Stage 7: freeze and export

Only now.

```bash
cd ~/arc_ws/src/arc-course/scripts/provision
sudo ./60-finalise.sh
sudo shutdown -h now
```

Stage 60 removes caches and logs, clears the machine ID and SSH host keys so that
thirty clones are not identical on the network, and zero-fills free space so the
exported appliance compresses. Expect the export to roughly halve in size.

Export as OVA named `ARC-VM-2026.1.ova`. Record alongside it:

- The SHA-256 checksum
- The hypervisor and version it was built under
- The `course-check --json` output from Gate 6
- The date and the name of whoever validated it

**Gate 7.** Import the OVA on a different machine, boot it, and run `course-check`
and `course-smoke-test`. An appliance that has never been imported has not been
tested; the export step itself can break things.

---

## Stage 8: distribution

A 20 to 30 GB appliance times a full cohort is a logistics problem, not a
download.

**Primary channel: pre-imaged laboratory machines.** Give the technicians the OVA
and have them import it on every laboratory PC before week 1. This is the only
channel that reliably works on day one.

**Secondary: a local mirror on the campus network**, for students who want it on
their own machines. Publish the checksum next to it.

**Tertiary: USB drives** at the first laboratory session, for students whose
connection cannot manage the download.

Do not rely on a cloud link. Fifty simultaneous downloads of a 25 GB file will
saturate whatever you are using.

If laboratory machines restore to a clean image between sessions, brief the
demonstrators on the consequence: **nothing inside the VM survives**, and the
last item of every weekly exit task is `git push`. This is stated in Lab 1 and in
both project specifications, and it will still catch somebody in week 2.

---

## Stage 9: during the semester

**Do not update the image.** No `apt upgrade`, no new ROS packages, no
distribution changes. The entire point of a frozen baseline is that a student in
week 10 has the same environment as in week 1, and that a result recorded in week
6 can be reproduced in week 12.

If a change is genuinely unavoidable:

1. Increment the release to `2026.2` and update `ARC_VM_RELEASE` in the profile,
   so `course-check` reports which image a machine is running.
2. Re-run Gates 2, 3 and 4.
3. Re-run any checklist item the change could touch.
4. Announce it, with the reason, and note which laboratories are affected.

**Collect the incident log.** Every `course-check` failure produces an incident ID
and a JSON artifact. Demonstrators record them. At the end of the semester that
log is your evidence base, both for which machines need replacing and for whether
the fairness guarantee held.

---

## Quick reference

```bash
# provisioning
sudo ./provision.sh                  # all stages up to 50
sudo ./provision.sh 30               # one stage, idempotent
sudo ./60-finalise.sh                # only after every gate passes

# validation
course-check                         # is everything present
course-smoke-test                    # does the robot actually work
course-smoke-test --headless         # the Tier B path
python3 -m pytest starters arc_eval -q   # 100 offline tests

# diagnosis
glxinfo -B | grep "OpenGL renderer"  # which graphics tier
echo $ROS_DOMAIN_ID                  # discovery isolation
ros2 topic hz /scan                  # is the sensor alive
ros2 control list_controllers        # are the controllers active
ros2 run tf2_tools view_frames       # is the TF tree whole
```

## Gate summary

| Gate | Passes when |
|------|-------------|
| 0 | A hypervisor gives non-software OpenGL, or Tier B is accepted deliberately |
| 1 | Base OS matches the specification and reports the allocated resources |
| 2 | `course-check` exits 0 in a new login shell |
| 3 | `course-smoke-test` exits 0 with and without the GUI |
| 4 | 100 offline tests pass |
| 5 | Every laboratory checklist signed off with measured timings |
| 6 | The weakest laboratory PC passes Gates 2, 3 and 4 |
| 7 | The exported OVA imports elsewhere and still passes |
