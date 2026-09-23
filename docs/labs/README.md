# Start here: Your robotics laboratory workspace

Welcome. In these labs you will teach a small simulated robot to sense its
surroundings, build a map and move towards a goal. You will first learn how its
programs exchange information. Later, you will compare programmed navigation
with a controller that learns from experience. You do not need previous ROS
experience; we introduce each tool when you need it.

This page is your guide to starting a session and getting back to a working
state if something breaks. Keep it open beside the lab. The commands below
belong **inside the Ubuntu course virtual machine**, not in Windows PowerShell
or your host computer's terminal.

## 1. Find your workspace

Start the supplied **ARC VM 2026.1** and log into Ubuntu with the account your
instructor provided. Open Terminal from the application menu, or press
Ctrl+Alt+T. A terminal accepts commands and displays their output.

You may see a prompt like `ros@ros-VirtualBox:~$`. Type only the command after
the prompt. Do not copy the prompt itself. Press Enter to run a line. A program
that is still running may not give you the prompt back; keep that terminal open
and use another terminal when the lab asks for one.

```bash
cd ~/arc_ws
pwd
ls
```

`cd` changes your current directory, `pwd` prints it, and `ls` lists its contents.
The symbol `~` means your home directory. For the account `ros`, the workspace
is normally `/home/ros/arc_ws`. We use `~` so the instructions also work with
another account name.

| Place | What you will find there |
|---|---|
| `~/arc_ws` | The workspace: the main folder used to build and run the course software |
| `~/arc_ws/src/arc-course` | The course repository: source code, lab text and helper programs |
| `~/arc_ws/build`, `install`, `log` | Files created by the build process; these can be regenerated |
| `~/arc_ws/maps`, `bags`, `results` | Maps, recorded robot data and other saved work |
| `~/student_ws/src/robot_workshop` | Your own package, created in Lab 1 and extended throughout the labs |
| `~/student_ws/build`, `install`, `log` | Generated files for your student workspace |
| `~/arc_ws/src/arc-course/results`, `models`, `runs` | Results and learning artifacts written by the later Python labs |

If `cd ~/arc_ws` reports “No such file or directory,” use the setup procedure
in section 3. Do not create a second workspace with a guessed name.

## 2. Prepare every new terminal

Once the workspace exists, run these three lines **in each new terminal**:

```bash
source /opt/ros/jazzy/setup.bash
source ~/arc_ws/install/setup.bash
cd ~/arc_ws/src/arc-course
```

The first line tells this terminal where the installed ROS 2 tools are. The
second adds the course packages built in your workspace. `source` loads these
settings into the current terminal; it does not reinstall software. The last
line puts you in the course folder so relative filenames and Python modules
work as shown in the labs. Successful `source` commands normally print nothing.

Check that the setup worked:

```bash
ros2 pkg prefix arc_lab1
```

You should see a path ending in `install/arc_lab1`. If the package is not found,
repeat the two `source` commands. If `install/setup.bash` itself is missing,
the workspace needs to be built using section 3.

From the build activity in Lab 1 onward, add your own installed package after
the two source commands above:

```bash
source ~/student_ws/install/local_setup.bash
```

Use this only after you have created and built that workspace. Run course Python
helpers from the course repository root as before. Your editable student code
lives in `~/student_ws/src/robot_workshop`, and its build command runs from
`~/student_ws`. Distinguishing source, build and execution locations prevents
many apparent missing-file errors.

The [ROS command guide](ROS_TOOLKIT.md) explains the general tools behind these
steps. The `arc-` commands prepare the supplied environment; the labs also teach
you to create, build and inspect packages using ordinary ROS 2 commands.

Each lab labels its running terminals **A**, **B**, **C** and so on. These are
ordinary terminal windows, not different machines. Prepare each one using the
same three lines. Keep the course's communication settings unchanged; all
terminals for your robot must use the same `ROS_DOMAIN_ID`. The supplied VM
uses local discovery so your robot's programs stay within your VM.

## 3. Set up the course, or rebuild it after an update

Use this on your first session if the instructor asks you to prepare the
workspace, after receiving course updates, or when a missing/broken package
needs rebuilding. It is **not** necessary every time you open a terminal.

First stop active experiments using section 4. If you have edited source files,
save them in your editor and make a backup before updating:

```bash
cp -a ~/arc_ws/src/arc-course "$HOME/arc-course-backup-$(date +%Y%m%d-%H%M%S)"
```

Run the backup command only if that source folder exists. It copies your course
folder to a new dated folder in your home directory; it does not change the
original. It can take time if your folder includes recorded results or models.

In a terminal, run:

```bash
source /opt/ros/jazzy/setup.bash
arc-setup
```

`arc-setup` retrieves course updates, resolves workspace dependencies, builds the
packages, installs course helper commands and runs checks. It needs network
access when retrieving updates/dependencies. If it asks for your Ubuntu password
to install a dependency, type it and press Enter; password characters are not
displayed. Run `arc-setup` as your normal account, not as `sudo arc-setup`.

Wait for it to finish. The build and simulator checks can take several minutes
on a VM. Read the final result: **“Environment ready”** means its required checks
passed. If a step says `FAILED`, keep the error output and use section 5.
Do not interpret “build complete” as proof that the simulator checks passed.

After a successful setup, prepare your current terminal again:

```bash
source ~/arc_ws/install/setup.bash
cd ~/arc_ws/src/arc-course
```

The setup script runs in its own process, so its temporary settings do not
automatically update your existing terminals. Re-source other terminals too.

If `arc-setup` is not found but the repository is present, run its source copy:

```bash
bash ~/arc_ws/src/arc-course/scripts/arc-setup
```

If both the command and the repository are missing, or `/opt/ros/jazzy` does not
exist, ask for the supplied course VM or its documented restoration procedure.
`arc-setup` rebuilds the course workspace; it does not install an entire Ubuntu,
ROS and Gazebo system onto an empty machine.

### Start setup automatically when you open the VM desktop

Run once in the VM as your normal account:

```bash
arc-autostart enable
arc-autostart status
```

On each desktop login, a terminal runs `arc-setup --unattended`: repository
update, dependency check, build, offline tests, environment check and simulator
smoke test. It installs course commands under `~/.local/bin`, so it never needs
sudo for them. Wait for the result before starting a lab. If new Ubuntu packages
are needed, setup stops and tells you to run the normal `arc-setup` manually
once to install them. The terminal shows any failure and saves its full output
under `~/arc_ws/.arc/startup-*.log`.

This begins **when the desktop user logs in**. A powered-on VM waiting at its
login screen does not have a desktop session in which to show the password
prompt or run graphics checks. To stop automatic setup, run
`arc-autostart disable`. You can always run `arc-setup` yourself.

## 4. Stop and clean an experiment

When a lab program is running, click its terminal and press **Ctrl+C** once.
Allow it to stop. Do this for the simulator, viewers, keyboard driver, recorder
and any other programs you started. Stop a recording this way so it can finish
writing its files.

Then, in a prepared terminal:

```bash
arc-clean
arc-clean --check
```

`arc-clean` stops remaining course processes and refreshes the ROS discovery
cache. It does **not** delete source code, maps or recordings. Its `--check`
option only looks for leftover course processes. If a program is still listed,
find its terminal and stop it. A manual `ros2 topic pub` or a separate recorder
may need Ctrl+C even after the course cleanup. Your new student executables
also need to be stopped in their own terminals; `arc-clean --check` checks
course process patterns, not every program you may write. Confirm the expected
nodes disappeared using `ros2 node list` after discovery has updated.

If the installed cleanup command is missing, use:

```bash
bash ~/arc_ws/src/arc-course/scripts/arc-clean
```

Use this cleanup before starting a fresh simulation, switching to recorded
playback, or rerunning a simulator check. Start one simulator at a time. Close
the keyboard driver before starting an autonomous driver so they do not send
competing commands to the same robot.

## 5. Recover in small steps

Most lab problems do not require reinstalling the VM. Choose the row that
matches what you actually see:

| What you see | First action | If it still fails |
|---|---|---|
| `ros2: command not found` | Source `/opt/ros/jazzy/setup.bash` | If that file is missing, the base VM needs restoration/support |
| `Package ... not found` | Source `~/arc_ws/install/setup.bash` | Run `arc-setup`, then source it again |
| Duplicate robots, old windows, or unexpected publishers | Ctrl+C in running terminals, then `arc-clean` | Close remaining manual programs and retry one launch |
| `No module named teaching` or `arc_rl` | `cd ~/arc_ws/src/arc-course` | Check that the updated files are present; use the course Python environment |
| A bag/output folder already exists | Choose a new output name, such as `_v2` | Keep the earlier result for comparison |
| A build fails, or rebuilt packages still appear stale | Save your work and stop all experiments | Use the clean rebuild below |
| A Git conflict appears during setup | Stop the update and keep the message | Ask your instructor to help reconcile edits; repeated setup cannot choose the correct code for you |
| Gazebo/RViz exits or the scan looks wrong | Clean up, then run the environment checks below | Keep the first error and health report for your instructor |

For a clean rebuild, open a **fresh terminal**, stop existing experiments, and
run:

```bash
source /opt/ros/jazzy/setup.bash
arc-setup --clean
```

This removes and regenerates `~/arc_ws/build`, `install` and `log`. It keeps
the source folder and saved data, and it also updates the repository as normal
setup does. Save/back up source edits first. After success, source the new
`install/setup.bash` in every terminal you will reuse.

Check the environment independently when needed. Run these commands one at a
time so you can read the result before continuing:

```bash
source ~/arc_ws/install/setup.bash
course-check
course-smoke-test --headless
```

Run the smoke test only with other experiments stopped and after resolving
required `course-check` failures. `course-check` checks the installed environment;
the smoke test launches the robot and checks behaviour. Here `--headless` hides
the Gazebo window; the lidar still needs working graphics inside the VM.

If setup fails while running Python exercise tests, keep the named failing test.
An incomplete student algorithm can fail a test even with a healthy ROS
installation. A clean rebuild preserves those source edits, so it will not
magically complete or repair the algorithm.

For a broken student build, keep `~/student_ws/src` intact. In a fresh terminal,
source ROS and the course underlay, `cd ~/student_ws`, then rerun
`colcon build --symlink-install --packages-select robot_workshop`. Correct the
first reported error. `arc-setup --clean` rebuilds the course workspace only;
it does not rebuild your student package. If a generated student installation
must be replaced, close its running programs, rename its `build`, `install`
and `log` directories to dated backup names in the file manager, and build
again. Preserve `src`, parameter snapshots and result/model files.

When asking for help, provide the command you ran and its first error, together
with the health report path printed by `course-check`. Failed simulator launch
output is saved at `~/arc_ws/.arc/last-launch.log`; setup's Python test output is
at `~/arc_ws/.arc/offline-tests.log`.

If the whole VM must be restored from the instructor's snapshot or re-imported,
copy both source workspaces, maps, bags and learning results **outside the VM first**.
Restoring a snapshot can remove everything created after it. Return to section
3 once the supplied VM is running again.

## 6. Begin the labs

The essential daily routine is: **prepare each terminal → run the lab → save
your evidence → Ctrl+C and clean up**. Setup and clean rebuild are recovery tools,
not extra commands to run between every activity.

| Lab | Build on | Observable result |
|---|---|---|
| [1](lab01/manual.md) | Running a program | Two ROS nodes exchange and use messages |
| [2](lab02/manual.md) | Messages | A robot moves in Gazebo with measured feedback |
| [3](lab03/manual.md) | Motion and sensors | Known-pose mapping, SLAM and a saved map |
| [4](lab04/manual.md) | Saved map | A planned and tracked route; connected extensions |
| [5](lab05/manual.md) | Planning and control | A Nav2 goal reaches a final action result |
| [6](lab06/manual.md) | Decision rules | A grid robot learns a policy from experience |
| [7](lab07/manual.md) | Returns and Q-learning | DQN and Double DQN robot replays |
| [8](lab08/manual.md) | Value approximation | REINFORCE and PPO policy learning |
| [9](lab09/manual.md) | Trained policy | A recorded ROS policy episode and transfer evaluation |

**Coursework is submitted in week 6 and assesses Labs 1–4 only.** Lab 5 introduces Nav2; it is not a dependency for that submission. Lab number and submission week are separate labels. Optional extensions are not required to finish the core investigation.

Each lab connects a working robot experiment with a component you build.
Write the input/output requirement first, implement a small version, then use
known inputs to check it. The reference listings help you diagnose differences;
you should still be able to create the files, explain their interfaces and adapt
them without importing a course-specific node.

The reading is more detailed than the code you need to type in two hours. Use
it before and during the session, and return to the mathematical examples when
you need them. Prioritise the baseline, student build and one well-explained
repair. Additional comparisons can continue after class. The optional extensions are there when you are
ready to go further. For your notebook, keep the command, what you expected,
what happened and the change that explains the difference. Start with Lab 1;
you do not need to learn every term in this table in advance.

If you want to revisit how sourcing works, the official
[ROS 2 environment tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)
explains the underlying settings. The course recovery commands above are
specific to this VM and repository.
