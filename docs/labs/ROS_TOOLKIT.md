<span id="ros_toolkit-diagnose-the-stage-that-failed" class="anchor-alias"></span><span id="ros_toolkit-frames-time-and-recordings" class="anchor-alias"></span><span id="ros_toolkit-add-a-program-then-build-and-test-it" class="anchor-alias"></span><span id="ros_toolkit-parameters-remapping-and-namespaces" class="anchor-alias"></span><span id="ros_toolkit-choose-the-interface-that-fits-the-interaction" class="anchor-alias"></span><span id="ros_toolkit-discover-first-then-inspect-values" class="anchor-alias"></span><span id="ros_toolkit-prepare-a-terminal-then-ask-for-help" class="anchor-alias"></span><span id="ros_toolkit-ros-2-commands-you-can-use-in-your-own-projects" class="anchor-alias"></span>

# ROS 2 from the beginning: understand it, then build with it

You can begin this chapter without knowing what ROS is. We will use one small problem throughout: a program produces a distance, and another program reports whether that distance is too small. First we will understand the pieces. Then we will create the files, run the programs and inspect what happens between them.

This is a learning chapter, not a list of commands to memorise. Work through sections 1–8 before or alongside Lab 1. Return to sections 9–14 when you need configuration files, launch files or the other communication methods. The complete chapter is reference material for several sessions; you are not expected to finish it and Lab 1 within the same two hours.

Choose a part of the ROS foundations chapter

1.  [1. What problem does ROS 2 solve?](#guide-why-ros)
2.  [2. Six words, one small system](#guide-six-words)
3.  [3. A terminal is a way to give instructions](#guide-terminal-basics)
4.  [4. What is a ROS project, and what is inside it?](#guide-project-layout)
5.  [5. Create a sender and understand every part](#guide-make-sender)
6.  [6. Create the receiver and follow one message](#guide-make-receiver)
7.  [7. Register, build and run your project](#guide-build-run)
8.  [8. Inspect the system instead of guessing](#guide-inspect-system)
9.  [9. YAML: write settings as readable data](#guide-yaml)
10. [10. Topics are one of three main communication patterns](#guide-communication-types)
11. [11. Launch files: start the pieces together](#guide-launch-files)
12. [12. XML, URDF and Xacro: describe a robot’s structure](#guide-robot-files)
13. [13. Names and delivery settings: why connected programs can still be silent](#guide-names-delivery)
14. [14. Follow the failure to the correct stage](#guide-repair-guide)
15. [Documentation behind this chapter](#guide-guide-references)

## 1. What problem does ROS 2 solve?

A robot needs several kinds of software. One program reads a distance sensor. Another decides whether there is an obstacle. Another chooses a route. Another controls the wheels. We could put everything in one large program, but separating these responsibilities makes it easier to test one part, replace a sensor or reuse the same navigation code on another robot.

Once we separate the programs, they need a way to communicate. How does the obstacle program receive the sensor reading? How do we inspect the reading without editing the sensor program? How do both programs agree on what the data means? **ROS 2 provides libraries, communication rules and tools for these jobs.**

ROS stands for **Robot Operating System**. Despite that name, ROS 2 is software that runs on an operating system such as Ubuntu. In this course, Ubuntu is the operating system inside your virtual machine. ROS 2 Jazzy is the release of ROS installed on it. A **release** is a named version of a software system; using the same release keeps our examples consistent.

You still write the robot’s decision rules. ROS does not automatically know that 0.5 metres is too close, choose your algorithm or make a robot safe. It provides the connections through which your programs exchange information and the tools through which you inspect them.

### Where Python, Gazebo and RViz fit

**Python** is the programming language we use to express our logic. A **library** is reusable code that another program can import. `rclpy` is the ROS 2 Python library: it provides the functions for creating nodes, sending messages and responding to incoming information.

**Gazebo** provides a simulated world. It calculates the motion of a virtual robot and generates simulated sensor readings. This allows us to test software without a physical robot on the desk. **RViz** displays ROS data, such as the robot’s shape, a map or sensor measurements. Seeing a robot in RViz does not mean that a physics simulation is running.

For this first project, we use Python to invent distance readings. No robot model, Gazebo world or hardware is needed. Later, a sensor can supply the distances while much of our receiving logic remains the same.

## 2. Six words, one small system

Suppose our first program produces the number `0.5`, meaning a distance of half a metre. Our second program receives it and compares it with `1.0`, meaning one metre.

A **node** is a named software component that uses ROS communication. In our project, a node called `distance_sender` produces the number. A node called `distance_receiver` checks it. Each runs in its own program here, although ROS also supports multiple nodes inside one program.

A **message** is one item of information sent between components. It has a defined structure, called its **message type**. We will use `std_msgs/msg/Float32`. This type has one field named `data`, which holds a floating-point number, meaning a number that can have a fractional part. A message might contain:

<div id="guide-cb1" class="sourceCode">

``` sourceCode
data: 0.5
```

</div>

Read this as “the field named data has the value 0.5.” The type does not encode metres. Our two programs agree that the number represents metres. If one program interpreted it as centimetres, the connection might still work while the result was wrong.

A **topic** is a named communication channel carrying messages of a particular type. Our channel will be `/practice_distance`. It is a ROS name, not a file on your computer. A **publisher** sends messages to the topic. A **subscription** receives messages from it; we commonly call the receiving component a subscriber.

The sender does not need the receiver’s Python function name. Both use the same topic name and compatible message type and delivery settings. This lets us add a third observer, such as a terminal display, without editing the sender. A topic can have multiple publishers and multiple subscribers.

A **callback** is a function we register for later execution when an event occurs. Our sender has a timer callback: “every half-second, send a value.” Our receiver has a subscription callback: “when a message arrives, check its value.” A node can contain both kinds of callback and can publish and subscribe at the same time.

**The ROS graph** means the running nodes and their communication connections. The graph changes as programs start and stop. Files saved on disk are not running nodes.

## 3. A terminal is a way to give instructions

Open Ubuntu’s Terminal application. The text before the cursor may look like `ros@ros-VirtualBox:~$`. That is the **prompt**, which tells you the terminal is ready. Do not type that prompt when copying a command. Type only the command, then press Enter.

The program that reads your command is the **shell**. In these labs we use Bash. Some commands, such as `cd`, belong to the shell; others, such as `ros2`, start a separate tool. A command can take **arguments**, which supply names or values, and **options**, which change how it operates.

Try these one at a time:

<div id="guide-cb2" class="sourceCode">

``` sourceCode
pwd
ls
```

</div>

`pwd` prints your current directory, the folder this terminal is working in. `ls` lists its contents. A **path** describes a location. `/opt/ros/jazzy/setup.bash` begins with `/`, so it is an absolute path starting from the filesystem’s root. `src/hello_robot` is a relative path, interpreted from the current directory. `~` means your home directory, usually `/home/ros` on the course VM.

<div id="guide-cb3" class="sourceCode">

``` sourceCode
mkdir -p ~/beginner_ws/src
cd ~/beginner_ws
pwd
```

</div>

`mkdir` creates a directory. `-p` also creates missing parent directories and accepts a directory that already exists. `cd` changes this terminal’s current directory. The final command should print a path ending in `/beginner_ws`.

A running program may keep the terminal busy. That is normal for a ROS node: it remains available for new messages. Open another terminal for your next command. Press **Ctrl+C** in the original terminal to request that its current program stop. Opening a new terminal does not stop programs in the old one.

### Make ROS available in this terminal

<div id="guide-cb4" class="sourceCode">

``` sourceCode
source /opt/ros/jazzy/setup.bash
ros2 --help
```

</div>

`source` reads a shell script into the current shell. This script adds the locations and settings needed to find ROS tools and packages. It does not start a robot or node. Its effects apply to this terminal and programs started from it. A different terminal needs its own setup unless your shell startup files already do it.

`--help` asks a tool to explain its available commands. `ros2 --help` lists groups such as `node`, `topic`, `service` and `action`. You can ask more specifically with `ros2 topic --help`, then `ros2 topic echo --help`.

A command split across lines may end each continued line with `\`. That means the next line belongs to the same command. There must be no characters after that backslash. You may instead write the whole command on one line with spaces.

## 4. What is a ROS project, and what is inside it?

A **project** is the whole application you are building. ROS does not require one particular folder named “project.” We organise our work using workspaces and packages.

A **workspace** is a directory in which we develop and build ROS packages. A **package** groups related code and information about how to install and use it. A project may contain one package or many. The course has several packages because describing a robot, simulating it and running navigation are different responsibilities.

We will create a separate practice workspace, `~/beginner_ws`, containing one package called `hello_robot`. Lab 1 uses `~/student_ws` and `robot_workshop` for the project you extend through the course. The practice names here are deliberately different so you can try these instructions without replacing your lab work. If you only want to create one project, read this worked example and follow Lab 1’s corresponding build activity.

Before the first build, your workspace contains `src`, short for source. **Source code** is the code you edit. Building will create three more directories:

<div class="table-scroll" aria-label="Scrollable reference table" role="region" tabindex="0">

| Directory under `~/beginner_ws` | What goes there                                                   | Do I normally edit it?           |
|---------------------------------|-------------------------------------------------------------------|----------------------------------|
| `src/`                          | Your packages and their source files                              | Yes. This is your work.          |
| `build/`                        | Intermediate files produced while preparing packages              | No. Build tools manage it.       |
| `install/`                      | The prepared packages, executable registrations and setup scripts | No. Rebuild from source instead. |
| `log/`                          | Reports from build and test commands                              | Read these when something fails. |

</div>

For Python, “build” usually means preparing the package for installation and discovery; it does not mean converting all your Python into machine code. This preparation is why saving a `.py` file alone does not automatically make `ros2 run` find it.

### Create the package skeleton

Use a fresh terminal. These two dependencies are already available in the course VM:

<div id="guide-cb5" class="sourceCode">

``` sourceCode
source /opt/ros/jazzy/setup.bash
mkdir -p ~/beginner_ws/src
cd ~/beginner_ws/src
ros2 pkg create --build-type ament_python --license Apache-2.0 hello_robot --dependencies rclpy std_msgs
```

</div>

Read the last command in pieces:

<div class="table-scroll" aria-label="Scrollable reference table" role="region" tabindex="0">

| Piece                           | Meaning                                                                                     |
|---------------------------------|---------------------------------------------------------------------------------------------|
| `ros2 pkg create`               | Ask the ROS tool to create a package’s starting files.                                      |
| `--build-type ament_python`     | Use ROS’s Python package support.                                                           |
| `--license Apache-2.0`          | Record the licence chosen for this example package.                                         |
| `hello_robot`                   | Name the package. You choose this name.                                                     |
| `--dependencies rclpy std_msgs` | Record that this package needs the ROS Python library and the standard message definitions. |

</div>

A **dependency** is software your code relies on. Declaring a dependency records the requirement; it does not itself install that software. Here `rclpy` supplies the ROS functions and `std_msgs` supplies the message definition. ROS also supports C++ packages, commonly using `ament_cmake`. We are choosing Python because it suits the course’s algorithm exercises.

Run creation once. If `hello_robot` already exists, inspect and continue editing it. Do not repeatedly create the package or delete your work to follow a command again.

### Open the generated files

Use Ubuntu’s Files application to open `beginner_ws`, then `src`, then `hello_robot`. Open source files in a plain text editor.

The outer directory is the ROS package. The inner directory of the same name contains its Python code. Those two directories have different jobs:

<div class="table-scroll" aria-label="Scrollable reference table" role="region" tabindex="0">

| Path relative to `~/beginner_ws/src/hello_robot/` | Its job                                                                                                            |
|---------------------------------------------------|--------------------------------------------------------------------------------------------------------------------|
| `package.xml`                                     | Identifies the package and declares dependencies and build type.                                                   |
| `setup.py`                                        | Tells Python’s installation tools which code and data to install and which command names to create.                |
| `setup.cfg`                                       | Tells the installation tools where ROS expects the Python executables. Keep the generated settings.                |
| `resource/hello_robot`                            | A marker used to register the installed package in ROS’s package index. It may be empty.                           |
| `hello_robot/__init__.py`                         | Makes the inner directory an ordinary Python package. It can remain empty here.                                    |
| `test/`                                           | Generated test files, commonly including style and licence checks. These do not prove the robot behaves correctly. |
| `hello_robot/sender.py`                           | Your sender program. You will create it next.                                                                      |
| `hello_robot/receiver.py`                         | Your receiver program. You will create it next.                                                                    |

</div>

An **index** is a lookup mechanism. The package index helps ROS locate installed packages without searching every directory on disk. An **executable** is a program entry that can be started. In our Python package, an executable registration will say which Python function to call.

## 5. Create a sender and understand every part

Create this file in the inner Python directory:

`~/beginner_ws/src/hello_robot/hello_robot/sender.py`

Paste the following complete program into it and save it. This sends the same configurable distance every half-second; there is no trigonometry in this first example.

<div id="guide-cb6" class="sourceCode">

``` sourceCode
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class DistanceSender(Node):
    def __init__(self):
        super().__init__('distance_sender')
        self.declare_parameter('distance_m', 0.5)
        self.publisher = self.create_publisher(
            Float32, 'practice_distance', 10)
        self.timer = self.create_timer(0.5, self.send_distance)

    def send_distance(self):
        message = Float32()
        message.data = float(self.get_parameter('distance_m').value)
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = DistanceSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```

</div>

### Read the imports first

`import rclpy` makes the ROS Python library available. `from rclpy.node import Node` brings in the basic node class. `from std_msgs.msg import Float32` brings in the message class. A **class** describes the data and functions that an object can have. Creating `Float32()` makes one message object we can fill.

The spelling changes with context: Python imports `std_msgs.msg.Float32`; the ROS command line names the interface `std_msgs/msg/Float32`. Both refer to the same message definition.

### Read the node’s setup

`class DistanceSender(Node):` defines our own kind of node using the existing ROS `Node` class. `__init__` is its constructor, the function Python calls when creating an object. `self` means this particular object. `super().__init__('distance_sender')` sets up the ROS node with that name.

`declare_parameter('distance_m', 0.5)` creates a named setting with a default value. A **parameter** is a node’s configuration value. Here it is the distance to publish, measured in metres by our agreement. We use a parameter so we can try another distance without rewriting the program.

`create_publisher(Float32, 'practice_distance', 10)` creates a publisher. The first argument selects the message type. The second selects the topic. The final `10` supplies a queue-depth setting, meaning up to ten samples in the relevant history setting. It does **not** mean ten messages per second or ten subscribers.

`create_timer(0.5, self.send_distance)` asks ROS to make the callback ready every 0.5 seconds. The desired rate is `1 / 0.5 = 2` messages per second, or **2 Hz**. The computer’s workload can affect actual timing.

Notice that we give the timer `self.send_distance` without `()`. We are giving it the function to call later. Writing `self.send_distance()` would call the function immediately and pass its return value instead.

### Read what happens at each timer event

The callback creates an empty `Float32` message, reads the parameter’s current value, puts it into `message.data` and publishes the message. Reading the parameter every time is a deliberate choice: it lets later parameter changes affect new messages. A parameter change does not automatically rewrite arbitrary Python variables or timers elsewhere in a program.

`float(...)` converts the value to a floating-point number. The dot in `message.data` means “the data field belonging to this message.” The `=` sign assigns a value; it is not a mathematical claim that two expressions are equal forever.

### Read where the program begins

Our executable will call `main()`. `rclpy.init()` prepares ROS communication. `DistanceSender()` constructs our node and its publisher and timer. `rclpy.spin(node)` lets ROS keep processing ready work, including timer and message callbacks. Here “spin” does not mean turn the robot or rotate a wheel.

`try` runs the main work. `except KeyboardInterrupt` handles a keyboard interruption such as Ctrl+C. `finally` performs cleanup as the program exits. `destroy_node()` releases this node’s resources; `shutdown()` shuts down its ROS context if it is still active.

A file that only defines `main` does not call it by itself. The executable registration in section 7 will make the call. Use `ros2 run` as shown there.

## 6. Create the receiver and follow one message

Create and save:

`~/beginner_ws/src/hello_robot/hello_robot/receiver.py`

<div id="guide-cb7" class="sourceCode">

``` sourceCode
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class DistanceReceiver(Node):
    def __init__(self):
        super().__init__('distance_receiver')
        self.subscription = self.create_subscription(
            Float32, 'practice_distance', self.on_distance, 10)

    def on_distance(self, message):
        if message.data < 1.0:
            self.get_logger().warning('Object is closer than 1 metre')
        else:
            self.get_logger().info('Object is at least 1 metre away')


def main(args=None):
    rclpy.init(args=args)
    node = DistanceReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```

</div>

The constructor registers a subscription using the sender’s topic and type. `self.on_distance` is the function to call when a message arrives. ROS supplies the received message as its argument. We store the subscription on `self` so the node keeps a reference to it.

For `data: 0.5`, the comparison `0.5 < 1.0` is true, so the warning line runs. For `data: 1.5`, it is false, so the `else` line runs. Exactly `1.0` also enters `else`, because `<` means strictly less than. If the requirement included exactly one metre, we would use `<=`.

A **logger** writes messages with useful context such as severity and node name. A warning line here reports our rule’s result. It does not stop a motor or send a braking command. This example assumes finite numeric inputs; Lab 1’s fuller receiver also checks for invalid numbers before making its decision.

The receiver does not contain an endless loop repeatedly checking the topic. ROS schedules its callback when data is available while `spin` is running. This event-based structure also lets a node handle timers and other communication.

## 7. Register, build and run your project

Open `~/beginner_ws/src/hello_robot/setup.py`. Find the `entry_points` argument. Replace its empty `console_scripts` list with the two entries below. Keep the other generated arguments. This is a fragment to edit inside the file, not a replacement for the entire file:

<div id="guide-cb8" class="sourceCode">

``` sourceCode
entry_points={
    'console_scripts': [
        'send_distance = hello_robot.sender:main',
        'check_distance = hello_robot.receiver:main',
    ],
},
```

</div>

For the first entry, `send_distance` is the executable name we choose. `hello_robot.sender` means the `sender.py` module inside the inner `hello_robot` directory. `:main` names the function to call. There is no `.py` at the end of this import path.

Now compare the different names. They do not have to be identical:

<div class="table-scroll" aria-label="Scrollable reference table" role="region" tabindex="0">

| Kind of name       | Our sender example   | Where we chose it                     |
|--------------------|----------------------|---------------------------------------|
| ROS package        | `hello_robot`        | Package creation command              |
| Python source file | `sender.py`          | File we created                       |
| Executable         | `send_distance`      | Entry in `setup.py`                   |
| Node               | `/distance_sender`   | `super().__init__` in the Python code |
| Topic              | `/practice_distance` | `create_publisher` in the Python code |

</div>

### Build in a terminal that has not sourced this workspace

Use a fresh terminal and run:

<div id="guide-cb9" class="sourceCode">

``` sourceCode
source /opt/ros/jazzy/setup.bash
cd ~/beginner_ws
colcon build --symlink-install --packages-select hello_robot
```

</div>

**colcon** is the tool that coordinates building packages in a workspace. `--packages-select hello_robot` selects our package. `--symlink-install` asks it to use links where supported, which helps during Python development. It does not remove the need to rebuild when you add executables, change installation rules or add data files.

Read the last lines. A successful result reports one package finished. If the build fails, fix the first relevant error before continuing. A successful build says the package was prepared; it does not say our distance rule is correct.

### Run the two programs in separate terminals

In **Terminal A**:

<div id="guide-cb10" class="sourceCode">

``` sourceCode
source /opt/ros/jazzy/setup.bash
source ~/beginner_ws/install/local_setup.bash
ros2 run hello_robot send_distance
```

</div>

In **Terminal B**:

<div id="guide-cb11" class="sourceCode">

``` sourceCode
source /opt/ros/jazzy/setup.bash
source ~/beginner_ws/install/local_setup.bash
ros2 run hello_robot check_distance
```

</div>

The sender may show no output; it publishes without logging. The receiver should repeatedly report that the object is closer than one metre. Leave both running.

Read `ros2 run hello_robot send_distance` as “use the ROS tool to run the executable named send_distance from the installed package hello_robot.” The command looks up the package. You do not have to stand in its source directory to run it.

The first `source` loads the ROS installation. The second adds this workspace. ROS is our **underlay**, the existing foundation; our workspace is an **overlay**, the added set of packages. In the course, there is also a course workspace between ROS and your student workspace. These words describe setup order, not different kinds of source code.

After editing a running Python program, stop and restart it. A running process normally continues using the code it already loaded. After changing package installation rules, rebuild and source the workspace again in the terminals that will run it.

## 8. Inspect the system instead of guessing

Open **Terminal C** and source ROS and the practice workspace as in A. A and B must still be running.

<div id="guide-cb12" class="sourceCode">

``` sourceCode
ros2 node list
```

</div>

You should find `/distance_sender` and `/distance_receiver`. This command answers “which nodes can this terminal discover?” It does not list all source files or all installed packages. Other nodes may appear if you have other ROS programs running.

<div id="guide-cb13" class="sourceCode">

``` sourceCode
ros2 node info /distance_receiver
```

</div>

Look under subscriptions for `/practice_distance`. This answers “what is this node connected to?” Other entries are normal because ROS nodes also expose supporting interfaces.

<div id="guide-cb14" class="sourceCode">

``` sourceCode
ros2 topic list -t
ros2 topic type /practice_distance
ros2 interface show std_msgs/msg/Float32
```

</div>

The first command lists discovered topic names and types. The second asks for one topic’s type. The third reads the type’s definition: expect `float32 data`, possibly with comments. An **interface** is a shared agreement about the structure of data exchanged between programs.

<div id="guide-cb15" class="sourceCode">

``` sourceCode
ros2 topic echo /practice_distance --once
```

</div>

Expect a message containing `data: 0.5`. `echo` subscribes and prints what arrives. `--once` exits after one message. If it waits, that usually means no matching message has arrived yet, not that the terminal is frozen.

<div id="guide-cb16" class="sourceCode">

``` sourceCode
ros2 topic hz /practice_distance
```

</div>

Let this run for several seconds, then press Ctrl+C in C. Expect an observed rate around 2 Hz. This measures messages reaching this observer. A busy VM or delivery differences can make the measured rate differ from the sender’s requested rate.

### Change one input and predict the result

Keep both nodes running. In C:

<div id="guide-cb17" class="sourceCode">

``` sourceCode
ros2 param get /distance_sender distance_m
ros2 param set /distance_sender distance_m 1.5
```

</div>

`get` reads the setting. `set` requests a change. Check that the change was accepted. New receiver lines should now report that the object is at least one metre away. This happens because the sender reads its parameter each time it prepares a message. Older warning lines remain visible as history.

Set the value to `1.0` and check the boundary case, then restore `0.5`. This is a small experiment: choose an input, predict the decision, run it and compare the output with your prediction.

### Test the receiver with a value you choose

Stop only the sender in A with Ctrl+C. Keep the receiver in B. In C:

<div id="guide-cb18" class="sourceCode">

``` sourceCode
ros2 topic pub --once /practice_distance std_msgs/msg/Float32 '{data: 0.25}'
```

</div>

`pub` means publish. `/practice_distance` is the destination topic. `std_msgs/msg/Float32` is the type. `'{data: 0.25}'` supplies the fields of the message. Single quotes keep that text together as one shell argument. The text inside uses YAML, explained next. `--once` sends one message and exits after the command’s normal discovery/publication handling.

The receiver should print a warning. Repeat with `'{data: 1.0}'` and expect the other branch. We stopped the sender so its repeating values would not be mixed with our controlled test inputs.

## 9. YAML: write settings as readable data

**YAML is a text format for data.** A `.yaml` file usually contains settings that a program reads. It is not Python, and it does not execute a control algorithm. You can edit it with the same plain text editor you use for code.

The basic form is `key: value`. A **key** names an item; the value gives its contents. Indentation groups items under other items. Use spaces consistently, commonly two per level, and do not use tabs for indentation.

Here is a complete ROS parameter file for our sender:

<div id="guide-cb19" class="sourceCode">

``` sourceCode
distance_sender:
  ros__parameters:
    distance_m: 0.75
```

</div>

Read it from the outside inward: “for the node distance_sender, set the ROS parameter distance_m to 0.75.” `ros__parameters` is the required ROS parameter-file key, with **two underscores**. Its spelling is not your choice. `distance_m` is our own parameter name, chosen in Python.

`0.75` is a number with a fractional part. `10` is an integer. `true` and `false` are Boolean values, meaning yes/no settings. Quoted text such as `"metres"` is a string. These types matter: `"0.75"` is text rather than the numeric value our parameter expects. To start this declared floating-point parameter at two metres, use `2.0` rather than changing its type to an integer.

The following is a separate YAML syntax illustration, not our sender’s parameter file:

<div id="guide-cb20" class="sourceCode">

``` sourceCode
sensor:
  name: "front_laser"
  enabled: true
  offsets: [0.1, 0.0, 0.12]
```

</div>

Here `sensor` contains three entries. Square brackets describe a list of values. The compact form `{data: 0.25}` in `ros2 topic pub` is also YAML: braces enclose a mapping of field names to values on one line. The accepted keys come from the message definition, which is why we inspect the interface first.

### Save and use our parameter file

Create a configuration directory:

<div id="guide-cb21" class="sourceCode">

``` sourceCode
mkdir -p ~/beginner_ws/src/hello_robot/config
```

</div>

Save the three-line ROS parameter example as `~/beginner_ws/src/hello_robot/config/sender.yaml`. If the sender is running, stop it. In a terminal with ROS and the practice workspace sourced, run:

<div id="guide-cb22" class="sourceCode">

``` sourceCode
ros2 run hello_robot send_distance --ros-args --params-file "$HOME/beginner_ws/src/hello_robot/config/sender.yaml"
```

</div>

`--ros-args` introduces ROS-specific options. `--params-file` supplies the path to the settings file. `$HOME` expands to your home directory; double quotes preserve the full path as one argument. We use an explicit source-file path here, so this file does not yet need an installation rule.

Inspect `/practice_distance` again. Expect `0.75`. A small difference in the last decimal digits is possible because Float32 has limited numerical precision. The filename itself can change; the node selector inside must still match the running node. Renaming or namespacing a node may require updating that selector.

Editing the YAML file while the program runs does not automatically reload it. Restart with that file or use the appropriate runtime parameter command. A program can reject changes, and a parameter only affects behaviour where the implementation actually uses it.

`ros2 param dump /distance_sender` prints a running node’s parameters as YAML. You can save that output with `>`:

<div id="guide-cb23" class="sourceCode">

``` sourceCode
ros2 param dump /distance_sender > ~/beginner_ws/sender_snapshot.yaml
```

</div>

`>` is shell output redirection: it writes the output into the named file and replaces that file’s previous contents. The snapshot may include standard parameters in addition to the one we declared.

## 10. Topics are one of three main communication patterns

Publisher/subscriber is not the only way ROS nodes communicate. Choose the pattern by asking what the interaction needs. A single node can use topics, services and actions together; they are not three mutually exclusive kinds of node.

### Topics: “Here is the latest reading”

A sensor produces measurements continuously. Its publishers send messages, and interested subscribers receive them. The sender does not wait for each receiver to finish processing and return an application-level answer. Our distance stream is a topic. A camera stream and a stream of velocity commands are other examples.

The message definition has one structure, such as the `data` field of Float32. Topics can also carry occasional events; a high message rate is common but is not what makes a topic a topic.

### Services: “Please do this short operation and give me an answer”

A **service client** sends a request to a **service server**. The server performs the operation and returns a response. For example, a client might ask a node to check whether a recent sensor reading is available. The server replies with a yes/no result and an explanation.

A service is normally chosen for an operation expected to finish promptly. It does not provide the standard goal-progress-cancellation structure of an action. A service call can fail or wait if its server is absent or unable to respond.

Lab 5 implements `/check_scan` using `std_srvs/srv/Trigger`. You can inspect its structure even before running that lab:

<div id="guide-cb24" class="sourceCode">

``` sourceCode
ros2 interface show std_srvs/srv/Trigger
```

</div>

The definition separates request and response with `---`. Trigger has no request fields; its response has `bool success` and `string message`. `bool` is a true/false value, and `string` is text.

The following call belongs to **Lab 5, while its scan-service node is running**. Our two practice nodes do not provide this service:

<div id="guide-cb25" class="sourceCode">

``` sourceCode
ros2 service call /check_scan std_srvs/srv/Trigger '{}'
```

</div>

The empty braces represent an empty request, not a missing response. Discover available services with `ros2 service list -t`, then inspect the type before deciding what fields to send. There can be node parameter services in this list even before you write your own application service.

### Actions: “Work toward this goal, report progress and let me request a stop”

Navigation can take many seconds. An **action client** sends a goal to an **action server**, which decides whether to accept it. While working, the server can send feedback, such as distance remaining. When the attempt ends, it produces a result and a status indicating how it ended. The client can request cancellation; the server handles that request according to its implementation.

Accepting a goal does not mean the goal has been reached. Feedback is not the final result. Closing a client or pressing Ctrl+C in its terminal does not by itself guarantee that the server has cancelled the task.

In Lab 5, Nav2 supplies a navigation action. Nav2 is a set of ROS 2 packages for navigation. Your program becomes the client that asks it to reach a specified pose, meaning a position and orientation.

<div id="guide-cb26" class="sourceCode">

``` sourceCode
ros2 action list -t
ros2 interface show nav2_msgs/action/NavigateToPose
```

</div>

The first command shows discovered action servers when they are running. The second shows the definition if the interface package is installed. An action definition has three sections: **goal, result, feedback**, separated by two `---` lines. This file order differs from the chronological order in which you experience feedback and the final result.

Lab 5 constructs the goal step by step, including its coordinate frame and orientation. Learn the structure before copying a large goal into `ros2 action send_goal`.

<div class="table-scroll" aria-label="Scrollable reference table" role="region" tabindex="0">

| Need                               | Suitable interface | Roles                    | What comes back?                                                                                          |
|------------------------------------|--------------------|--------------------------|-----------------------------------------------------------------------------------------------------------|
| Repeated distance readings         | Topic              | Publisher and subscriber | No required application-level reply                                                                       |
| Check whether a usable scan exists | Service            | Client and server        | One response for a request                                                                                |
| Drive to a destination             | Action             | Client and server        | Acceptance/rejection, optional feedback, then outcome for an accepted goal; cancellation can be requested |

</div>

Parameters are settings attached to nodes. ROS exposes parameter operations through services, but for everyday use we manage them with `ros2 param`. A launch file is a startup description, not a fourth message-exchange pattern.

## 11. Launch files: start the pieces together

Running one command in each terminal helped us see each part. Once those parts work, a **launch file** records how to start them together. It can select packages and executables, set parameters and change names. A Python launch file describes startup actions; it is different from the Python code implementing your distance rule.

Stop the practice sender and receiver. Create a launch directory:

<div id="guide-cb27" class="sourceCode">

``` sourceCode
mkdir -p ~/beginner_ws/src/hello_robot/launch
```

</div>

Save this complete file as `~/beginner_ws/src/hello_robot/launch/practice.launch.py`:

<div id="guide-cb28" class="sourceCode">

``` sourceCode
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hello_robot',
            executable='send_distance',
            parameters=[{'distance_m': 0.75}],
            output='screen',
        ),
        Node(
            package='hello_robot',
            executable='check_distance',
            output='screen',
        ),
    ])
```

</div>

`LaunchDescription` contains the startup actions. Each `Node(...)` action requests that an executable be started. The `Node` imported from `launch_ros.actions` is a launch action; the `Node` from `rclpy.node` in our earlier programs is the class used to implement a running node. The same word appears in two different libraries for related purposes.

`parameters=[{'distance_m': 0.75}]` supplies a startup setting to the sender. Here the braces are a **Python dictionary**, because we are inside a Python file. They look similar to compact YAML, but the parser reading the file is different. `output='screen'` asks launch to show process output in the terminal.

### Make the launch file part of the installed package

In `setup.py`, add this import near the existing imports:

<div id="guide-cb29" class="sourceCode">

``` sourceCode
from glob import glob
```

</div>

`glob` finds files matching a pattern. Find the existing `data_files` list. Keep its generated entries and add the following item inside that list, with commas separating the items:

<div id="guide-cb30" class="sourceCode">

``` sourceCode
('share/' + package_name + '/launch', glob('launch/*.launch.py')),
```

</div>

This tells installation to put matching launch files in the package’s installed share directory. Without it, the file can exist in `src` while `ros2 launch` still cannot find it by package name.

In `package.xml`, add these lines alongside the existing dependencies, inside `<package>`, if they are not already present:

<div id="guide-cb31" class="sourceCode">

``` sourceCode
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
```

</div>

They record that running the launch file needs these two packages. Build again from a fresh terminal with only the ROS underlay sourced. Then source ROS and the practice overlay in a run terminal, as in section 7, and run:

<div id="guide-cb32" class="sourceCode">

``` sourceCode
ros2 launch hello_robot practice.launch.py
```

</div>

Both nodes should now start from that one command. The receiver should warn because 0.75 is less than 1.0. Ctrl+C in the launch terminal requests shutdown of the launched processes. Use `ros2 node list` from another terminal if you need to confirm they have stopped.

## 12. XML, URDF and Xacro: describe a robot’s structure

You will encounter **URDF** and **Xacro** in Lab 2. They describe robot structure; the Python nodes implement the behaviour. You do not need a robot-description file for the two distance programs we have just built.

**XML** is a text format that structures data using tags. In `package.xml`, for example:

<div id="guide-cb33" class="sourceCode">

``` sourceCode
<name>hello_robot</name>
```

</div>

`<name>` opens the element and `</name>` closes it. The text between them is its value. The following uses an **attribute**, an extra named value inside an opening tag:

<div id="guide-cb34" class="sourceCode">

``` sourceCode
<link name="base_link"/>
```

</div>

Here `link` is the element and `name="base_link"` is an attribute. The final `/>` closes an element with no nested contents. When there are nested contents, we instead finish with `</link>`.

**URDF** means **Unified Robot Description Format**. It uses XML to describe robot parts and their connections. A **link** is a rigid part. A **joint** connects one link to another and states how they can move relative to each other. A fixed joint means the connection does not move.

This complete small URDF describes a visible rectangular body:

<div id="guide-cb35" class="sourceCode">

``` sourceCode
<?xml version="1.0"?>
<robot name="small_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.4 0.3 0.2"/>
      </geometry>
    </visual>
  </link>
</robot>
```

</div>

The size gives the box’s x, y and z lengths in metres: 40 cm, 30 cm and 20 cm. `visual` describes the appearance to draw. This minimal example does not supply the mass, collision properties, actuators and simulation configuration needed for a useful moving physics model. Lab 2 adds another link and a fixed joint, then displays the model.

**Xacro** means XML macros. It adds features for reusing values and blocks of XML. A macro is a reusable definition that can generate repeated content. Xacro also supports properties, which act as named values, and arithmetic expressions. A Xacro processor expands these features into ordinary XML, commonly a URDF robot description.

For example, this is a complete small Xacro input:

<div id="guide-cb36" class="sourceCode">

``` sourceCode
<?xml version="1.0"?>
<robot name="small_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="body_length" value="0.4"/>
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${body_length} 0.3 0.2"/>
      </geometry>
    </visual>
  </link>
</robot>
```

</div>

The namespace declaration makes the `xacro:` prefix available. It identifies the vocabulary; processing this file does not require downloading that URL. The property assigns `0.4` to `body_length`. `${body_length}` asks Xacro to substitute its value. The generated box has the same dimensions as before.

If you save this example as `~/beginner_ws/small_robot.urdf.xacro`, you can expand it in a terminal with ROS sourced and Xacro installed:

<div id="guide-cb37" class="sourceCode">

``` sourceCode
xacro ~/beginner_ws/small_robot.urdf.xacro > ~/beginner_ws/small_robot.urdf
```

</div>

The command creates an output file. It does not start a simulation or open RViz. In Lab 2, a launch file processes a description and supplies it as the string parameter `robot_description` to `robot_state_publisher`, which computes relationships between robot parts from the model and joint positions.

<div class="table-scroll" aria-label="Scrollable reference table" role="region" tabindex="0">

| File or format | What it describes                             | Example use                                                  |
|----------------|-----------------------------------------------|--------------------------------------------------------------|
| `.py`          | Python instructions                           | Process a distance reading                                   |
| `.yaml`        | Structured configuration data                 | Set a node’s threshold                                       |
| `package.xml`  | ROS package metadata written as XML           | Declare dependencies                                         |
| `.urdf`        | Robot links and joints written as XML         | Describe the body and sensor mount                           |
| `.urdf.xacro`  | A description with Xacro substitutions/macros | Reuse dimensions before generating URDF                      |
| `.launch.py`   | Startup actions written in Python             | Start the model publisher and viewer                         |
| `.sdf`         | Simulation descriptions using SDFormat        | Describe a Gazebo world, models and physics-related settings |

</div>

A file extension helps you recognise the expected format, but the contents still have to obey that format. YAML, XML and Python are read by different parsers. A **parser** is software that reads text according to a format’s rules. An indentation error in YAML and a missing closing tag in XML are different kinds of parsing failure.

## 13. Names and delivery settings: why connected programs can still be silent

A **remapping** changes a ROS name for one run without editing the source. Stop previous practice nodes, then start a sender like this:

<div id="guide-cb38" class="sourceCode">

``` sourceCode
ros2 run hello_robot send_distance --ros-args -r practice_distance:=test_distance
```

</div>

`-r` introduces a remapping. The topic written as `practice_distance` in the code now resolves to `/test_distance`. A receiver still listening to `/practice_distance` will not receive that stream. You can restore the original sender command or start the receiver with the same remapping.

A **namespace** adds a shared prefix to relative ROS names. Starting both practice programs with `--ros-args -r __ns:=/team_a` makes their relative topic `practice_distance` become `/team_a/practice_distance`. The node names also gain the prefix. An absolute topic name written with an initial slash, such as `/scan`, does not gain that prefix. A namespace organises ROS names; it does not move the robot physically or transform coordinates.

**Quality of Service**, abbreviated **QoS**, means delivery settings used by publishers and subscriptions. For example, a sender may prioritise fresh sensor readings and allow some loss, or it may use reliable delivery. Both ends need compatible settings, not necessarily identical settings. Inspect them with:

<div id="guide-cb39" class="sourceCode">

``` sourceCode
ros2 topic info /practice_distance --verbose
```

</div>

A subscription requesting reliable delivery cannot match a publisher offering only best-effort reliability. Lab 2 explains this when you connect to a sensor. Our first publisher and subscriber use matching default settings, so we can learn the program structure before dealing with sensor-specific delivery choices.

The **ROS domain ID** groups participants for discovery. It is one of the settings that determines which nodes can find one another; it is not authentication or a security boundary. The course VM sets the communication environment. If programs unexpectedly cannot discover one another, compare their terminal setup and follow the course’s environment checks.

## 14. Follow the failure to the correct stage

Keep source files in `src`, build from the workspace root and run from a terminal that has sourced the installed workspace. These are three separate stages. Diagnose the one that failed.

<div class="table-scroll" aria-label="Scrollable reference table" role="region" tabindex="0">

| What you see                             | What to check first                                                                  | Why                                                                        |
|------------------------------------------|--------------------------------------------------------------------------------------|----------------------------------------------------------------------------|
| `ros2: command not found`                | Source `/opt/ros/jazzy/setup.bash`                                                   | This shell cannot locate the ROS tool.                                     |
| Package not found                        | Build successfully, then source the workspace; inspect `ros2 pkg prefix hello_robot` | ROS needs to find the installed package.                                   |
| Executable not found                     | Check `console_scripts`, rebuild, then use `ros2 pkg executables hello_robot`        | An existing `.py` file alone is not an executable registration.            |
| Python import error                      | Check module spelling and dependencies                                               | The program started but could not load required code.                      |
| Node exists but no message appears       | Check sender activity, topic name, type and QoS                                      | Being discoverable does not prove data is flowing.                         |
| Parameter changes but behaviour does not | Find where the code reads and applies that parameter                                 | The setting and the algorithm are related only through the implementation. |
| Launch file not found                    | Check its `data_files` installation rule and rebuild                                 | Launch searches the installed package.                                     |

</div>

For this Python workspace, `rosdep install --from-paths src --ignore-src -r -y`, run from `~/beginner_ws`, can resolve declared dependencies when rosdep is configured. `--from-paths src` tells it where to read package declarations; `--ignore-src` skips dependencies already supplied as source packages; `-r` continues processing after errors; `-y` accepts installation prompts. Read the final result. Recording dependencies and successfully installing them are separate steps, and installation may need network access and administrator credentials.

`colcon test --packages-select hello_robot` runs the package’s registered tests. `colcon test-result --verbose` reports the outcomes. The generated tests largely inspect style and metadata. Our input checks with 0.25, 1.0 and 1.5 are behavioural checks: they examine whether the decision agrees with its requirement.

If discovery information seems stale after stopping programs, `ros2 daemon stop` followed by `ros2 daemon start` restarts the background helper used by some command-line discovery operations. It does not restart your nodes or repair their code.

### Keep the idea, not just the command

Before running a command, say what evidence you want: “I want to know whether the node exists,” “I want to read one actual measurement,” or “I want to check what this field means.” Then choose `node list`, `topic echo` or `interface show` accordingly. This is how the commands become useful in a new project whose package names you have never seen before.

Stop the practice nodes before returning to the course labs. Use the course’s terminal setup for the course workspace and `student_ws`; this separate practice package is not a prerequisite for running Lab 1.

## Documentation behind this chapter

These links open the official documentation. Select **Jazzy** when comparing commands with the course VM. The examples above are written for this reader; the references let you check the mechanisms and explore further.

- [ROS 2 basic concepts](https://docs.ros.org/en/jazzy/Concepts/Basic.html): nodes, communication and the ROS graph.
- [Creating a workspace](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html): source, build and overlay setup.
- [Creating a package](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html): generated Python package files and executable registration.
- [Python publisher and subscriber](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html): node construction, messages and callbacks.
- [Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html): node settings, types and runtime changes.
- [Topics, services and actions](https://docs.ros.org/en/jazzy/How-To-Guides/Topics-Services-Actions.html): choosing a communication pattern.
- [Integrating launch files](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-system.html): installing and running launch descriptions.
- [Building a visual URDF model](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html) and [using Xacro](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html): robot-description syntax and expansion.
- [YAML specification](https://yaml.org/spec/1.2.2/): the data format itself. You do not need to read the specification to complete these labs.
- [Gazebo and ROS 2](https://gazebosim.org/docs/harmonic/ros2_overview/) and [SDFormat](http://sdformat.org/spec): simulation integration and simulation-description files.
