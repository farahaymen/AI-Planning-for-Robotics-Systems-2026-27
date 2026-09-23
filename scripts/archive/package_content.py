"""Package contents for generate_packages.py. Kept separate so the generator
itself stays readable."""

from __future__ import annotations

from pathlib import Path


def build(root: Path, write, package_xml, cmake_lists, python_package) -> None:

    # =====================================================================
    # arc_description
    # =====================================================================
    write(root, "arc_description/package.xml", package_xml(
        "arc_description", "ARC reference robot description and controllers",
        "ament_cmake",
        ["xacro", "robot_state_publisher", "joint_state_publisher_gui", "rviz2",
         "ros2_control", "ros2_controllers", "diff_drive_controller",
         "joint_state_broadcaster", "gz_ros2_control"]))
    write(root, "arc_description/CMakeLists.txt",
          cmake_lists("arc_description", ["urdf", "config", "launch", "rviz"]))

    write(root, "arc_description/launch/display.launch.py", '''
"""Show the robot description in RViz. No physics, no simulator.

    ros2 launch arc_description display.launch.py
    ros2 launch arc_description display.launch.py model:=broken_tf.urdf.xacro
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model = LaunchConfiguration("model")
    share = FindPackageShare("arc_description")
    description = Command(["xacro ", PathJoinSubstitution([share, "urdf", model])])

    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="arc_bot.urdf.xacro"),
        DeclareLaunchArgument("gui", default_value="true"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description}], output="screen"),
        # Without joint states, non-fixed joints have no transform and the robot
        # appears in RViz as a pile of shapes at the origin.
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", PathJoinSubstitution([share, "rviz", "display.rviz"])]),
    ])
''')

    # =====================================================================
    # arc_gazebo
    # =====================================================================
    write(root, "arc_gazebo/package.xml", package_xml(
        "arc_gazebo", "Simulation worlds and bring-up for the ARC reference robot",
        "ament_cmake",
        ["ros_gz_sim", "ros_gz_bridge", "arc_description", "xacro",
         "robot_state_publisher", "controller_manager", "rviz2"]))
    write(root, "arc_gazebo/CMakeLists.txt",
          cmake_lists("arc_gazebo", ["worlds", "launch", "config", "rviz"]))

    write(root, "arc_gazebo/launch/simulation.launch.py", '''
"""Start Gazebo, spawn the robot, bridge the topics, activate the controllers.

    ros2 launch arc_gazebo simulation.launch.py
    ros2 launch arc_gazebo simulation.launch.py headless:=true     # graphics Tier B
    ros2 launch arc_gazebo simulation.launch.py world:=broken_a    # Lab 3 fault

The broken_* worlds are the standard warehouse with a deliberate SENSOR fault,
because the faults live in the robot description rather than in the world. The
launch file maps the world name onto a xacro argument so the manual's interface
stays simple.
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

FAULTS = {"broken_a": "a", "broken_b": "b", "broken_c": "c"}


def launch_setup(context, *args, **kwargs):
    world_arg = LaunchConfiguration("world").perform(context)
    headless = LaunchConfiguration("headless")

    fault = FAULTS.get(world_arg, "none")
    world_file = "arc_warehouse.sdf" if world_arg in FAULTS else f"{world_arg}.sdf"

    desc_share = FindPackageShare("arc_description")
    gz_share = FindPackageShare("arc_gazebo")

    robot_description = Command([
        "xacro ", PathJoinSubstitution([desc_share, "urdf", "arc_bot.urdf.xacro"]),
        " fault:=", fault,
    ])

    gz_args = [PathJoinSubstitution([gz_share, "worlds", world_file])]
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
        launch_arguments={
            # -s is server only, no GUI. -r starts the simulation running.
            "gz_args": [*gz_args, " -r -s --headless-rendering"],
        }.items(),
        condition=IfCondition(headless),
    )
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
        launch_arguments={"gz_args": [*gz_args, " -r"]}.items(),
        condition=UnlessCondition(headless),
    )

    state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}])

    spawn = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-topic", "robot_description", "-name", "arc_bot",
                   "-x", "1.0", "-y", "1.0", "-z", "0.08"])

    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        parameters=[{"config_file": PathJoinSubstitution(
            [gz_share, "config", "ros_gz_bridge.yaml"]), "use_sim_time": True}])

    # Controllers must be spawned AFTER the robot exists in the simulator,
    # otherwise the controller manager has no hardware to claim and the
    # spawner fails in a way that looks like a configuration error.
    joint_state = Node(package="controller_manager", executable="spawner",
                       arguments=["joint_state_broadcaster"], output="screen")
    diff_drive = Node(package="controller_manager", executable="spawner",
                      arguments=["diff_drive_controller"], output="screen")

    return [
        gz, gz_gui, state_publisher, bridge, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn,
                                           on_exit=[joint_state])),
        RegisterEventHandler(OnProcessExit(target_action=joint_state,
                                           on_exit=[diff_drive])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="arc_warehouse",
                              description="arc_warehouse, broken_a, broken_b or broken_c"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="true for graphics Tier B"),
        OpaqueFunction(function=launch_setup),
    ])
''')

    write(root, "arc_gazebo/config/ros_gz_bridge.yaml", '''
# Topics crossing the Gazebo/ROS boundary. A topic absent from this file exists
# in Gazebo and is invisible to ROS, which is one of the Lab 3 faults.
- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS

- ros_topic_name: "/scan"
  gz_topic_name: "/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS

- ros_topic_name: "/imu"
  gz_topic_name: "/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS
''')

    write(root, "arc_gazebo/worlds/arc_warehouse.sdf", '''
<?xml version="1.0" ?>
<!-- ARC warehouse. Deliberately low polygon: this must run on the weakest
     laboratory PC in graphics Tier B, not look impressive in a screenshot. -->
<sdf version="1.9">
  <world name="arc_warehouse">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Perimeter: a 12 x 12 m room. -->
    <model name="walls">
      <static>true</static>
      <link name="link">
        <collision name="n"><pose>6 12 0.5 0 0 0</pose>
          <geometry><box><size>12 0.15 1.0</size></box></geometry></collision>
        <visual name="nv"><pose>6 12 0.5 0 0 0</pose>
          <geometry><box><size>12 0.15 1.0</size></box></geometry></visual>
        <collision name="s"><pose>6 0 0.5 0 0 0</pose>
          <geometry><box><size>12 0.15 1.0</size></box></geometry></collision>
        <visual name="sv"><pose>6 0 0.5 0 0 0</pose>
          <geometry><box><size>12 0.15 1.0</size></box></geometry></visual>
        <collision name="w"><pose>0 6 0.5 0 0 0</pose>
          <geometry><box><size>0.15 12 1.0</size></box></geometry></collision>
        <visual name="wv"><pose>0 6 0.5 0 0 0</pose>
          <geometry><box><size>0.15 12 1.0</size></box></geometry></visual>
        <collision name="e"><pose>12 6 0.5 0 0 0</pose>
          <geometry><box><size>0.15 12 1.0</size></box></geometry></collision>
        <visual name="ev"><pose>12 6 0.5 0 0 0</pose>
          <geometry><box><size>0.15 12 1.0</size></box></geometry></visual>
      </link>
    </model>

    <!-- A partition with a 1.2 m doorway. Lab 6 plans through this gap, and it
         is the one that closes when the inflation radius is set too large. -->
    <model name="partition">
      <static>true</static>
      <link name="link">
        <collision name="lower"><pose>6 2.4 0.5 0 0 0</pose>
          <geometry><box><size>0.15 4.8 1.0</size></box></geometry></collision>
        <visual name="lowerv"><pose>6 2.4 0.5 0 0 0</pose>
          <geometry><box><size>0.15 4.8 1.0</size></box></geometry></visual>
        <collision name="upper"><pose>6 9.0 0.5 0 0 0</pose>
          <geometry><box><size>0.15 6.0 1.0</size></box></geometry></collision>
        <visual name="upperv"><pose>6 9.0 0.5 0 0 0</pose>
          <geometry><box><size>0.15 6.0 1.0</size></box></geometry></visual>
      </link>
    </model>

    <model name="pillars">
      <static>true</static>
      <link name="link">
        <collision name="p1"><pose>3 8 0.5 0 0 0</pose>
          <geometry><cylinder><radius>0.35</radius><length>1.0</length></cylinder></geometry></collision>
        <visual name="p1v"><pose>3 8 0.5 0 0 0</pose>
          <geometry><cylinder><radius>0.35</radius><length>1.0</length></cylinder></geometry></visual>
        <collision name="p2"><pose>9 4 0.5 0 0 0</pose>
          <geometry><cylinder><radius>0.35</radius><length>1.0</length></cylinder></geometry></collision>
        <visual name="p2v"><pose>9 4 0.5 0 0 0</pose>
          <geometry><cylinder><radius>0.35</radius><length>1.0</length></cylinder></geometry></visual>
        <collision name="p3"><pose>9.5 9.5 0.5 0 0 0</pose>
          <geometry><box><size>1.2 0.8 1.0</size></box></geometry></collision>
        <visual name="p3v"><pose>9.5 9.5 0.5 0 0 0</pose>
          <geometry><box><size>1.2 0.8 1.0</size></box></geometry></visual>
      </link>
    </model>
  </world>
</sdf>
''')

    # =====================================================================
    # arc_nav
    # =====================================================================
    write(root, "arc_nav/package.xml", package_xml(
        "arc_nav", "Nav2 configuration, behaviour trees and bring-up for the ARC robot",
        "ament_cmake",
        ["navigation2", "nav2_bringup", "nav2_mppi_controller",
         "nav2_collision_monitor", "slam_toolbox", "robot_localization",
         "nav2_map_server", "rviz2", "arc_description"]))
    write(root, "arc_nav/CMakeLists.txt",
          cmake_lists("arc_nav", ["config", "launch", "behavior_trees", "maps", "rviz"]))

    write(root, "arc_nav/behavior_trees/arc_recovery.xml", '''
<?xml version="1.0"?>
<!-- Lab 7, Code 7.1. Point bt_navigator here, then deactivate and reactivate it:
     the tree is loaded on configuration, so setting the parameter alone changes
     nothing. -->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">

      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
        </RateController>
        <FollowPath path="{path}" controller_id="FollowPath"/>
      </PipelineSequence>

      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <RoundRobin name="RecoveryActions">
          <Sequence name="ClearingActions">
            <ClearEntireCostmap name="ClearLocal"
              service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap name="ClearGlobal"
              service_name="global_costmap/clear_entirely_global_costmap"/>
          </Sequence>
          <Spin spin_dist="1.57"/>
          <Wait wait_duration="4.0"/>
          <BackUp backup_dist="0.30" backup_speed="0.10"/>
        </RoundRobin>
      </ReactiveFallback>

    </RecoveryNode>
  </BehaviorTree>
</root>
''')

    write(root, "arc_nav/launch/slam.launch.py", '''
"""Run SLAM Toolbox in online asynchronous mode against a live simulation.

    ros2 launch arc_nav slam.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare("slam_toolbox"), "launch",
                 "online_async_launch.py"])),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "slam_params_file": PathJoinSubstitution(
                    [FindPackageShare("arc_nav"), "config", "slam_toolbox.yaml"]),
            }.items()),
    ])
''')

    write(root, "arc_nav/config/slam_toolbox.yaml", '''
slam_toolbox:
  ros__parameters:
    use_sim_time: true
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan
    mode: mapping

    resolution: 0.05
    max_laser_range: 12.0
    minimum_time_interval: 0.2
    transform_timeout: 0.2
    map_update_interval: 2.0

    # Loop closure. Lab 5 depends on this triggering reliably on the reference
    # bag; if it does not, lower the coarse search distance before anything else.
    do_loop_closing: true
    loop_search_maximum_distance: 3.0
    loop_match_minimum_chain_size: 10
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    minimum_travel_distance: 0.3
    minimum_travel_heading: 0.3
    scan_buffer_size: 10
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
''')

    write(root, "arc_nav/launch/localization.launch.py", '''
"""Map server plus AMCL against a saved map.

    ros2 launch arc_nav localization.launch.py map:=$HOME/arc_ws/maps/slam_map.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map", description="Path to the saved map YAML"),
        DeclareLaunchArgument("params", default_value="dwb"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "launch",
                 "localization_launch.py"])),
            launch_arguments={
                "map": LaunchConfiguration("map"),
                "use_sim_time": "true",
                "params_file": PathJoinSubstitution(
                    [FindPackageShare("arc_nav"), "config",
                     ["nav2_", LaunchConfiguration("params"), ".yaml"]]),
            }.items()),
    ])
''')
    write(root, "arc_nav/maps/README.md",
          "Saved maps live here. `map_saver_cli -f <name>` writes a .pgm and a .yaml.\n")

    # =====================================================================
    # Minimal but valid RViz configurations. A launch file that passes -d to a
    # missing or malformed config fails in a confusing way, so these ship.
    # =====================================================================
    def rviz(fixed_frame: str, displays: list[str]) -> str:
        blocks = {
            "grid": "    - Class: rviz_default_plugins/Grid\n      Enabled: true\n      Name: Grid\n",
            "robot": ("    - Class: rviz_default_plugins/RobotModel\n      Enabled: true\n"
                      "      Name: RobotModel\n      Description Topic:\n"
                      "        Value: /robot_description\n"),
            "tf": "    - Class: rviz_default_plugins/TF\n      Enabled: true\n      Name: TF\n",
            "scan": ("    - Class: rviz_default_plugins/LaserScan\n      Enabled: true\n"
                     "      Name: LaserScan\n      Size (m): 0.03\n      Topic:\n"
                     "        Value: /scan\n        Reliability Policy: Best Effort\n"),
            "map": ("    - Class: rviz_default_plugins/Map\n      Enabled: true\n"
                    "      Name: Map\n      Topic:\n        Value: /map\n"
                    "        Durability Policy: Transient Local\n"),
            "path": ("    - Class: rviz_default_plugins/Path\n      Enabled: true\n"
                     "      Name: Plan\n      Topic:\n        Value: /plan\n"),
        }
        body = "".join(blocks[d] for d in displays)
        return f"""
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Displays:
{body}  Global Options:
    Fixed Frame: {fixed_frame}
    Frame Rate: 20
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/SetInitialPose
      Topic:
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Value: /goal_pose
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 18
      Name: Current View
      Target Frame: <Fixed Frame>
Window Geometry:
  Height: 800
  Width: 1200
"""

    write(root, "arc_description/rviz/display.rviz", rviz("base_footprint", ["grid", "robot", "tf"]))
    write(root, "arc_gazebo/rviz/simulation.rviz", rviz("odom", ["grid", "robot", "tf", "scan"]))
    write(root, "arc_nav/rviz/arc_navigation.rviz", rviz("map", ["grid", "robot", "tf", "scan", "map", "path"]))
    write(root, "arc_nav/rviz/slam.rviz", rviz("odom", ["grid", "robot", "tf", "scan", "map"]))

    # =====================================================================
    # Lab 1
    # =====================================================================
    python_package(root, "arc_lab1", "Lab 1: ROS 2 nodes, QoS and managed lifecycle",
                   ["rclpy", "std_msgs", "lifecycle_msgs"],
                   {"range_source": "arc_lab1.range_source:main",
                    "range_monitor": "arc_lab1.range_monitor:main",
                    "managed_beacon": "arc_lab1.managed_beacon:main"})

    write(root, "arc_lab1/arc_lab1/range_source.py", '''
#!/usr/bin/env python3
"""Publishes a fake range reading at 10 Hz, the rate of the real LiDAR.

Exercise 1.2 asks you to change the publisher QoS to best effort and observe
that the default subscriber then receives nothing, with no error anywhere.
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class RangeSource(Node):
    def __init__(self):
        super().__init__("range_source")
        # Declared, not hard coded, so `ros2 param set` works at runtime.
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("amplitude", 2.0)
        rate = self.get_parameter("rate_hz").value

        self.publisher = self.create_publisher(Float32, "range", 10)
        # A timer, not a while loop. rclpy.spin hands the thread to an executor;
        # a while loop here starves every other callback and the node looks
        # alive while silently ignoring everything.
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.k = 0
        self.get_logger().info(f"publishing /range at {rate} Hz")

    def tick(self):
        amplitude = self.get_parameter("amplitude").value
        msg = Float32()
        msg.data = float(amplitude * (1.5 + math.sin(self.k * 0.1)))
        self.publisher.publish(msg)
        self.k += 1


def main():
    rclpy.init()
    node = RangeSource()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
''')

    write(root, "arc_lab1/arc_lab1/range_monitor.py", '''
#!/usr/bin/env python3
"""Subscribes to /range and reports the rate it is actually receiving.

Reporting zero explicitly is the point. A subscriber that says nothing when it
receives nothing is indistinguishable from one that has crashed.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class RangeMonitor(Node):
    def __init__(self):
        super().__init__("range_monitor")
        self.subscription = self.create_subscription(Float32, "range", self.on_range, 10)
        self.count = 0
        self.create_timer(2.0, self.report)

    def on_range(self, msg):
        self.count += 1
        if msg.data < 1.0:
            self.get_logger().warn(f"close range {msg.data:.2f}")

    def report(self):
        self.get_logger().info(f"{self.count / 2.0:.1f} Hz over the last 2 s")
        self.count = 0


def main():
    rclpy.init()
    node = RangeMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
''')

    write(root, "arc_lab1/arc_lab1/managed_beacon.py", '''
#!/usr/bin/env python3
"""A managed lifecycle node, for Exercise 1.3.

After `configure` the publisher EXISTS and the topic appears in `ros2 topic
list`, and nothing is published. A topic existing is not the same as data
flowing, and treating them as the same causes a great deal of confused
debugging from Lab 6 onwards.

    ros2 lifecycle set /managed_beacon configure
    ros2 lifecycle set /managed_beacon activate
"""

import rclpy
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from std_msgs.msg import Float32


class ManagedBeacon(LifecycleNode):
    def __init__(self):
        super().__init__("managed_beacon")
        self.publisher = None
        self.timer = None
        self.k = 0

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.publisher = self.create_lifecycle_publisher(Float32, "beacon", 10)
        self.timer = self.create_timer(0.1, self.tick)
        self.timer.cancel()
        self.get_logger().info("configured: publisher created, nothing published yet")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.timer.reset()
        self.get_logger().info("active: publishing")
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.timer.cancel()
        self.get_logger().info("inactive: still running, not publishing")
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.destroy_timer(self.timer)
        self.destroy_lifecycle_publisher(self.publisher)
        return TransitionCallbackReturn.SUCCESS

    def tick(self):
        msg = Float32()
        msg.data = float(self.k)
        self.publisher.publish(msg)
        self.k += 1


def main():
    rclpy.init()
    node = ManagedBeacon()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
''')

    # =====================================================================
    # Lab 2: the C++ build task
    # =====================================================================
    write(root, "arc_lab2_cpp/package.xml", package_xml(
        "arc_lab2_cpp", "Lab 2: a small rclcpp node, to be modified and rebuilt",
        "ament_cmake", ["rclcpp", "sensor_msgs"]))
    write(root, "arc_lab2_cpp/CMakeLists.txt", cmake_lists(
        "arc_lab2_cpp", [], {"wheel_watchdog": ["rclcpp", "sensor_msgs"]}))
    write(root, "arc_lab2_cpp/src/wheel_watchdog.cpp", '''
// Lab 2, Exercise 2.5. You will not write C++ in this course, but you will read
// it, and reading it is much easier once you have compiled something.
//
// Two changes to make:
//   1. max_wheel_speed default 20.0 -> 16.3, the value the kinematics demanded.
//   2. Change the subscribed topic, rebuild, then change it back.
//
// Notice the structure is the same as Code 1.1: a class deriving from Node, a
// subscription with a callback, a declared parameter. Only the syntax and the
// build step differ.

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class WheelWatchdog : public rclcpp::Node
{
public:
  WheelWatchdog()
  : Node("wheel_watchdog")
  {
    this->declare_parameter<double>("max_wheel_speed", 20.0);
    this->declare_parameter<std::string>("joint_states_topic", "/joint_states");

    const auto topic = this->get_parameter("joint_states_topic").as_string();

    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      topic, 10,
      std::bind(&WheelWatchdog::on_joint_states, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "watching %s", topic.c_str());
  }

private:
  void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    const double limit = this->get_parameter("max_wheel_speed").as_double();
    for (size_t i = 0; i < msg->name.size() && i < msg->velocity.size(); ++i) {
      if (std::abs(msg->velocity[i]) > limit) {
        RCLCPP_WARN(
          this->get_logger(), "%s at %.2f rad/s exceeds %.2f",
          msg->name[i].c_str(), msg->velocity[i], limit);
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelWatchdog>());
  rclcpp::shutdown();
  return 0;
}
''')

    # =====================================================================
    # Lab 3
    # =====================================================================
    python_package(root, "arc_sensors", "Lab 3: sensor diagnostics and recording",
                   ["rclpy", "sensor_msgs", "rosbag2_py"],
                   {"sensor_doctor": "arc_sensors.sensor_doctor:main"})

    write(root, "arc_sensors/arc_sensors/sensor_doctor.py", '''
#!/usr/bin/env python3
"""Diagnose a sensor topic beyond its average rate.

    ros2 run arc_sensors sensor_doctor --ros-args -p use_sim_time:=true
    ros2 run arc_sensors sensor_doctor --ros-args -p use_sim_time:=false

The second invocation is the teaching moment: a node on the wall clock while the
simulator publishes /clock reports a stamp lag of about 1.7 billion seconds.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

from arc_sensors.rate_stats import analyse, stamp_lag

# Sensor data is published best effort. A subscription using the default
# reliable profile connects successfully and receives nothing. Lab 1, Ex 1.2.
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)


class SensorDoctor(Node):
    def __init__(self):
        super().__init__("sensor_doctor")
        self.declare_parameter("topic", "/scan")
        self.declare_parameter("expected_hz", 10.0)
        self.declare_parameter("window", 100)

        self.expected_hz = self.get_parameter("expected_hz").value
        self.window = self.get_parameter("window").value
        self.header_stamps, self.receive_times, self.frame_id = [], [], None

        topic = self.get_parameter("topic").value
        self.create_subscription(LaserScan, topic, self.on_scan, SENSOR_QOS)
        self.get_logger().info(f"watching {topic}, expecting {self.expected_hz} Hz")

    def on_scan(self, msg):
        # Two clocks on purpose: when the sensor says the measurement was taken,
        # and when this node saw it. Comparing them catches use_sim_time errors.
        self.header_stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        self.receive_times.append(self.get_clock().now().nanoseconds / 1e9)
        self.frame_id = msg.header.frame_id
        if len(self.header_stamps) >= self.window:
            self.report()
            self.header_stamps.clear()
            self.receive_times.clear()

    def report(self):
        result = analyse(self.header_stamps, self.expected_hz)
        lag = stamp_lag(self.header_stamps, self.receive_times)
        self.get_logger().info(str(result))
        self.get_logger().info(f"frame_id '{self.frame_id}', mean stamp lag {lag:.4f} s")
        if abs(lag) > 1.0:
            self.get_logger().error(
                "stamp lag over one second. This node and the publisher are on "
                "different clocks. Check use_sim_time.")


def main():
    rclpy.init()
    node = SensorDoctor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
''')

    # =====================================================================
    # Lab 5
    # =====================================================================
    python_package(root, "arc_localization", "Lab 5: odometry drift measurement and SLAM bring-up",
                   ["rclpy", "nav_msgs", "geometry_msgs", "tf2_ros", "slam_toolbox"],
                   {"drift_meter": "arc_localization.drift_meter:main"},
                   data_dirs=["launch"])

    write(root, "arc_localization/arc_localization/drift_meter.py", '''
#!/usr/bin/env python3
"""Measure odometry drift against ground truth.

The robot does not have access to ground truth. You do, because this is
simulation, and comparing the two is the entire motivation for Lab 5.

    ros2 run arc_localization drift_meter
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class DriftMeter(Node):
    def __init__(self):
        super().__init__("drift_meter")
        self.declare_parameter("truth_topic", "/ground_truth")
        self.declare_parameter("report_every_m", 5.0)

        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(
            Odometry, self.get_parameter("truth_topic").value, self.on_truth, 10)

        self.odom = self.truth = None
        self.distance = 0.0
        self.last_odom = None
        self.next_report = self.get_parameter("report_every_m").value

    @staticmethod
    def pose_of(msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        return p.x, p.y, yaw

    def on_odom(self, msg):
        self.odom = self.pose_of(msg)
        if self.last_odom is not None:
            self.distance += math.hypot(self.odom[0] - self.last_odom[0],
                                        self.odom[1] - self.last_odom[1])
        self.last_odom = self.odom
        if self.distance >= self.next_report:
            self.report()
            self.next_report += self.get_parameter("report_every_m").value

    def on_truth(self, msg):
        self.truth = self.pose_of(msg)

    def report(self):
        if self.odom is None or self.truth is None:
            self.get_logger().warn("no ground truth; is it published?")
            return
        error = math.hypot(self.truth[0] - self.odom[0], self.truth[1] - self.odom[1])
        heading = math.degrees(math.atan2(
            math.sin(self.truth[2] - self.odom[2]),
            math.cos(self.truth[2] - self.odom[2])))
        # The percentage is the interesting column: roughly constant means the
        # error is proportional to distance, which is what accumulating random
        # error looks like. Growing means something systematic is present too.
        self.get_logger().info(
            f"driven {self.distance:5.1f} m | position error {error:5.3f} m "
            f"| heading error {heading:6.1f} deg | {100 * error / max(self.distance, 1e-6):4.1f}%")


def main():
    rclpy.init()
    node = DriftMeter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
''')

    # =====================================================================
    # Lab 4 and Lab 7
    # =====================================================================
    python_package(root, "arc_mapping", "Lab 4: reading recordings for occupancy mapping",
                   ["rclpy", "rosbag2_py", "sensor_msgs", "nav_msgs", "nav2_map_server"],
                   {"bag_reader": "arc_mapping.bag_reader:main"})

    write(root, "arc_mapping/arc_mapping/bag_reader.py", '''
#!/usr/bin/env python3
"""Read a bag into plain NumPy arrays. No ROS node, no live simulation.

Supplied because parsing a bag is not the learning objective. Lab 4 Code 4.1.
"""

import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


def read_run(path, scan_topic="/scan", odom_topic="/odom"):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    scans, odoms = [], []
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic == scan_topic:
            scans.append((stamp_ns, deserialize_message(data, LaserScan)))
        elif topic == odom_topic:
            odoms.append((stamp_ns, deserialize_message(data, Odometry)))
    return scans, odoms


def pose_from_odom(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    theta = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
    return float(p.x), float(p.y), float(theta)


def nearest_pose(scan_stamp_ns, odoms):
    """Odometry runs at 50 Hz and the LiDAR at 10 Hz, so no odometry sample
    exists at exactly the right instant. Nearest neighbour costs up to 10 ms,
    which at 0.5 m/s is 5 mm. Interpolating would be better and is a legitimate
    extension."""
    stamps = np.array([s for s, _ in odoms])
    return pose_from_odom(odoms[int(np.argmin(np.abs(stamps - scan_stamp_ns)))][1])


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "bags/lab03_mapping_run"
    scans, odoms = read_run(path)
    print(f"{len(scans)} scans, {len(odoms)} odometry samples")
    if scans:
        s = scans[0][1]
        print(f"{len(s.ranges)} beams, angle_min {s.angle_min:.3f}, "
              f"increment {s.angle_increment:.5f}, frame '{s.header.frame_id}'")


if __name__ == "__main__":
    main()
''')

    python_package(root, "arc_recovery", "Lab 7: deliberately difficult navigation scenarios",
                   ["rclpy", "navigation2", "nav2_bringup", "arc_nav", "arc_gazebo"],
                   {"fault_injector": "arc_recovery.fault_injector:main"},
                   data_dirs=["launch", "config"])

    write(root, "arc_recovery/arc_recovery/fault_injector.py", '''
#!/usr/bin/env python3
"""Apply one of the Lab 7 scenario faults to a running Nav2 stack.

Used by demonstrators, not by students. Each fault is a parameter change, so it
is reversible and leaves the rest of the stack untouched.

    ros2 run arc_recovery fault_injector --ros-args -p scenario:=1
"""

import subprocess
import sys

import rclpy
from rclpy.node import Node

# Each entry is (node, parameter, value, expected symptom).
SCENARIOS = {
    1: ("/controller_server", "FollowPath.max_vel_x", "0.02",
        "robot plans a path and creeps, progress checker eventually fails"),
    2: ("/global_costmap/global_costmap", "inflation_layer.inflation_radius", "0.95",
        "no valid path through the doorway"),
    3: ("/collision_monitor", "PolygonStop.radius", "1.5",
        "robot refuses to move at all, cmd_vel is zeroed"),
    4: ("/controller_server", "progress_checker.movement_time_allowance", "1.0",
        "recoveries are abandoned partway through"),
    5: ("/planner_server", "GridBased.tolerance", "0.01",
        "goals near obstacles are rejected"),
}


class FaultInjector(Node):
    def __init__(self):
        super().__init__("fault_injector")
        self.declare_parameter("scenario", 1)
        self.declare_parameter("revert", False)


def apply(scenario: int) -> int:
    if scenario not in SCENARIOS:
        print(f"unknown scenario {scenario}; choose from {sorted(SCENARIOS)}")
        return 2
    node, parameter, value, symptom = SCENARIOS[scenario]
    print(f"scenario {scenario}: setting {node} {parameter} = {value}")
    print(f"expected symptom: {symptom}")
    result = subprocess.run(["ros2", "param", "set", node, parameter, value],
                            capture_output=True, text=True)
    print(result.stdout.strip() or result.stderr.strip())
    return result.returncode


def main():
    rclpy.init()
    node = FaultInjector()
    scenario = node.get_parameter("scenario").value
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(apply(scenario))


if __name__ == "__main__":
    main()
''')

    write(root, "arc_recovery/launch/difficult_robot.launch.py", '''
"""Bring up the simulation and Nav2, then apply one scenario fault.

    ros2 launch arc_recovery difficult_robot.launch.py scenario:=1
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scenario = LaunchConfiguration("scenario")
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="1"),
        DeclareLaunchArgument("map"),
        DeclareLaunchArgument("headless", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare("arc_gazebo"), "launch", "simulation.launch.py"])),
            launch_arguments={"headless": LaunchConfiguration("headless")}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare("arc_nav"), "launch", "navigation.launch.py"])),
            launch_arguments={"map": LaunchConfiguration("map")}.items()),
        # The fault is applied once the lifecycle nodes have had time to reach
        # active; setting a parameter on an inactive node has no effect.
        TimerAction(period=25.0, actions=[
            ExecuteProcess(cmd=["ros2", "run", "arc_recovery", "fault_injector",
                                "--ros-args", "-p", ["scenario:=", scenario]],
                           output="screen")]),
    ])
''')

    # =====================================================================
    # arc_rl and arc_eval: pure Python, but installed so they are importable
    # from anywhere in the workspace rather than only from the repo root.
    # =====================================================================
    python_package(root, "arc_rl", "Reinforcement learning environments and arenas",
                   ["rclpy", "sensor_msgs", "geometry_msgs", "nav_msgs"], {})
    python_package(root, "arc_eval", "Seeded evaluation harness and competition scoring",
                   ["rclpy", "nav2_msgs", "geometry_msgs", "nav_msgs"],
                   {"arc_eval": "arc_eval.runner:main"},
                   data_dirs=["configs"])

    # =====================================================================
    # Repository files
    # =====================================================================
    write(root, ".gitignore", '''
# build artifacts
build/
install/
log/
__pycache__/
*.py[cod]
.pytest_cache/

# generated, reproducible from source
docs/figures/*.png
results/*.json

# large and machine specific
bags/
models/*.zip
maps/*.pgm
*.mcap
*.ova
*.vdi

# editors
.vscode/
.idea/
''')

    write(root, ".github/workflows/ci.yml", '''
# Runs the part of the suite that needs no ROS, on every push. Catches the class
# of error that unit tests catch: a broken sensor model, a scoring formula that
# rewards the wrong behaviour, an arena the robot cannot fit through.
name: tests

on: [push, pull_request]

jobs:
  offline-tests:
    runs-on: ubuntu-24.04
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-python@v5
        with:
          python-version: "3.12"
      - name: Install dependencies
        run: pip install "numpy<2" gymnasium pyyaml pytest pillow matplotlib
      - name: Run the offline suite
        run: python -m pytest starters arc_eval -q
''')
