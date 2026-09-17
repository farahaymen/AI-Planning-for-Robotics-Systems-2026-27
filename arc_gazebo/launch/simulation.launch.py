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
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

FAULTS = {"broken_a": "a", "broken_b": "b", "broken_c": "c"}


def launch_setup(context, *args, **kwargs):
    world_arg = LaunchConfiguration("world").perform(context)
    headless = LaunchConfiguration("headless")

    fault = FAULTS.get(world_arg, "none")
    world_file = "arc_warehouse.sdf" if world_arg in FAULTS else f"{world_arg}.sdf"

    desc_share = FindPackageShare("arc_description")
    gz_share = FindPackageShare("arc_gazebo")

    # value_type=str is required. Without it launch tries to parse the URDF as
    # YAML, which fails because a URDF is full of colons and braces that mean
    # something else in YAML.
    robot_description = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([desc_share, "urdf", "arc_bot.urdf.xacro"]),
            " fault:=", fault,
        ]),
        value_type=str,
    )

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
    # Remapping the SPAWNER does nothing: it is a separate short-lived process
    # that only calls services. The controller itself lives inside the
    # controller manager, so the remap must be forwarded to it directly.
    diff_drive = Node(
        package="controller_manager", executable="spawner", output="screen",
        arguments=["diff_drive_controller", "--controller-ros-args",
                   "-r /diff_drive_controller/cmd_vel:=/cmd_vel "
                   "-r /diff_drive_controller/odom:=/odom"])

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
