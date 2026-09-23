"""Bring up the simulation and Nav2, then apply one scenario fault.

    ros2 launch arc_recovery difficult_robot.launch.py scenario:=1
    ros2 launch arc_recovery difficult_robot.launch.py scenario:=1 map:=/path/to/other.yaml

The map defaults to the one you saved in Lab 5. Pass map:= to use a different
one.
"""

import os

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
        # Without a default this aborts with a missing-argument error, which is
        # a poor first experience of a lab about diagnosing failures.
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(
                os.path.expanduser("~"), "arc_ws", "maps", "slam_map.yaml")),
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
