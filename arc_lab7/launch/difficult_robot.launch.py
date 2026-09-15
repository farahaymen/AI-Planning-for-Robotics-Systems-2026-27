"""Bring up the simulation and Nav2, then apply one scenario fault.

    ros2 launch arc_lab7 difficult_robot.launch.py scenario:=1
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
            ExecuteProcess(cmd=["ros2", "run", "arc_lab7", "fault_injector",
                                "--ros-args", "-p", ["scenario:=", scenario]],
                           output="screen")]),
    ])
