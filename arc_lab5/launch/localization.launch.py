"""Simulation plus map server plus AMCL, for Exercise 5.4.

    ros2 launch arc_lab5 localization.launch.py map:=~/arc_ws/maps/slam_map.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map", description="Path to the saved map YAML"),
        DeclareLaunchArgument("headless", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare("arc_gazebo"), "launch", "simulation.launch.py"])),
            launch_arguments={"headless": LaunchConfiguration("headless")}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare("arc_nav"), "launch", "localization.launch.py"])),
            launch_arguments={"map": LaunchConfiguration("map")}.items()),
    ])
