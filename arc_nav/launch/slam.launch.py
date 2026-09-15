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
