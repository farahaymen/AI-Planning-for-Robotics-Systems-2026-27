"""Run SLAM Toolbox against a recorded bag rather than a live simulation.

    ros2 launch arc_localization slam_from_bag.launch.py bag:=~/arc_ws/bags/lab03_mapping_run

Replaying a bag makes the exercise deterministic: the same recording produces the
same map every time, which is what makes it fair to grade and possible to debug.
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bag = LaunchConfiguration("bag")
    return LaunchDescription([
        DeclareLaunchArgument("bag", description="Path to the recorded run"),
        DeclareLaunchArgument("rate", default_value="1.0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare("arc_nav"), "launch", "slam.launch.py"]))),
        # --clock makes the replay drive simulation time for everything
        # consuming it. Without it, SLAM Toolbox and the bag disagree about now.
        TimerAction(period=4.0, actions=[
            ExecuteProcess(
                cmd=["ros2", "bag", "play", bag, "--clock",
                     "--rate", LaunchConfiguration("rate")],
                output="screen")]),
    ])
