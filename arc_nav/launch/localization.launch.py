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
