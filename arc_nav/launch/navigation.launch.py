"""
Bring up the ARC navigation stack.

  ros2 launch arc_nav navigation.launch.py map:=/path/to/map.yaml params:=dwb
  ros2 launch arc_nav navigation.launch.py map:=/path/to/map.yaml params:=mppi

The params argument selects between the two controller baselines compared in
Lab 5. Everything else about the two configurations is identical, which is what
makes the comparison a single-variable one.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_yaml = LaunchConfiguration("map")
    params = LaunchConfiguration("params")
    use_rviz = LaunchConfiguration("rviz")
    autostart = LaunchConfiguration("autostart")

    params_file = PathJoinSubstitution(
        [FindPackageShare("arc_nav"), "config", ["nav2_", params, ".yaml"]]
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"]
            )
        ),
        launch_arguments={
            "map": map_yaml,
            "params_file": params_file,
            "use_sim_time": "true",
            "autostart": autostart,
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(use_rviz),
        arguments=["-d", PathJoinSubstitution(
            [FindPackageShare("arc_nav"), "rviz", "arc_navigation.rviz"])],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("map", description="Path to the saved map YAML"),
        DeclareLaunchArgument("params", default_value="dwb",
                              description="Controller baseline: dwb or mppi"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("autostart", default_value="true"),
        nav2,
        rviz,
    ])
