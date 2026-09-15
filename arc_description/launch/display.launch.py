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
