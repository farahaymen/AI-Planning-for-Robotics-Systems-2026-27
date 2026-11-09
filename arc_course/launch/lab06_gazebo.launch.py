"""Start a fresh copy of the Lab 6 grid and run one saved policy."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('q_table', description='Path to the saved Lab 6 q.npy'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run Gazebo without its GUI'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('arc_gazebo'), 'launch', 'simulation.launch.py',
            ])),
            launch_arguments={'world': 'lab06_grid', 'headless': LaunchConfiguration('headless')}.items(),
        ),
        Node(package='arc_course', executable='grid_policy_driver', output='screen',
             parameters=[{'q_table': ParameterValue(LaunchConfiguration('q_table'), value_type=str)}]),
    ])
