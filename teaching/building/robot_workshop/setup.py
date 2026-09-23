from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_workshop'
setup(
    name=package_name, version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='ARC course team', maintainer_email='farah.aymen@bue.edu.eg',
    description='Reference for the progressive student build activities',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'drive_distance = robot_workshop.drive_distance:main',
        'distance_source = robot_workshop.distance_source:main',
        'distance_alert = robot_workshop.distance_alert:main',
        'front_range = robot_workshop.front_range:main',
        'small_map = robot_workshop.small_map:main',
        'scan_service = robot_workshop.scan_service:main',
        'send_goal = robot_workshop.send_goal:main',
        'progress = robot_workshop.progress:main',
        'approach_env = robot_workshop.approach_env:main',
        'train_approach = robot_workshop.train_approach:main',
        'command_gate = robot_workshop.command_gate:main',
    ]},
)
