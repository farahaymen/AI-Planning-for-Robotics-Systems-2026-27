import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'arc_lab1'

setup(
    name=package_name,
    version='2026.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Farah Aymen',
    maintainer_email='farah.aymen@bue.edu.eg',
    description='Lab 1: ROS 2 nodes, QoS and managed lifecycle',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'range_source = arc_lab1.range_source:main',
            'range_monitor = arc_lab1.range_monitor:main',
            'managed_beacon = arc_lab1.managed_beacon:main',
        ],
    },
)
