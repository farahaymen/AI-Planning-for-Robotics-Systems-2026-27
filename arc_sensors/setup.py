import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'arc_sensors'

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
    description='Lab 2: sensor diagnostics and recording',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sensor_doctor = arc_sensors.sensor_doctor:main',
        ],
    },
)
