import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'arc_lab4'

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
    description='Lab 4: reading recordings for occupancy mapping',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bag_reader = arc_lab4.bag_reader:main',
        ],
    },
)
