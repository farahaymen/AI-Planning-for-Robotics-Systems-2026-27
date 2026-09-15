import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'arc_lab7'

setup(
    name=package_name,
    version='2026.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Farah Aymen',
    maintainer_email='farah.aymen@bue.edu.eg',
    description='Lab 7: deliberately difficult navigation scenarios',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fault_injector = arc_lab7.fault_injector:main',
        ],
    },
)
