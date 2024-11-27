from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'fruit_picking'
config = "parameters.json"

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[                                            # Archivos de datos
        ('share/ament_index/resource_index/packages',
            ['resource/' + config]),
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/arise_system_module',
            ['module/' + package_name]),
        ('share/' + package_name + '/launch', ['launch/fruit_picking.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raucal',
    maintainer_email='raucal@cartif.es',
    description='Fruit picking with collaborative robot',
    license='ARISE - CARTIF',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_fruits = fruit_picking.detect_fruits:main',
            'intel_transforms = fruit_picking.intel_transforms:main',
            'robot_pick = fruit_picking.robot_pick:main',
            'arise_ai = fruit_picking.arise_ai:main',
            'fruits_manager = fruit_picking.fruits_manager:main',
            'linear_guide_controller = fruit_picking.linear_guide_controller:main'
        ],
    },
)
