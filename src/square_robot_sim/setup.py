import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'square_robot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michael',
    maintainer_email='33632547+why-does-ie-still-exist@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'square_robot_node = square_robot_sim.square_robot_node:main',
            'simple_republisher = square_robot_sim.simple_republisher:main',
            'error_publisher = square_robot_sim.error_publisher:main',
        ],
    },
)
