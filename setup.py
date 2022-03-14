#!/usr/bin/env python
# setup.py
"""Install script for ROS1 catkin / ROS2 ament_python."""

from setuptools import setup

package_name = 'ng_trajectory_ros'

setup(
 name=package_name,
 version='0.0.0',
 packages=[package_name, package_name+".module"],
 data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
   ],
 install_requires=['setuptools'],
 zip_safe=True,
 author='Jaroslav Klapálek',
 author_email='klapajar@fel.cvut.cz',
 maintainer='Jaroslav Klapálek',
 maintainer_email='klapajar@fel.cvut.cz',
 description='ROS wrapper for ng_trajectory.',
 license='GPLv3',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'run = ng_trajectory_ros.run:main'
     ],
   },
)
