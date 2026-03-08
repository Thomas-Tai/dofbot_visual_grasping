# Copyright (c) 2024 DOFBOT Project
# SPDX-License-Identifier: BSD-3-Clause

"""Setup script for the dofbot_hardware package."""

from glob import glob
import os
from setuptools import setup, find_packages

package_name = 'dofbot_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DOFBOT Project',
    maintainer_email='dofbot@example.com',
    description='Hardware interface package for DOFBOT robot arm',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_monitor = dofbot_hardware.safety_monitor:main',
        ],
    },
)