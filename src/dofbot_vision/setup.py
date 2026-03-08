from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dofbot_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Package marker for ament index (required for ros2 pkg list)
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         ['resource/' + package_name]),
        # Package manifest
        (os.path.join('share', package_name), ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy]*'))),
        # Config files
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Thomas Tai',
    maintainer_email='thomastai.uni@gmail.com',
    description='Vision perception for DOFBOT visual grasping',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = dofbot_vision.object_detector_node:main',
            'transform_node = dofbot_vision.coordinate_transform_node:main',
            'calibrate-hsv = dofbot_vision.calibrate_hsv:main',
            'calibrate-handeye = dofbot_vision.calibration_tool:main',
        ],
    },
)