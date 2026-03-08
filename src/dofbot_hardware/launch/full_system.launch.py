# Copyright (c) 2024 DOFBOT Project
# SPDX-License-Identifier: BSD-3-Clause

"""
Launch file for the complete DOFBOT visual grasping system.

This launch file starts the full system including:
    1. MoveIt2 (dofbot_moveit_config demo.launch.py)
    2. Hardware interface
    3. Vision pipeline (dofbot_vision)
    4. Safety monitor

Arguments:
    use_mock: Use mock hardware for simulation (default: True)
    use_vision: Enable vision pipeline (default: True)
    safety_enabled: Enable safety monitoring (default: True)
    camera_device: Camera device path (default: '/dev/video0')
    publish_rate: Joint state publish rate in Hz (default: 50.0)

Usage:
    # Full simulation mode (default)
    ros2 launch dofbot_hardware full_system.launch.py

    # Real hardware with real camera
    ros2 launch dofbot_hardware full_system.launch.py use_mock:=false

    # Real hardware, simulated vision
    ros2 launch dofbot_hardware full_system.launch.py use_mock:=false use_vision:=false

    # Custom camera device
    ros2 launch dofbot_hardware full_system.launch.py camera_device:=/dev/video1
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    """Generate the launch description for the full system."""
    
    # Get package directories
    dofbot_hardware_dir = get_package_share_directory('dofbot_hardware')
    dofbot_vision_dir = get_package_share_directory('dofbot_vision')
    dofbot_moveit_dir = get_package_share_directory('dofbot_moveit_config')
    
    # Declare launch arguments
    use_mock_arg = DeclareLaunchArgument(
        'use_mock',
        default_value='true',
        description='Use mock hardware for simulation'
    )
    
    use_vision_arg = DeclareLaunchArgument(
        'use_vision',
        default_value='true',
        description='Enable vision pipeline'
    )
    
    safety_enabled_arg = DeclareLaunchArgument(
        'safety_enabled',
        default_value='true',
        description='Enable safety monitoring'
    )
    
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='Camera device path'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='50.0',
        description='Joint state publish rate in Hz'
    )
    
    velocity_scaling_arg = DeclareLaunchArgument(
        'velocity_scaling',
        default_value='0.5',
        description='Velocity scaling factor for safety'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    # Hardware interface launch
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dofbot_hardware_dir, 'launch', 'hardware.launch.py')
        ),
        launch_arguments={
            'use_mock': LaunchConfiguration('use_mock'),
            'safety_enabled': LaunchConfiguration('safety_enabled'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'velocity_scaling': LaunchConfiguration('velocity_scaling'),
        }.items()
    )
    
    # Vision pipeline launch (conditional)
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dofbot_vision_dir, 'launch', 'vision_pipeline.launch.py')
        ),
        launch_arguments={
            'camera_device': LaunchConfiguration('camera_device'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_vision'))
    )
    
    # MoveIt2 launch (simulation mode - uses MoveIt2's joint state publisher)
    # When use_mock is true, we don't need to publish joint states from hardware
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dofbot_moveit_dir, 'launch', 'demo.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_mock'))
    )
    
    # MoveIt2 launch (hardware mode - uses our hardware node)
    # For real hardware, we use our hardware node for joint states
    moveit_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dofbot_moveit_dir, 'launch', 'demo.launch.py')
        ),
        launch_arguments={
            'use_sim': 'false',
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('use_mock'))
    )
    
    # Log startup info
    startup_log = LogInfo(msg=[
        '\n========================================\n',
        'DOFBOT Visual Grasping System Starting\n',
        '----------------------------------------\n',
        'Mock Hardware: ', LaunchConfiguration('use_mock'), '\n',
        'Vision Enabled: ', LaunchConfiguration('use_vision'), '\n',
        'Safety Enabled: ', LaunchConfiguration('safety_enabled'), '\n',
        'Camera Device: ', LaunchConfiguration('camera_device'), '\n',
        'Publish Rate: ', LaunchConfiguration('publish_rate'), ' Hz\n',
        '========================================\n'
    ])
    
    return LaunchDescription([
        # Launch arguments
        use_mock_arg,
        use_vision_arg,
        safety_enabled_arg,
        camera_device_arg,
        publish_rate_arg,
        velocity_scaling_arg,
        use_rviz_arg,
        
        # Startup log
        startup_log,
        
        # System components
        hardware_launch,
        vision_launch,
        moveit_launch,
        moveit_hardware_launch,
    ])