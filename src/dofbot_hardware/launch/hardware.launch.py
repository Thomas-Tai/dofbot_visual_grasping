# Copyright (c) 2024 DOFBOT Project
# SPDX-License-Identifier: BSD-3-Clause

"""
Launch file for the DOFBOT hardware interface.

This launch file starts:
    1. Hardware node (with mock option for simulation)
    2. Safety monitor (optional)
    3. Joint state publisher (if using ROS2 control)

Arguments:
    use_mock: Use mock driver for simulation (default: False)
    safety_enabled: Enable safety monitoring (default: True)
    publish_rate: Joint state publish rate in Hz (default: 50.0)
    velocity_scaling: Velocity scaling factor (default: 0.5)

Usage:
    # Real hardware
    ros2 launch dofbot_hardware hardware.launch.py

    # Simulation mode
    ros2 launch dofbot_hardware hardware.launch.py use_mock:=true

    # With safety disabled
    ros2 launch dofbot_hardware hardware.launch.py safety_enabled:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    """Generate the launch description for hardware interface."""
    
    # Declare launch arguments
    use_mock_arg = DeclareLaunchArgument(
        'use_mock',
        default_value='false',
        description='Use mock driver for simulation'
    )
    
    safety_enabled_arg = DeclareLaunchArgument(
        'safety_enabled',
        default_value='true',
        description='Enable safety monitoring'
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
    
    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value='1.0',
        description='Maximum joint velocity in rad/s'
    )
    
    joint_names_arg = DeclareLaunchArgument(
        'joint_names',
        default_value="['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper']",
        description='List of joint names'
    )
    
    # Hardware node
    hardware_node = LifecycleNode(
        package='dofbot_hardware',
        executable='hardware_node',
        name='dofbot_hardware',
        namespace='',
        output='screen',
        parameters=[{
            'use_mock': LaunchConfiguration('use_mock'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'velocity_scaling': LaunchConfiguration('velocity_scaling'),
            'max_velocity': LaunchConfiguration('max_velocity'),
            'joint_names': PythonExpression([
                LaunchConfiguration('joint_names')
            ]),
        }],
        # Automatically configure and activate
        # This is a workaround for lifecycle node auto-activation
    )
    
    # Safety monitor node (conditional)
    safety_node = LifecycleNode(
        package='dofbot_hardware',
        executable='safety_monitor',
        name='safety_monitor',
        namespace='',
        output='screen',
        parameters=[{
            'max_position_error': 0.1,
            'heartbeat_timeout': 1.0,
            'safety_check_rate': 100.0,
        }],
        condition=IfCondition(LaunchConfiguration('safety_enabled'))
    )
    
    return LaunchDescription([
        # Launch arguments
        use_mock_arg,
        safety_enabled_arg,
        publish_rate_arg,
        velocity_scaling_arg,
        max_velocity_arg,
        joint_names_arg,
        
        # Nodes
        hardware_node,
        safety_node,
    ])