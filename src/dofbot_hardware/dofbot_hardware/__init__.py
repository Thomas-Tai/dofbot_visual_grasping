# Copyright (c) 2024 DOFBOT Project
# SPDX-License-Identifier: BSD-3-Clause

"""
DOFBOT Hardware Interface Package.

This package provides hardware abstraction and ROS2 integration for the
DOFBOT 5-DOF robot arm with gripper.

Modules:
    exceptions: Custom exception hierarchy for hardware errors.
    arm_driver: Production hardware driver using Arm_Lib.
    mock_driver: Simulated driver for testing without hardware.
    hardware_node: ROS2 lifecycle node for hardware interface.
    joint_state_publisher: Joint state publisher with velocity estimation.
    trajectory_executor: Trajectory execution with safety features.
    safety_monitor: Safety monitoring and emergency stop.
"""

from .exceptions import (
    DofbotError,
    DofbotConnectionError,
    DofbotCommunicationError,
    DofbotTimeoutError,
    DofbotJointError,
    DofbotValueError,
)

# Import drivers conditionally to allow installation without Arm_Lib
try:
    from .arm_driver import DofbotDriver
except ImportError:
    # Arm_Lib not installed, DofbotDriver unavailable
    DofbotDriver = None  # type: ignore

from .mock_driver import MockDofbotDriver, MockConfig, CommandRecord, FailureType
from .joint_state_publisher import (
    VelocityEstimator,
    VelocityEstimatorConfig,
)
from .trajectory_executor import (
    TrajectoryExecutor,
    TrajectoryValidator,
    TrajectoryConfig,
    JointLimits,
    ExecutionState,
    get_default_joint_limits,
)

__all__ = [
    # Exceptions
    'DofbotError',
    'DofbotConnectionError',
    'DofbotCommunicationError',
    'DofbotTimeoutError',
    'DofbotJointError',
    'DofbotValueError',
    # Real driver
    'DofbotDriver',
    # Mock driver
    'MockDofbotDriver',
    'MockConfig',
    'CommandRecord',
    'FailureType',
    # Joint state publisher
    'VelocityEstimator',
    'VelocityEstimatorConfig',
    # Trajectory executor
    'TrajectoryExecutor',
    'TrajectoryValidator',
    'TrajectoryConfig',
    'JointLimits',
    'ExecutionState',
    'get_default_joint_limits',
]

__version__ = '0.1.0'