# Copyright (c) 2024 DOFBOT Project
# SPDX-License-Identifier: BSD-3-Clause

"""
Hardware driver interface for DOFBOT robot arm.

This module defines the protocol interface for hardware drivers,
enabling dependency injection and mock testing.
"""

from typing import List, Optional, Protocol, runtime_checkable
from dataclasses import dataclass


@dataclass
class JointLimits:
    """Safety limits for a single joint.
    
    Attributes:
        position_min: Minimum position in radians.
        position_max: Maximum position in radians.
        velocity_max: Maximum velocity in radians/second.
        effort_max: Maximum effort in Nm or current.
    """
    position_min: float = -1.5708  # -π/2 radians
    position_max: float = 1.5708   # π/2 radians
    velocity_max: float = 1.0      # rad/s
    effort_max: float = 1.0        # Nm or relative units


@runtime_checkable
class DofbotDriverInterface(Protocol):
    """
    Protocol defining the interface for DOFBOT hardware drivers.
    
    This interface enables dependency injection, allowing the same
    control code to work with real hardware or mock drivers.
    
    All angle values are in radians with the following convention:
    - Hardware servos use 0-180 degrees
    - Joint center (90°) maps to 0 radians
    - 0° maps to -π/2, 180° maps to +π/2
    """
    
    def connect(self) -> bool:
        """
        Establish connection to the hardware.
        
        Returns:
            True if connection successful, False otherwise.
            
        Raises:
            DofbotConnectionError: If connection fails.
        """
        ...
    
    def disconnect(self) -> None:
        """
        Disconnect from the hardware.
        
        Should be safe to call multiple times.
        """
        ...
    
    def is_connected(self) -> bool:
        """
        Check if currently connected to hardware.
        
        Returns:
            True if connected, False otherwise.
        """
        ...
    
    def read_joint_positions(self) -> List[float]:
        """
        Read all joint positions from hardware.
        
        Returns:
            List of 6 joint positions in radians [j1, j2, j3, j4, j5, gripper].
            
        Raises:
            DofbotCommunicationError: If read fails.
            DofbotConnectionError: If not connected.
        """
        ...
    
    def write_joint_positions(
        self, 
        positions: List[float], 
        time_ms: int
    ) -> bool:
        """
        Write all joint positions to hardware.
        
        Args:
            positions: List of 6 joint positions in radians.
            time_ms: Execution time in milliseconds.
            
        Returns:
            True if write successful, False otherwise.
            
        Raises:
            DofbotValueError: If positions length != 6.
            DofbotCommunicationError: If write fails.
            DofbotConnectionError: If not connected.
        """
        ...
    
    def read_single_joint(self, joint_id: int) -> Optional[float]:
        """
        Read a single joint position.
        
        Args:
            joint_id: Joint ID (1-6 for DOFBOT).
            
        Returns:
            Joint position in radians, or None if read fails.
            
        Raises:
            DofbotJointError: If joint_id is invalid.
        """
        ...
    
    def write_single_joint(
        self, 
        joint_id: int, 
        angle: float, 
        time_ms: int
    ) -> bool:
        """
        Write a single joint position.
        
        Args:
            joint_id: Joint ID (1-6 for DOFBOT).
            angle: Joint angle in radians.
            time_ms: Execution time in milliseconds.
            
        Returns:
            True if write successful, False otherwise.
            
        Raises:
            DofbotJointError: If joint_id is invalid.
        """
        ...
    
    def stop_motion(self) -> bool:
        """
        Immediately stop all motion.
        
        Commands current positions with 0ms time to halt movement.
        
        Returns:
            True if successful, False otherwise.
        """
        ...


# Default joint limits for DOFBOT (5 DOF + gripper)
DEFAULT_JOINT_LIMITS = [
    JointLimits(position_min=-1.5708, position_max=1.5708, velocity_max=1.5, effort_max=1.0),  # Joint 1 (base)
    JointLimits(position_min=-1.5708, position_max=1.5708, velocity_max=1.5, effort_max=1.0),  # Joint 2 (shoulder)
    JointLimits(position_min=-1.5708, position_max=1.5708, velocity_max=2.0, effort_max=0.8),  # Joint 3 (elbow)
    JointLimits(position_min=-1.5708, position_max=1.5708, velocity_max=2.0, effort_max=0.5),  # Joint 4 (wrist pitch)
    JointLimits(position_min=-1.5708, position_max=1.5708, velocity_max=2.5, effort_max=0.3),  # Joint 5 (wrist roll)
    JointLimits(position_min=0.0, position_max=0.8, velocity_max=3.0, effort_max=0.5),          # Gripper
]

# Joint names matching URDF
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper']