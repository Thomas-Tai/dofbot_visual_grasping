#!/usr/bin/env python3
# Copyright (c) 2024 DOFBOT Project
# SPDX-License-Identifier: BSD-3-Clause

"""
Unified Motion Control Interface for DOFBOT.

This module provides a unified interface that can seamlessly switch between
simulation and real hardware control modes.

Usage:
    from dofbot_control.unified_interface import UnifiedMotionInterface, HardwareMode
    
    # Create interface for simulation
    interface = UnifiedMotionInterface(mode=HardwareMode.SIMULATION)
    
    # Or for real hardware
    interface = UnifiedMotionInterface(mode=HardwareMode.HARDWARE)
    
    # Use the same API regardless of mode
    interface.move_to_joint_state([0.0, 0.5, 0.0, 0.0, 0.0, 0.4])
    interface.set_gripper(closed=True)
"""

import time
import logging
from enum import Enum
from dataclasses import dataclass
from typing import List, Optional, Callable, Any

# Import hardware components
try:
    from dofbot_hardware.arm_driver import DofbotDriver
    from dofbot_hardware.mock_driver import MockDofbotDriver
    from dofbot_hardware.driver_interface import DofbotDriverInterface, JOINT_NAMES
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False
    JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper']


logger = logging.getLogger(__name__)


class HardwareMode(Enum):
    """Hardware operation modes.
    
    Attributes:
        SIMULATION: All simulated (Rviz + mock driver).
        HARDWARE: All real (real robot + real camera).
        HYBRID_MOTION: Real robot, simulated vision.
        HYBRID_VISION: Simulated robot, real camera.
    """
    SIMULATION = 'simulation'
    HARDWARE = 'hardware'
    HYBRID_MOTION = 'hybrid_motion'
    HYBRID_VISION = 'hybrid_vision'


@dataclass
class ControlConfig:
    """Configuration for motion control.
    
    Attributes:
        mode: Hardware operation mode.
        hardware_ns: Namespace for hardware topics.
        simulation_ns: Namespace for simulation topics.
        velocity_scaling: Scaling factor for velocities (0.0-1.0).
        publish_rate: Rate for joint state publishing (Hz).
    """
    mode: HardwareMode = HardwareMode.SIMULATION
    hardware_ns: str = '/hardware'
    simulation_ns: str = '/simulation'
    velocity_scaling: float = 0.5
    publish_rate: int = 50
    
    @classmethod
    def from_dict(cls, config_dict: dict) -> 'ControlConfig':
        """Create config from dictionary."""
        mode_str = config_dict.get('mode', 'simulation')
        mode = HardwareMode(mode_str) if isinstance(mode_str, str) else mode_str
        return cls(
            mode=mode,
            hardware_ns=config_dict.get('hardware_ns', '/hardware'),
            simulation_ns=config_dict.get('simulation_ns', '/simulation'),
            velocity_scaling=config_dict.get('velocity_scaling', 0.5),
            publish_rate=config_dict.get('publish_rate', 50),
        )


class UnifiedMotionInterface:
    """
    Motion control interface that works in both simulation and hardware.
    
    This interface provides a unified API for robot arm control that
    automatically adapts to the configured hardware mode.
    
    Features:
    - Seamless switching between simulation and real hardware
    - Consistent API for all modes
    - Safety checks and limits enforcement
    - Gripper control integration
    
    Example:
        interface = UnifiedMotionInterface(mode=HardwareMode.SIMULATION)
        
        # Connect to hardware/simulation
        if interface.connect():
            # Move to home position
            interface.move_to_named_pose('home')
            
            # Move to specific joint state
            interface.move_to_joint_state([0.0, 0.5, 0.0, 0.0, 0.0, 0.4])
            
            # Control gripper
            interface.set_gripper(closed=True)
            
            # Disconnect when done
            interface.disconnect()
    """
    
    # Named poses matching SRDF
    NAMED_POSES = {
        'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.4],
        'ready': [0.0, 0.72, 0.74, 0.0, 0.0, 0.4],
        'down': [0.0, 1.57, 0.0, 0.0, 0.0, 0.4],
        'up': [0.0, 0.0, 0.0, 0.0, 0.0, 0.4],
        'look_at': [0.0, 0.785, 0.785, 0.0, 0.0, 0.4],
    }
    
    def __init__(
        self, 
        mode: HardwareMode = HardwareMode.SIMULATION,
        config: Optional[ControlConfig] = None,
        moveit_interface: Optional[Any] = None
    ):
        """
        Initialize the unified motion interface.
        
        Args:
            mode: Hardware operation mode.
            config: Optional configuration object.
            moveit_interface: Optional MoveIt interface for motion planning.
        """
        self.config = config or ControlConfig(mode=mode)
        self.mode = self.config.mode
        self._moveit_interface = moveit_interface
        
        # Create driver based on mode
        self._driver: Optional[DofbotDriverInterface] = None
        self._create_driver()
        
        # State tracking
        self._connected = False
        self._current_positions = [0.0] * 6
        
        logger.info("UnifiedMotionInterface initialized with mode: %s", self.mode.value)
    
    def _create_driver(self) -> None:
        """Create the appropriate driver based on mode."""
        if not HARDWARE_AVAILABLE:
            logger.warning("Hardware package not available, using mock driver")
            self._driver = MockDofbotDriver()
            return
        
        if self.mode == HardwareMode.SIMULATION or self.mode == HardwareMode.HYBRID_VISION:
            self._driver = MockDofbotDriver()
            logger.info("Created mock driver for simulation mode")
        else:
            self._driver = DofbotDriver()
            logger.info("Created real hardware driver")
    
    def connect(self) -> bool:
        """
        Connect to the hardware or simulation.
        
        Returns:
            True if connection successful.
        """
        if self._driver is None:
            logger.error("No driver available")
            return False
        
        try:
            self._connected = self._driver.connect()
            if self._connected:
                logger.info("Successfully connected to %s", 
                           "simulation" if self.mode == HardwareMode.SIMULATION else "hardware")
                # Read initial positions
                self._current_positions = self._driver.read_joint_positions()
            return self._connected
            
        except Exception as e:
            logger.error("Connection failed: %s", e)
            return False
    
    def disconnect(self) -> None:
        """Disconnect from hardware or simulation."""
        if self._driver is not None:
            self._driver.disconnect()
            self._connected = False
            logger.info("Disconnected")
    
    def is_connected(self) -> bool:
        """Check if connected."""
        return self._connected and (self._driver is not None and self._driver.is_connected())
    
    def get_joint_state_topic(self) -> str:
        """Get correct joint state topic based on mode."""
        if self.mode == HardwareMode.SIMULATION:
            return '/joint_states'  # MoveIt2 simulation
        else:
            return f'{self.config.hardware_ns}/joint_states'
    
    def get_trajectory_topic(self) -> str:
        """Get correct trajectory command topic."""
        if self.mode == HardwareMode.SIMULATION:
            return '/execute_trajectory'  # MoveIt2 action
        else:
            return f'{self.config.hardware_ns}/joint_trajectory'
    
    def read_joint_positions(self) -> List[float]:
        """
        Read current joint positions.
        
        Returns:
            List of 6 joint positions in radians.
        """
        if not self.is_connected():
            raise RuntimeError("Not connected to hardware")
        
        self._current_positions = self._driver.read_joint_positions()
        return self._current_positions
    
    def move_to_joint_state(
        self, 
        positions: List[float], 
        time_ms: int = 1000,
        wait: bool = True
    ) -> bool:
        """
        Move to a specified joint state.
        
        Args:
            positions: List of 6 joint positions in radians.
            time_ms: Execution time in milliseconds.
            wait: Whether to wait for motion to complete.
            
        Returns:
            True if successful.
        """
        if not self.is_connected():
            logger.error("Not connected")
            return False
        
        if len(positions) != 6:
            logger.error("Expected 6 positions, got %d", len(positions))
            return False
        
        # Apply velocity scaling to time
        scaled_time = int(time_ms / self.config.velocity_scaling)
        
        try:
            success = self._driver.write_joint_positions(positions, scaled_time)
            
            if success and wait:
                # Wait for motion to complete
                time.sleep(scaled_time / 1000.0)
                
                # Update current positions
                self._current_positions = positions
            
            return success
            
        except Exception as e:
            logger.error("Move failed: %s", e)
            return False
    
    def move_to_named_pose(self, name: str, time_ms: int = 1000) -> bool:
        """
        Move to a named pose.
        
        Args:
            name: Name of the pose (e.g., 'home', 'ready').
            time_ms: Execution time in milliseconds.
            
        Returns:
            True if successful.
        """
        if name not in self.NAMED_POSES:
            logger.error("Unknown pose: %s", name)
            return False
        
        return self.move_to_joint_state(self.NAMED_POSES[name], time_ms)
    
    def move_to_pose(self, x: float, y: float, z: float) -> bool:
        """
        Move end effector to a Cartesian pose.
        
        Uses MoveIt for IK solving if available, otherwise fails.
        
        Args:
            x: X position in meters.
            y: Y position in meters.
            z: Z position in meters.
            
        Returns:
            True if successful.
        """
        if self._moveit_interface is None:
            logger.error("MoveIt interface required for Cartesian moves")
            return False
        
        try:
            return self._moveit_interface.move_to_pose(x, y, z)
        except Exception as e:
            logger.error("Move to pose failed: %s", e)
            return False
    
    def set_gripper(
        self, 
        closed: bool = True, 
        width: Optional[float] = None,
        time_ms: int = 500
    ) -> bool:
        """
        Control the gripper.
        
        Args:
            closed: True to close gripper, False to open.
            width: Optional specific gripper width (0.0 to 0.8).
            time_ms: Execution time in milliseconds.
            
        Returns:
            True if successful.
        """
        if not self.is_connected():
            logger.error("Not connected")
            return False
        
        # Determine gripper position
        if width is not None:
            gripper_pos = max(0.0, min(0.8, width))
        else:
            # Closed: ~0.0, Open: ~0.4 radians
            gripper_pos = 0.0 if closed else 0.4
        
        # Read current positions
        current = self._current_positions.copy()
        current[5] = gripper_pos  # Gripper is joint 6 (index 5)
        
        return self.move_to_joint_state(current, time_ms)
    
    def stop_motion(self) -> bool:
        """
        Immediately stop all motion.
        
        Returns:
            True if successful.
        """
        if not self.is_connected():
            return False
        
        return self._driver.stop_motion()
    
    def switch_mode(self, new_mode: HardwareMode) -> bool:
        """
        Safely switch between simulation and hardware modes.
        
        Args:
            new_mode: The new hardware mode.
            
        Returns:
            True if switch successful.
        """
        logger.info("Switching mode from %s to %s", self.mode.value, new_mode.value)
        
        # 1. Stop any ongoing motion
        self.stop_motion()
        
        # 2. Wait for joints to settle
        time.sleep(0.5)
        
        # 3. Disconnect current driver
        if self._driver is not None and self._driver.is_connected():
            self._driver.disconnect()
        
        # 4. Update mode
        self.mode = new_mode
        self.config.mode = new_mode
        
        # 5. Create new driver
        self._create_driver()
        
        # 6. Verify connection
        self._connected = self._driver.connect()
        
        if self._connected:
            # 7. Read current positions
            try:
                self._current_positions = self._driver.read_joint_positions()
                logger.info("Mode switch successful, current positions: %s", 
                           self._current_positions)
            except Exception as e:
                logger.warning("Could not read positions after switch: %s", e)
        
        return self._connected
    
    def get_current_positions(self) -> List[float]:
        """Get the last known joint positions."""
        return self._current_positions.copy()
    
    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()
        return False


def create_interface(
    mode: str = 'simulation',
    **kwargs
) -> UnifiedMotionInterface:
    """
    Factory function to create a motion interface.
    
    Args:
        mode: Mode string ('simulation', 'hardware', 'hybrid_motion', 'hybrid_vision').
        **kwargs: Additional arguments passed to UnifiedMotionInterface.
        
    Returns:
        Configured UnifiedMotionInterface instance.
    """
    hardware_mode = HardwareMode(mode)
    return UnifiedMotionInterface(mode=hardware_mode, **kwargs)