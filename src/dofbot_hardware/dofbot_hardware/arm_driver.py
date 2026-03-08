# Copyright (c) 2024 DOFBOT Project
# SPDX-License-Identifier: BSD-3-Clause

"""
Hardware driver for DOFBOT robot arm.

This module provides the production driver that interfaces with the
physical DOFBOT hardware through the Arm_Lib library.

Usage:
    driver = DofbotDriver()
    try:
        if driver.connect():
            positions = driver.read_joint_positions()
            driver.write_joint_positions([0.0, 0.5, 0.0, 0.0, 0.0, 0.4], 1000)
    finally:
        driver.disconnect()
"""

import math
import time
import threading
import logging
from typing import List, Optional

from .driver_interface import DofbotDriverInterface, JointLimits, DEFAULT_JOINT_LIMITS, JOINT_NAMES
from .exceptions import (
    DofbotConnectionError,
    DofbotCommunicationError,
    DofbotTimeoutError,
    DofbotJointError,
    DofbotValueError,
)


logger = logging.getLogger(__name__)


class DofbotDriver:
    """
    Production hardware driver for DOFBOT robot arm.
    
    This driver interfaces with the physical DOFBOT hardware through
    the Arm_Lib Python library, which wraps the C serial communication.
    
    Features:
    - Thread-safe hardware access
    - Degree-to-radian conversion with joint offsets
    - Retry logic with exponential backoff
    - Proper error handling
    
    Thread Safety:
        All hardware access is protected by a lock to prevent
        concurrent serial communication issues.
    """
    
    def __init__(
        self, 
        max_retries: int = 3,
        retry_delay_base: float = 0.01,
        read_timeout: float = 0.1
    ):
        """
        Initialize the hardware driver.
        
        Args:
            max_retries: Maximum number of retries for failed operations.
            retry_delay_base: Base delay in seconds for exponential backoff.
            read_timeout: Timeout for read operations in seconds.
        """
        self._max_retries = max_retries
        self._retry_delay_base = retry_delay_base
        self._read_timeout = read_timeout
        
        # Hardware interface
        self._arm = None
        self._connected = False
        self._lock = threading.Lock()
        
        # Joint limits
        self._joint_limits = DEFAULT_JOINT_LIMITS
        
        logger.info("DofbotDriver initialized")
    
    def connect(self) -> bool:
        """
        Establish connection to the DOFBOT hardware.
        
        Creates an Arm_Device instance which opens serial communication.
        
        Returns:
            True if connection successful.
            
        Raises:
            DofbotConnectionError: If connection fails.
        """
        with self._lock:
            if self._connected:
                logger.warning("Already connected to hardware")
                return True
            
            try:
                # Import Arm_Lib here to allow module to load without hardware
                import Arm_Lib
                self._arm = Arm_Lib.Arm_Device()
                self._connected = True
                logger.info("Successfully connected to DOFBOT hardware")
                return True
                
            except ImportError as e:
                logger.error("Arm_Lib not available: %s", e)
                raise DofbotConnectionError(
                    "Arm_Lib library not found. Please ensure DOFBOT drivers are installed."
                )
            except Exception as e:
                logger.error("Failed to connect to hardware: %s", e)
                raise DofbotConnectionError(f"Failed to connect to DOFBOT: {e}")
    
    def disconnect(self) -> None:
        """
        Disconnect from the hardware.
        
        Safe to call multiple times.
        """
        with self._lock:
            if not self._connected:
                return
            
            try:
                # Arm_Device doesn't have explicit close, just release reference
                self._arm = None
            except Exception as e:
                logger.warning("Error during disconnect: %s", e)
            finally:
                self._connected = False
                logger.info("Disconnected from DOFBOT hardware")
    
    def is_connected(self) -> bool:
        """Check if connected to hardware."""
        with self._lock:
            return self._connected
    
    def read_joint_positions(self) -> List[float]:
        """
        Read all joint positions from hardware.
        
        Returns:
            List of 6 joint positions in radians.
            
        Raises:
            DofbotConnectionError: If not connected.
            DofbotCommunicationError: If read fails after retries.
        """
        with self._lock:
            self._check_connected()
            
            positions = []
            for joint_id in range(1, 7):  # Joints 1-6
                angle = self._read_with_retry(joint_id)
                if angle is None:
                    raise DofbotCommunicationError(
                        f"Failed to read joint {joint_id} after {self._max_retries} retries"
                    )
                positions.append(self._degrees_to_radians(angle, joint_id - 1))
            
            return positions
    
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
            True if successful.
            
        Raises:
            DofbotValueError: If positions length is incorrect.
            DofbotConnectionError: If not connected.
            DofbotCommunicationError: If write fails.
        """
        with self._lock:
            self._check_connected()
            
            if len(positions) != 6:
                raise DofbotValueError(
                    f"Expected 6 joint positions, got {len(positions)}"
                )
            
            # Convert to degrees
            degree_positions = [
                self._radians_to_degrees(p, i) for i, p in enumerate(positions)
            ]
            
            # Clamp to valid range (0-180 degrees)
            degree_positions = [
                max(0.0, min(180.0, p)) for p in degree_positions
            ]
            
            try:
                # Use write6_array for all joints at once
                result = self._arm.Arm_serial_servo_write6_array(
                    degree_positions, 
                    time_ms
                )
                
                if result:
                    logger.debug(
                        "Wrote positions: %s (time: %dms)", 
                        degree_positions, 
                        time_ms
                    )
                return bool(result)
                
            except Exception as e:
                logger.error("Write failed: %s", e)
                raise DofbotCommunicationError(f"Failed to write joints: {e}")
    
    def read_single_joint(self, joint_id: int) -> Optional[float]:
        """
        Read a single joint position.
        
        Args:
            joint_id: Joint ID (1-6).
            
        Returns:
            Joint position in radians, or None if read fails.
        """
        self._validate_joint_id(joint_id)
        
        with self._lock:
            self._check_connected()
            
            angle = self._read_with_retry(joint_id)
            if angle is None:
                return None
            
            return self._degrees_to_radians(angle, joint_id - 1)
    
    def write_single_joint(
        self, 
        joint_id: int, 
        angle: float, 
        time_ms: int
    ) -> bool:
        """
        Write a single joint position.
        
        Args:
            joint_id: Joint ID (1-6).
            angle: Joint angle in radians.
            time_ms: Execution time in milliseconds.
            
        Returns:
            True if successful.
        """
        self._validate_joint_id(joint_id)
        
        with self._lock:
            self._check_connected()
            
            # Convert to degrees and clamp
            degree_angle = self._radians_to_degrees(angle, joint_id - 1)
            degree_angle = max(0.0, min(180.0, degree_angle))
            
            try:
                result = self._arm.Arm_serial_servo_write(
                    joint_id, 
                    degree_angle, 
                    time_ms
                )
                return bool(result)
                
            except Exception as e:
                logger.error("Write joint %d failed: %s", joint_id, e)
                return False
    
    def stop_motion(self) -> bool:
        """
        Immediately stop all motion.
        
        Commands current positions with 0ms time to halt movement.
        
        Returns:
            True if successful.
        """
        with self._lock:
            if not self._connected:
                return False
            
            try:
                # Read current positions
                current_positions = []
                for joint_id in range(1, 7):
                    angle = self._arm.Arm_serial_servo_read(joint_id)
                    if angle < 0:
                        angle = 90.0  # Default to center on read failure
                    current_positions.append(angle)
                
                # Command same positions with 0ms to stop
                self._arm.Arm_serial_servo_write6_array(current_positions, 0)
                logger.info("Motion stopped")
                return True
                
            except Exception as e:
                logger.error("Stop motion failed: %s", e)
                return False
    
    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()
        return False
    
    # Private methods
    
    def _check_connected(self):
        """Check if connected, raise if not."""
        if not self._connected or self._arm is None:
            raise DofbotConnectionError("Not connected to hardware")
    
    def _validate_joint_id(self, joint_id: int):
        """Validate joint ID is in range 1-6."""
        if joint_id < 1 or joint_id > 6:
            raise DofbotJointError(
                joint_id, 
                f"Invalid joint ID {joint_id}. Must be 1-6."
            )
    
    def _read_with_retry(self, joint_id: int) -> Optional[float]:
        """
        Read a joint position with retry logic.
        
        Args:
            joint_id: Joint ID (1-6).
            
        Returns:
            Position in degrees, or None if all retries fail.
        """
        for attempt in range(self._max_retries):
            try:
                angle = self._arm.Arm_serial_servo_read(joint_id)
                
                # Arm_serial_servo_read returns -1 on error
                if angle < 0:
                    raise DofbotCommunicationError(
                        f"Read returned error value: {angle}"
                    )
                
                return float(angle)
                
            except Exception as e:
                logger.warning(
                    "Read joint %d failed (attempt %d/%d): %s",
                    joint_id, 
                    attempt + 1, 
                    self._max_retries,
                    e
                )
                
                if attempt < self._max_retries - 1:
                    # Exponential backoff
                    delay = self._retry_delay_base * (2 ** attempt)
                    time.sleep(delay)
        
        return None
    
    def _degrees_to_radians(self, degrees: float, joint_index: int) -> float:
        """
        Convert degrees to radians.
        
        Hardware convention: 0-180 degrees
        ROS convention: -π/2 to +π/2 radians
        Center (90°) maps to 0 radians.
        
        Args:
            degrees: Angle in degrees (0-180).
            joint_index: Index of the joint (0-5).
            
        Returns:
            Angle in radians.
        """
        return (degrees - 90.0) * math.pi / 180.0
    
    def _radians_to_degrees(self, radians: float, joint_index: int) -> float:
        """
        Convert radians to degrees.
        
        ROS convention: -π/2 to +π/2 radians
        Hardware convention: 0-180 degrees
        
        Args:
            radians: Angle in radians.
            joint_index: Index of the joint (0-5).
            
        Returns:
            Angle in degrees (0-180).
        """
        return radians * 180.0 / math.pi + 90.0