# Copyright (c) 2024 DOFBOT Project
# SPDX-License-Identifier: BSD-3-Clause

"""
Mock hardware driver for testing DOFBOT control software without physical hardware.

This module provides a simulated hardware driver that implements the same interface
as DofbotDriver, enabling:
    - Unit testing without hardware
    - CI/CD pipeline testing
    - Development without physical robot
    - Simulated physics for realistic behavior
    - Failure injection for error handling tests

Key Features:
    - Identical interface to DofbotDriver
    - Physics simulation with position interpolation
    - Configurable communication delay and noise
    - Failure injection mechanisms for testing
    - Command recording for test assertions
"""

import logging
import math
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any
from enum import Enum

from .exceptions import (
    DofbotCommunicationError,
    DofbotConnectionError,
    DofbotJointError,
    DofbotValueError,
)

# Configure module logger
logger = logging.getLogger(__name__)


class FailureType(Enum):
    """Types of failures that can be injected for testing."""
    CONNECTION_LOST = 'connection_lost'
    TIMEOUT = 'timeout'
    JOINT_STUCK = 'joint_stuck'
    NOISE = 'noise'
    READ_FAILURE = 'read_failure'
    WRITE_FAILURE = 'write_failure'


@dataclass
class CommandRecord:
    """Record of a command sent to the driver.
    
    Attributes:
        timestamp: When the command was executed.
        command_type: 'read' or 'write'.
        joint_id: Specific joint if single-joint command, None otherwise.
        positions: Position values (for write commands).
        time_ms: Execution time in ms (for write commands).
    """
    timestamp: float
    command_type: str
    joint_id: Optional[int] = None
    positions: Optional[List[float]] = None
    time_ms: Optional[int] = None


@dataclass
class MockConfig:
    """Configuration for the mock driver behavior.
    
    Attributes:
        communication_delay_ms: Simulated serial communication delay.
        position_noise_std: Standard deviation of position noise in degrees.
        failure_rate: Probability of random failure (0.0 to 1.0).
        max_velocity: Maximum servo velocity in degrees/second.
        auto_start: Automatically start physics thread on connect.
        enable_physics: Enable position interpolation physics.
    """
    communication_delay_ms: int = 5
    position_noise_std: float = 0.1
    failure_rate: float = 0.0
    max_velocity: float = 90.0  # deg/s
    auto_start: bool = True
    enable_physics: bool = True


@dataclass
class JointState:
    """State of a single simulated joint.
    
    Attributes:
        position: Current position in degrees.
        velocity: Current velocity in degrees/second.
        target_position: Target position in degrees.
        is_stuck: Whether joint is stuck (failure injection).
    """
    position: float = 90.0
    velocity: float = 0.0
    target_position: float = 90.0
    is_stuck: bool = False


class MockDofbotDriver:
    """Mock hardware driver for testing without physical DOFBOT.
    
    This driver simulates the DOFBOT hardware behavior including:
        - Position interpolation with velocity limits
        - Communication delays
        - Position noise
        - Failure injection for testing error handling
    
    The driver implements the same interface as DofbotDriver, allowing
    seamless switching between real and simulated hardware.
    
    Example:
        >>> config = MockConfig(communication_delay_ms=10)
        >>> with MockDofbotDriver(config) as driver:
        ...     positions = driver.read_joint_positions()
        ...     driver.write_joint_positions([0.1, 0.2, 0.3, 0.4, 0.5, 0.0], 1000)
        ...     history = driver.get_command_history()
    """
    
    # Hardware constants (matching real driver)
    NUM_JOINTS = 6
    MIN_JOINT_ID = 1
    MAX_JOINT_ID = 6
    DEGREE_OFFSET = 90.0
    
    # Physics update rate
    PHYSICS_UPDATE_RATE = 100  # Hz
    
    def __init__(self, config: Optional[MockConfig] = None) -> None:
        """Initialize the mock hardware driver.
        
        Args:
            config: Configuration for mock behavior. Uses defaults if None.
        """
        self._config = config or MockConfig()
        self._connected = False
        self._lock = threading.Lock()
        
        # Joint states (stored in degrees internally)
        self._joint_states: List[JointState] = [
            JointState() for _ in range(self.NUM_JOINTS)
        ]
        
        # Physics simulation thread
        self._physics_thread: Optional[threading.Thread] = None
        self._physics_running = False
        
        # Command history for test assertions
        self._command_history: List[CommandRecord] = []
        self._max_history = 1000
        
        # Injected failures
        self._injected_failures: Dict[str, Any] = {}
        
        logger.info("MockDofbotDriver initialized with config: %s", self._config)
    
    def __enter__(self) -> 'MockDofbotDriver':
        """Context manager entry - connect to simulated hardware."""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Context manager exit - disconnect from simulated hardware."""
        self.disconnect()
    
    def _degrees_to_radians(self, degrees: float, joint_index: int = 0) -> float:
        """Convert hardware degrees to ROS2 radians.
        
        Args:
            degrees: Angle in degrees (0-180).
            joint_index: Joint index (unused, for interface compatibility).
            
        Returns:
            Angle in radians (-π/2 to +π/2).
        """
        return (degrees - self.DEGREE_OFFSET) * math.pi / 180.0
    
    def _radians_to_degrees(self, radians: float, joint_index: int = 0) -> float:
        """Convert ROS2 radians to hardware degrees.
        
        Args:
            radians: Angle in radians (-π/2 to +π/2).
            joint_index: Joint index (unused, for interface compatibility).
            
        Returns:
            Angle in degrees (0-180).
        """
        return radians * 180.0 / math.pi + self.DEGREE_OFFSET
    
    def _clamp_degrees(self, degrees: float) -> float:
        """Clamp degrees to valid hardware range [0, 180]."""
        return max(0.0, min(180.0, degrees))
    
    def _validate_joint_id(self, joint_id: int) -> None:
        """Validate joint ID is within valid range."""
        if not (self.MIN_JOINT_ID <= joint_id <= self.MAX_JOINT_ID):
            raise DofbotJointError(
                joint_id,
                f"Invalid joint ID. Must be {self.MIN_JOINT_ID}-{self.MAX_JOINT_ID}"
            )
    
    def _validate_positions(self, positions: List[float]) -> None:
        """Validate positions list has correct length."""
        if len(positions) != self.NUM_JOINTS:
            raise DofbotValueError(
                f"Expected {self.NUM_JOINTS} positions, got {len(positions)}"
            )
    
    def _add_command_record(self, command_type: str, joint_id: Optional[int] = None,
                            positions: Optional[List[float]] = None,
                            time_ms: Optional[int] = None) -> None:
        """Record a command for test assertions."""
        record = CommandRecord(
            timestamp=time.time(),
            command_type=command_type,
            joint_id=joint_id,
            positions=positions.copy() if positions else None,
            time_ms=time_ms
        )
        self._command_history.append(record)
        
        # Trim history if too long
        if len(self._command_history) > self._max_history:
            self._command_history = self._command_history[-self._max_history:]
    
    def _simulate_delay(self) -> None:
        """Simulate communication delay."""
        if self._config.communication_delay_ms > 0:
            time.sleep(self._config.communication_delay_ms / 1000.0)
    
    def _should_fail_randomly(self) -> bool:
        """Check if a random failure should occur."""
        import random
        return random.random() < self._config.failure_rate
    
    def _add_noise(self, degrees: float) -> float:
        """Add Gaussian noise to position reading."""
        if self._config.position_noise_std > 0:
            import random
            noise = random.gauss(0, self._config.position_noise_std)
            return degrees + noise
        return degrees
    
    def _physics_loop(self) -> None:
        """Physics simulation loop for position interpolation.
        
        This runs in a separate thread and updates joint positions
        towards their targets with velocity limiting.
        """
        dt = 1.0 / self.PHYSICS_UPDATE_RATE
        
        while self._physics_running:
            with self._lock:
                for i, state in enumerate(self._joint_states):
                    if state.is_stuck:
                        continue
                    
                    # Calculate distance to target
                    error = state.target_position - state.position
                    
                    if abs(error) < 0.01:  # Close enough
                        state.velocity = 0.0
                        continue
                    
                    # Calculate velocity (trapezoidal profile)
                    max_vel = self._config.max_velocity
                    direction = 1.0 if error > 0 else -1.0
                    
                    # Simple velocity limiting
                    state.velocity = direction * min(max_vel, abs(error) / dt)
                    
                    # Update position
                    state.position += state.velocity * dt
                    
                    # Clamp to valid range
                    state.position = self._clamp_degrees(state.position)
            
            time.sleep(dt)
    
    def connect(self) -> bool:
        """Establish connection to simulated hardware.
        
        Returns:
            Always True for mock driver.
            
        Raises:
            DofbotConnectionError: If connection_lost failure is injected.
        """
        # Check for injected connection failure (outside lock to allow testing)
        if 'connection_lost' in self._injected_failures:
            raise DofbotConnectionError("Simulated connection lost")
        
        with self._lock:
            if self._connected:
                logger.warning("Already connected to mock hardware")
                return True
            
            self._connected = True
            
            # Start physics thread if enabled
            if self._config.enable_physics and self._config.auto_start:
                self._physics_running = True
                self._physics_thread = threading.Thread(
                    target=self._physics_loop,
                    daemon=True
                )
                self._physics_thread.start()
            
            logger.info("Connected to mock hardware")
            return True
    
    def disconnect(self) -> None:
        """Disconnect from simulated hardware."""
        with self._lock:
            if not self._connected:
                return
            
            # Stop physics thread
            if self._physics_running:
                self._physics_running = False
                if self._physics_thread:
                    self._physics_thread.join(timeout=1.0)
                    self._physics_thread = None
            
            self._connected = False
            logger.info("Disconnected from mock hardware")
    
    def is_connected(self) -> bool:
        """Check if mock hardware is connected."""
        return self._connected
    
    def read_joint_positions(self) -> List[float]:
        """Read all joint positions from simulated hardware.
        
        Returns:
            List of 6 joint positions in radians.
        """
        if not self._connected:
            raise DofbotConnectionError("Not connected to mock hardware")
        
        # Check for injected failures
        if 'connection_lost' in self._injected_failures:
            raise DofbotConnectionError("Simulated connection lost")
        
        if 'timeout' in self._injected_failures:
            raise DofbotCommunicationError("Simulated timeout")
        
        if 'read_failure' in self._injected_failures:
            raise DofbotCommunicationError("Simulated read failure")
        
        if self._should_fail_randomly():
            raise DofbotCommunicationError("Random simulated failure")
        
        self._simulate_delay()
        
        with self._lock:
            positions = []
            for i, state in enumerate(self._joint_states):
                # Add noise if enabled
                position_deg = self._add_noise(state.position)
                position_rad = self._degrees_to_radians(position_deg)
                positions.append(position_rad)
            
            self._add_command_record('read', positions=positions)
            return positions
    
    def write_joint_positions(self, positions: List[float], time_ms: int) -> bool:
        """Write all joint positions to simulated hardware.
        
        Args:
            positions: List of 6 joint positions in radians.
            time_ms: Time in milliseconds to reach target positions.
            
        Returns:
            True if write successful.
        """
        self._validate_positions(positions)
        
        if not self._connected:
            raise DofbotConnectionError("Not connected to mock hardware")
        
        # Check for injected failures
        if 'connection_lost' in self._injected_failures:
            raise DofbotConnectionError("Simulated connection lost")
        
        if 'write_failure' in self._injected_failures:
            raise DofbotCommunicationError("Simulated write failure")
        
        if self._should_fail_randomly():
            raise DofbotCommunicationError("Random simulated failure")
        
        self._simulate_delay()
        
        with self._lock:
            for i, pos_rad in enumerate(positions):
                pos_deg = self._clamp_degrees(self._radians_to_degrees(pos_rad))
                
                # Check if joint is stuck
                if not self._joint_states[i].is_stuck:
                    self._joint_states[i].target_position = pos_deg
                    
                    # If physics disabled, set position immediately
                    if not self._config.enable_physics:
                        self._joint_states[i].position = pos_deg
            
            self._add_command_record('write', positions=positions, time_ms=time_ms)
            logger.debug(f"Wrote mock positions: {positions} rad, {time_ms}ms")
            return True
    
    def read_single_joint(self, joint_id: int) -> Optional[float]:
        """Read a single joint position.
        
        Args:
            joint_id: Joint ID (1-6 for DOFBOT).
            
        Returns:
            Joint position in radians.
        """
        self._validate_joint_id(joint_id)
        
        if not self._connected:
            raise DofbotConnectionError("Not connected to mock hardware")
        
        self._simulate_delay()
        
        with self._lock:
            state = self._joint_states[joint_id - 1]
            position_deg = self._add_noise(state.position)
            position_rad = self._degrees_to_radians(position_deg)
            
            self._add_command_record('read', joint_id=joint_id)
            return position_rad
    
    def write_single_joint(self, joint_id: int, angle: float, time_ms: int) -> bool:
        """Write a single joint position.
        
        Args:
            joint_id: Joint ID (1-6 for DOFBOT).
            angle: Joint angle in radians.
            time_ms: Time in milliseconds to reach target angle.
            
        Returns:
            True if write successful.
        """
        self._validate_joint_id(joint_id)
        
        if not self._connected:
            raise DofbotConnectionError("Not connected to mock hardware")
        
        self._simulate_delay()
        
        with self._lock:
            idx = joint_id - 1
            pos_deg = self._clamp_degrees(self._radians_to_degrees(angle))
            
            if not self._joint_states[idx].is_stuck:
                self._joint_states[idx].target_position = pos_deg
                
                if not self._config.enable_physics:
                    self._joint_states[idx].position = pos_deg
            
            self._add_command_record('write', joint_id=joint_id,
                                     positions=[angle], time_ms=time_ms)
            return True
    
    # =====================
    # Test Utility Methods
    # =====================
    
    def inject_failure(self, failure_type, joint_id: Optional[int] = None) -> None:
        """Inject a failure for testing error handling.
        
        Args:
            failure_type: Type of failure to inject (FailureType enum or string).
                - 'connection_lost': Simulate serial disconnection
                - 'timeout': Read operations raise timeout error
                - 'joint_stuck': Specified joint doesn't move
                - 'noise': Add extra noise to position readings
                - 'read_failure': Read operations fail
                - 'write_failure': Write operations fail
            joint_id: Joint ID for 'joint_stuck' failure.
        """
        # Handle FailureType enum
        if hasattr(failure_type, 'value'):
            failure_type = failure_type.value
        
        if failure_type == 'joint_stuck' and joint_id is not None:
            self._validate_joint_id(joint_id)
            with self._lock:
                self._joint_states[joint_id - 1].is_stuck = True
        else:
            self._injected_failures[failure_type] = True
        
        logger.info(f"Injected failure: {failure_type}" + 
                   (f" (joint {joint_id})" if joint_id else ""))
    
    def clear_failure(self, failure_type, joint_id: Optional[int] = None) -> None:
        """Clear an injected failure.
        
        Args:
            failure_type: Type of failure to clear (FailureType enum or string).
            joint_id: Joint ID for 'joint_stuck' failure.
        """
        # Handle FailureType enum
        if hasattr(failure_type, 'value'):
            failure_type = failure_type.value
        
        if failure_type == 'joint_stuck' and joint_id is not None:
            with self._lock:
                self._joint_states[joint_id - 1].is_stuck = False
        elif failure_type in self._injected_failures:
            del self._injected_failures[failure_type]
        
        logger.info(f"Cleared failure: {failure_type}")
    
    def clear_all_failures(self) -> None:
        """Clear all injected failures."""
        with self._lock:
            self._injected_failures.clear()
            for state in self._joint_states:
                state.is_stuck = False
        
        logger.info("Cleared all failures")
    
    def get_command_history(self) -> List[CommandRecord]:
        """Get history of all commands sent to the driver.
        
        Returns:
            List of CommandRecord objects.
        """
        return self._command_history.copy()
    
    def clear_command_history(self) -> None:
        """Clear the command history."""
        self._command_history.clear()
    
    def clear_history(self) -> None:
        """Alias for clear_command_history for backward compatibility."""
        self.clear_command_history()
    
    def get_internal_positions(self) -> List[float]:
        """Get internal joint positions in degrees (for testing).
        
        Returns:
            List of 6 joint positions in degrees [0-180].
        """
        with self._lock:
            return [s.position for s in self._joint_states]
    
    def get_joint_states(self) -> List[JointState]:
        """Get current joint states (for debugging/testing).
        
        Returns:
            List of JointState objects.
        """
        with self._lock:
            return [JointState(
                position=s.position,
                velocity=s.velocity,
                target_position=s.target_position,
                is_stuck=s.is_stuck
            ) for s in self._joint_states]
    
    def set_joint_position_direct(self, joint_id: int, position_deg: float) -> None:
        """Directly set a joint position bypassing physics.
        
        Useful for setting up test scenarios.
        
        Args:
            joint_id: Joint ID (1-6).
            position_deg: Position in degrees.
        """
        self._validate_joint_id(joint_id)
        with self._lock:
            self._joint_states[joint_id - 1].position = self._clamp_degrees(position_deg)
            self._joint_states[joint_id - 1].target_position = self._clamp_degrees(position_deg)
    
    def wait_for_motion_complete(self, timeout: float = 5.0) -> bool:
        """Wait for all joints to reach their target positions.
        
        Args:
            timeout: Maximum time to wait in seconds.
            
        Returns:
            True if motion completed, False if timeout.
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            with self._lock:
                all_reached = all(
                    abs(s.position - s.target_position) < 0.1
                    for s in self._joint_states
                )
            
            if all_reached:
                return True
            
            time.sleep(0.01)
        
        return False
    
    def get_hardware_info(self) -> dict:
        """Get mock hardware information and status."""
        return {
            'connected': self._connected,
            'num_joints': self.NUM_JOINTS,
            'config': self._config,
            'joint_states': [
                {
                    'position': s.position,
                    'velocity': s.velocity,
                    'target': s.target_position,
                    'is_stuck': s.is_stuck
                }
                for s in self._joint_states
            ],
            'injected_failures': list(self._injected_failures.keys()),
            'command_history_count': len(self._command_history)
        }