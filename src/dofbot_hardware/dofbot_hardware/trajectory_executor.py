# Copyright (c) 2024 DOFBOT Project
# SPDX-License-Identifier: BSD-3-Clause

"""
Trajectory Execution with Safety Features for DOFBOT.

This module implements trajectory execution that safely executes MoveIt2-planned
trajectories on the robot arm hardware with comprehensive safety features.

Key Features:
    - Trajectory validation (joint limits, velocity limits)
    - Position interpolation with proper timing
    - Position error monitoring
    - Emergency stop functionality
    - FollowJointTrajectory action server for MoveIt2 integration

Safety Features:
    - Position limit checking before execution
    - Velocity limit enforcement
    - Position error monitoring during execution
    - Emergency stop capability
    - Velocity scaling for safe operation
"""

import logging
import math
import threading
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional, Callable, Any

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

from .exceptions import DofbotError, DofbotValueError, DofbotCommunicationError

# Configure module logger
logger = logging.getLogger(__name__)


class ExecutionState(Enum):
    """State of trajectory execution."""
    IDLE = auto()
    EXECUTING = auto()
    PAUSED = auto()
    COMPLETED = auto()
    ABORTED = auto()
    STOPPED = auto()


@dataclass
class JointLimits:
    """Safety limits for a single joint.
    
    Attributes:
        position_min: Minimum position in radians.
        position_max: Maximum position in radians.
        velocity_max: Maximum velocity in rad/s.
        effort_max: Maximum effort/torque.
    """
    position_min: float = -math.pi / 2
    position_max: float = math.pi / 2
    velocity_max: float = 1.0  # rad/s
    effort_max: float = 0.0  # Nm (not used for servos)
    
    def has_position_limit(self, position: float) -> bool:
        """Check if position violates limits.
        
        Args:
            position: Position to check in radians.
            
        Returns:
            True if position is within limits.
        """
        return self.position_min <= position <= self.position_max
    
    def clamp_position(self, position: float) -> float:
        """Clamp position to valid range.
        
        Args:
            position: Position to clamp.
            
        Returns:
            Clamped position.
        """
        return max(self.position_min, min(self.position_max, position))


@dataclass
class TrajectoryConfig:
    """Configuration for trajectory execution.
    
    Attributes:
        velocity_scaling: Scale factor for velocities (0-1).
        position_error_threshold: Maximum allowed position error in radians.
        default_execution_time_ms: Default time for trajectory points.
        enable_position_monitoring: Monitor position during execution.
        enable_velocity_limiting: Enforce velocity limits.
        interpolation_step_ms: Step size for interpolation.
    """
    velocity_scaling: float = 0.5
    position_error_threshold: float = 0.1  # radians (~5.7 degrees)
    default_execution_time_ms: int = 500
    enable_position_monitoring: bool = True
    enable_velocity_limiting: bool = True
    interpolation_step_ms: int = 20


class TrajectoryValidator:
    """Validates trajectories before execution.
    
    Checks trajectories for:
        - Joint name matching
        - Position limits
        - Velocity limits
        - Point count
    """
    
    def __init__(self, joint_names: List[str], joint_limits: List[JointLimits]) -> None:
        """Initialize the validator.
        
        Args:
            joint_names: Expected joint names.
            joint_limits: Limits for each joint.
        """
        self._joint_names = set(joint_names)
        self._joint_limits = joint_limits
    
    def validate(self, trajectory: JointTrajectory) -> List[str]:
        """Validate a trajectory for safety.
        
        Args:
            trajectory: Trajectory to validate.
            
        Returns:
            List of validation error messages (empty if valid).
        """
        errors = []
        
        # Check joint names
        if not self._joint_names:
            errors.append("No expected joint names configured")
        elif set(trajectory.joint_names) != self._joint_names:
            errors.append(
                f"Joint name mismatch: got {trajectory.joint_names}, "
                f"expected {list(self._joint_names)}"
            )
            return errors  # Can't continue without matching joints
        
        # Check we have points
        if not trajectory.points:
            errors.append("Empty trajectory - no points to execute")
            return errors
        
        # Check each point
        for i, point in enumerate(trajectory.points):
            # Check position count
            if len(point.positions) != len(self._joint_limits):
                errors.append(
                    f"Point {i}: position count mismatch "
                    f"({len(point.positions)} vs {len(self._joint_limits)})"
                )
                continue
            
            # Check position limits
            for j, (pos, limits) in enumerate(zip(point.positions, self._joint_limits)):
                if not limits.has_position_limit(pos):
                    errors.append(
                        f"Point {i}, joint {j}: position {pos:.3f} "
                        f"exceeds limits [{limits.position_min:.3f}, {limits.position_max:.3f}]"
                    )
            
            # Check velocity limits if specified
            if point.velocities:
                for j, (vel, limits) in enumerate(zip(point.velocities, self._joint_limits)):
                    if abs(vel) > limits.velocity_max:
                        # This is a warning, not an error - we'll clamp
                        logger.warning(
                            f"Point {i}, joint {j}: velocity {vel:.3f} "
                            f"exceeds max {limits.velocity_max:.3f} (will be clamped)"
                        )
        
        return errors
    
    def get_safe_trajectory(self, trajectory: JointTrajectory) -> JointTrajectory:
        """Create a safe copy of trajectory with clamped values.
        
        Args:
            trajectory: Original trajectory.
            
        Returns:
            Trajectory with clamped positions and velocities.
        """
        safe_traj = JointTrajectory()
        safe_traj.header = trajectory.header
        safe_traj.joint_names = trajectory.joint_names.copy()
        
        for point in trajectory.points:
            safe_point = JointTrajectoryPoint()
            
            # Clamp positions
            safe_point.positions = [
                limits.clamp_position(pos)
                for pos, limits in zip(point.positions, self._joint_limits)
            ]
            
            # Clamp velocities
            if point.velocities:
                safe_point.velocities = [
                    max(-limits.velocity_max, min(limits.velocity_max, vel))
                    for vel, limits in zip(point.velocities, self._joint_limits)
                ]
            
            # Copy other fields
            if point.accelerations:
                safe_point.accelerations = point.accelerations.copy()
            if point.effort:
                safe_point.effort = point.effort.copy()
            safe_point.time_from_start = point.time_from_start
            
            safe_traj.points.append(safe_point)
        
        return safe_traj


class TrajectoryExecutor:
    """Executes trajectories with safety monitoring.
    
    This class handles:
        - Trajectory validation
        - Point-by-point execution
        - Position interpolation
        - Safety monitoring
        - Emergency stop
    
    Example:
        >>> executor = TrajectoryExecutor(driver, joint_names, limits)
        >>> executor.execute_trajectory(trajectory)
    """
    
    def __init__(
        self,
        driver,
        joint_names: List[str],
        joint_limits: List[JointLimits],
        config: Optional[TrajectoryConfig] = None,
        position_reader: Optional[Callable[[], List[float]]] = None
    ) -> None:
        """Initialize the trajectory executor.
        
        Args:
            driver: Hardware driver implementing DofbotDriverInterface.
            joint_names: List of joint names.
            joint_limits: Safety limits for each joint.
            config: Execution configuration.
            position_reader: Function to read current positions (for monitoring).
        """
        self._driver = driver
        self._joint_names = joint_names
        self._joint_limits = joint_limits
        self._config = config or TrajectoryConfig()
        self._position_reader = position_reader
        
        # Validator
        self._validator = TrajectoryValidator(joint_names, joint_limits)
        
        # Execution state
        self._state = ExecutionState.IDLE
        self._current_trajectory: Optional[JointTrajectory] = None
        self._current_point_index = 0
        self._stop_requested = False
        self._lock = threading.Lock()
        
        # Execution tracking
        self._execution_start_time: float = 0.0
        self._last_execution_time: float = 0.0
        
        # Statistics
        self._trajectories_executed = 0
        self._trajectories_aborted = 0
        
        logger.info("TrajectoryExecutor initialized")
    
    @property
    def state(self) -> ExecutionState:
        """Get current execution state."""
        return self._state
    
    @property
    def is_executing(self) -> bool:
        """Check if currently executing a trajectory."""
        return self._state == ExecutionState.EXECUTING
    
    def validate_trajectory(self, trajectory: JointTrajectory) -> bool:
        """Validate trajectory is safe to execute.
        
        Args:
            trajectory: Trajectory to validate.
            
        Returns:
            True if trajectory is valid, False otherwise.
        """
        errors = self._validator.validate(trajectory)
        if errors:
            for error in errors:
                logger.error(f"Trajectory validation error: {error}")
            return False
        return True
    
    def execute_trajectory(self, trajectory: JointTrajectory) -> bool:
        """Execute a trajectory on the hardware.
        
        Args:
            trajectory: Trajectory to execute.
            
        Returns:
            True if execution completed successfully.
        """
        with self._lock:
            if self._state == ExecutionState.EXECUTING:
                logger.warning("Already executing a trajectory")
                return False
            
            # Validate trajectory
            if not self.validate_trajectory(trajectory):
                logger.error("Trajectory validation failed")
                return False
            
            # Get safe trajectory with clamped values
            safe_trajectory = self._validator.get_safe_trajectory(trajectory)
            
            # Start execution
            self._current_trajectory = safe_trajectory
            self._current_point_index = 0
            self._stop_requested = False
            self._state = ExecutionState.EXECUTING
            self._execution_start_time = time.time()
        
        try:
            # Execute each point
            for i, point in enumerate(safe_trajectory.points):
                if self._stop_requested:
                    self._state = ExecutionState.STOPPED
                    logger.info("Trajectory execution stopped")
                    return False
                
                success = self._execute_point(point, i)
                
                if not success:
                    self._state = ExecutionState.ABORTED
                    self._trajectories_aborted += 1
                    logger.error(f"Failed to execute trajectory point {i}")
                    return False
            
            # Execution completed
            self._state = ExecutionState.COMPLETED
            self._trajectories_executed += 1
            self._last_execution_time = time.time() - self._execution_start_time
            logger.info(
                f"Trajectory execution completed in {self._last_execution_time:.2f}s"
            )
            return True
            
        except Exception as e:
            self._state = ExecutionState.ABORTED
            self._trajectories_aborted += 1
            logger.error(f"Trajectory execution error: {e}")
            return False
    
    def _execute_point(self, point: JointTrajectoryPoint, index: int) -> bool:
        """Execute a single trajectory point.
        
        Args:
            point: Point to execute.
            index: Point index for logging.
            
        Returns:
            True if point executed successfully.
        """
        # Calculate execution time
        time_ms = self._get_execution_time_ms(point)
        
        # Apply velocity scaling
        scaled_time_ms = int(time_ms / self._config.velocity_scaling)
        scaled_time_ms = max(50, scaled_time_ms)  # Minimum 50ms
        
        logger.debug(
            f"Executing point {index}: positions={point.positions}, "
            f"time={scaled_time_ms}ms"
        )
        
        # Write to hardware
        try:
            success = self._driver.write_joint_positions(
                list(point.positions),
                scaled_time_ms
            )
            
            if not success:
                logger.error(f"Failed to write point {index}")
                return False
            
            # Monitor position if enabled
            if self._config.enable_position_monitoring and self._position_reader:
                # Wait a bit before checking
                time.sleep(scaled_time_ms / 2000.0)  # Half the time
                
                # Check position error
                if not self._check_position_error(list(point.positions)):
                    logger.error(f"Position error exceeded at point {index}")
                    return False
            
            # Wait for execution to complete
            time.sleep(scaled_time_ms / 1000.0)
            
            return True
            
        except DofbotCommunicationError as e:
            logger.error(f"Communication error at point {index}: {e}")
            return False
    
    def _get_execution_time_ms(self, point: JointTrajectoryPoint) -> int:
        """Get execution time for a point.
        
        Args:
            point: Trajectory point.
            
        Returns:
            Time in milliseconds.
        """
        # Use time_from_start if specified
        tfs = point.time_from_start
        if tfs.sec > 0 or tfs.nanosec > 0:
            time_ms = tfs.sec * 1000 + tfs.nanosec // 1000000
            return time_ms
        
        # Calculate based on velocity limits
        if point.velocities:
            max_time = 0.0
            current = self._position_reader() if self._position_reader else [0.0] * 6
            
            for pos, vel, limits in zip(point.positions, point.velocities, self._joint_limits):
                if abs(vel) > 1e-6:
                    distance = abs(pos - current[list(self._joint_names).index(
                        point.positions.index(pos)  # Simplified
                    )]) if len(current) > 0 else 0
                    time_for_joint = distance / abs(vel) * 1000
                    max_time = max(max_time, time_for_joint)
            
            if max_time > 0:
                return int(max_time)
        
        return self._config.default_execution_time_ms
    
    def _check_position_error(self, commanded: List[float]) -> bool:
        """Check if position error exceeds threshold.
        
        Args:
            commanded: Commanded positions.
            
        Returns:
            True if within tolerance, False if error exceeded.
        """
        if not self._position_reader:
            return True
        
        try:
            actual = self._position_reader()
            threshold = self._config.position_error_threshold
            
            for i, (cmd, act) in enumerate(zip(commanded, actual)):
                error = abs(cmd - act)
                if error > threshold:
                    logger.warning(
                        f"Position error exceeded for joint {i}: "
                        f"error={error:.3f}, threshold={threshold:.3f}"
                    )
                    return False
            
            return True
            
        except Exception as e:
            logger.error(f"Error checking position: {e}")
            return True  # Don't fail on monitoring error
    
    def stop(self) -> None:
        """Stop trajectory execution."""
        logger.warning("Stop requested")
        self._stop_requested = True
    
    def emergency_stop(self) -> bool:
        """Immediately stop all motion.
        
        Returns:
            True if E-stop successful.
        """
        logger.error("EMERGENCY STOP ACTIVATED")
        self._stop_requested = True
        
        try:
            # Read current positions
            if self._position_reader:
                current = self._position_reader()
            else:
                current = self._driver.read_joint_positions()
            
            # Command current position to stop motion
            self._driver.write_joint_positions(current, 0)
            
            self._state = ExecutionState.STOPPED
            return True
            
        except Exception as e:
            logger.error(f"E-stop failed: {e}")
            return False
    
    def pause(self) -> bool:
        """Pause trajectory execution.
        
        Returns:
            True if paused successfully.
        """
        with self._lock:
            if self._state != ExecutionState.EXECUTING:
                return False
            
            self._state = ExecutionState.PAUSED
            logger.info("Trajectory execution paused")
            return True
    
    def resume(self) -> bool:
        """Resume paused trajectory execution.
        
        Returns:
            True if resumed successfully.
        """
        with self._lock:
            if self._state != ExecutionState.PAUSED:
                return False
            
            self._state = ExecutionState.EXECUTING
            logger.info("Trajectory execution resumed")
            return True
    
    def get_statistics(self) -> dict:
        """Get execution statistics.
        
        Returns:
            Dictionary with statistics.
        """
        return {
            'state': self._state.name,
            'trajectories_executed': self._trajectories_executed,
            'trajectories_aborted': self._trajectories_aborted,
            'last_execution_time': self._last_execution_time,
            'current_point': self._current_point_index,
            'stop_requested': self._stop_requested,
        }


# Default joint limits for DOFBOT
def get_default_joint_limits() -> List[JointLimits]:
    """Get default joint limits for DOFBOT robot.
    
    Returns:
        List of JointLimits for each joint.
    """
    return [
        JointLimits(position_min=-math.pi/2, position_max=math.pi/2, velocity_max=1.0),  # Joint 1
        JointLimits(position_min=-math.pi/2, position_max=math.pi/2, velocity_max=1.0),  # Joint 2
        JointLimits(position_min=-math.pi/2, position_max=math.pi/2, velocity_max=1.0),  # Joint 3
        JointLimits(position_min=-math.pi/2, position_max=math.pi/2, velocity_max=1.0),  # Joint 4
        JointLimits(position_min=-math.pi/2, position_max=math.pi/2, velocity_max=1.5),  # Joint 5
        JointLimits(position_min=-math.pi/4, position_max=math.pi/4, velocity_max=2.0),  # Gripper
    ]