# Copyright (c) 2024 DOFBOT Project
# SPDX-License-Identifier: BSD-3-Clause

"""
Joint state publisher with velocity estimation for DOFBOT hardware.

This module provides a high-quality joint state publisher that computes
accurate position, velocity, and effort information for MoveIt2 planning.
Since servo motors don't provide velocity/effort feedback directly, velocity
is estimated from position changes with low-pass filtering.

Key Features:
- JointState message construction with all fields
- Low-pass filtered velocity estimation
- Error recovery with exponential backoff
- Timestamp accuracy for simulation compatibility
"""

import logging
import time
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .arm_driver import DofbotDriverInterface
from .exceptions import DofbotCommunicationError

logger = logging.getLogger(__name__)


@dataclass
class VelocityEstimatorConfig:
    """Configuration for velocity estimation.
    
    Attributes:
        alpha: Low-pass filter coefficient (0-1). Higher values = less filtering.
        min_dt: Minimum time delta to avoid division issues.
        max_velocity: Maximum expected velocity for sanity checks (rad/s).
    """
    alpha: float = 0.3
    min_dt: float = 1e-6
    max_velocity: float = 2.0  # rad/s


class VelocityEstimator:
    """Estimate joint velocity from position measurements.
    
    Uses filtered differentiation to compute velocity from position
    measurements. A low-pass filter is applied to reduce noise.
    
    The filter formula is:
        v_filtered = alpha * v_raw + (1 - alpha) * v_prev
    
    Example:
        >>> estimator = VelocityEstimator(alpha=0.3)
        >>> for positions in position_stream:
        ...     velocities = estimator.update(positions, time.time())
    """
    
    def __init__(self, num_joints: int = 6, config: Optional[VelocityEstimatorConfig] = None):
        """Initialize the velocity estimator.
        
        Args:
            num_joints: Number of joints to track.
            config: Velocity estimator configuration.
        """
        self._num_joints = num_joints
        self._config = config or VelocityEstimatorConfig()
        
        # State tracking
        self._prev_positions: Optional[List[float]] = None
        self._prev_velocities: Optional[List[float]] = None
        self._prev_time: Optional[float] = None
        
        logger.debug(f"VelocityEstimator initialized with alpha={self._config.alpha}")
    
    def update(self, positions: List[float], current_time: float) -> List[float]:
        """Update velocity estimate with new position measurement.
        
        Args:
            positions: Current joint positions in radians.
            current_time: Current timestamp in seconds.
        
        Returns:
            Estimated joint velocities in rad/s.
        """
        # First measurement - initialize
        if self._prev_positions is None:
            self._prev_positions = positions.copy()
            self._prev_velocities = [0.0] * self._num_joints
            self._prev_time = current_time
            return [0.0] * self._num_joints
        
        # Calculate time delta
        dt = current_time - self._prev_time
        if dt < self._config.min_dt:
            return self._prev_velocities.copy()
        
        # Calculate raw velocities
        raw_velocities = []
        for i, (curr, prev) in enumerate(zip(positions, self._prev_positions)):
            velocity = (curr - prev) / dt
            # Sanity check - clamp to max velocity
            velocity = max(-self._config.max_velocity, 
                          min(self._config.max_velocity, velocity))
            raw_velocities.append(velocity)
        
        # Apply low-pass filter
        filtered_velocities = []
        for i, (raw, prev_v) in enumerate(zip(raw_velocities, self._prev_velocities)):
            filtered = (self._config.alpha * raw + 
                       (1 - self._config.alpha) * prev_v)
            filtered_velocities.append(filtered)
        
        # Update state
        self._prev_positions = positions.copy()
        self._prev_velocities = filtered_velocities.copy()
        self._prev_time = current_time
        
        return filtered_velocities
    
    def reset(self) -> None:
        """Reset the estimator state."""
        self._prev_positions = None
        self._prev_velocities = None
        self._prev_time = None
    
    def set_initial_velocities(self, velocities: List[float]) -> None:
        """Set initial velocity estimates.
        
        Args:
            velocities: Initial velocity estimates in rad/s.
        """
        if len(velocities) != self._num_joints:
            raise ValueError(f"Expected {self._num_joints} velocities")
        self._prev_velocities = velocities.copy()


class JointStatePublisher:
    """Publishes JointState messages with velocity estimation.
    
    This class reads joint positions from the hardware driver and
    publishes JointState messages with position, velocity, and effort.
    Velocity is estimated using filtered differentiation.
    
    Example:
        >>> publisher = JointStatePublisher(node, driver)
        >>> publisher.update()  # Read hardware and publish
    """
    
    def __init__(
        self,
        node: Node,
        driver: DofbotDriverInterface,
        joint_names: List[str],
        publish_rate: float = 50.0
    ):
        """Initialize the joint state publisher.
        
        Args:
            node: ROS2 node for publishing.
            driver: Hardware driver for reading positions.
            joint_names: List of joint names (must match URDF).
            publish_rate: Publishing rate in Hz.
        """
        self._node = node
        self._driver = driver
        self._joint_names = joint_names
        self._publish_rate = publish_rate
        
        # Create publisher
        self._publisher = node.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Velocity estimator
        self._velocity_estimator = VelocityEstimator(num_joints=len(joint_names))
        
        # State tracking
        self._last_positions: Optional[List[float]] = None
        self._read_failures = 0
        self._max_read_retries = 3
        
        logger.info(f"JointStatePublisher initialized for {len(joint_names)} joints at {publish_rate}Hz")
    
    def update(self) -> Optional[JointState]:
        """Read hardware and publish JointState message.
        
        Returns:
            Published JointState message, or None if read failed.
        """
        positions = self._read_with_retry()
        
        if positions is None:
            logger.warning("Failed to read joint positions after retries")
            return None
        
        # Get current time
        current_time = self._node.get_clock().now()
        
        # Estimate velocities
        velocities = self._velocity_estimator.update(
            positions, 
            current_time.nanoseconds / 1e9
        )
        
        # Create JointState message
        msg = self._create_joint_state_msg(positions, velocities, current_time)
        
        # Publish
        self._publisher.publish(msg)
        self._last_positions = positions
        
        return msg
    
    def _create_joint_state_msg(
        self,
        positions: List[float],
        velocities: List[float],
        current_time
    ) -> JointState:
        """Create a JointState message.
        
        Args:
            positions: Joint positions in radians.
            velocities: Joint velocities in rad/s.
            current_time: ROS2 time for timestamp.
        
        Returns:
            JointState message.
        """
        msg = JointState()
        msg.header.stamp = current_time.to_msg()
        msg.name = self._joint_names.copy()
        msg.position = positions.copy()
        msg.velocity = velocities.copy()
        msg.effort = [0.0] * len(positions)  # Servos don't provide effort feedback
        
        return msg
    
    def _read_with_retry(self) -> Optional[List[float]]:
        """Read joint positions with retry logic.
        
        Returns:
            List of joint positions, or None if all retries fail.
        """
        for attempt in range(self._max_read_retries):
            try:
                positions = self._driver.read_joint_positions()
                if len(positions) == len(self._joint_names):
                    self._read_failures = 0  # Reset failure count on success
                    return positions
            except DofbotCommunicationError as e:
                logger.warning(f"Read failed (attempt {attempt + 1}): {e}")
                if attempt < self._max_read_retries - 1:
                    # Exponential backoff
                    time.sleep(0.01 * (2 ** attempt))
            except Exception as e:
                logger.error(f"Unexpected error reading positions: {e}")
                break
        
        self._read_failures += 1
        return None
    
    def get_last_positions(self) -> Optional[List[float]]:
        """Get the last successfully read positions.
        
        Returns:
            Last positions in radians, or None if never read.
        """
        return self._last_positions
    
    def get_read_failure_count(self) -> int:
        """Get the number of consecutive read failures.
        
        Returns:
            Number of consecutive read failures.
        """
        return self._read_failures
    
    def reset_velocity_estimator(self) -> None:
        """Reset the velocity estimator state."""
        self._velocity_estimator.reset()
    
    def set_velocity_estimator_alpha(self, alpha: float) -> None:
        """Set the velocity estimator filter coefficient.
        
        Args:
            alpha: New filter coefficient (0-1).
        """
        if not 0.0 <= alpha <= 1.0:
            raise ValueError("alpha must be between 0 and 1")
        self._velocity_estimator._config.alpha = alpha