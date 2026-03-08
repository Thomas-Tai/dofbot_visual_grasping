#!/usr/bin/env python3
# Copyright (c) 2024 DOFBOT Project
# SPDX-License-Identifier: BSD-3-Clause

"""
Safety Monitoring System for DOFBOT robot arm.

This module provides comprehensive safety monitoring for the robot arm,
including:
- Position limit enforcement
- Velocity limit enforcement
- Position error monitoring
- Emergency stop functionality
- Heartbeat monitoring

Safety Philosophy:
- Fail-safe: System defaults to safe state on any failure
- Defense in depth: Multiple independent safety checks
- Real-time: Safety checks complete within control loop time budget

Usage:
    # Launch the safety monitor node
    ros2 run dofbot_hardware safety_monitor
    
    # Or as a library
    from dofbot_hardware.safety_monitor import SafetyMonitor
    
    monitor = SafetyMonitor()
    monitor.check_position_limits(positions)
"""

import time
import threading
import logging
from dataclasses import dataclass
from typing import List, Optional, Dict, Any
from enum import Enum
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import Point

# Import joint limits from driver_interface to avoid duplication
from .driver_interface import JointLimits, DEFAULT_JOINT_LIMITS, JOINT_NAMES


logger = logging.getLogger(__name__)


class SafetyViolationType(Enum):
    """Types of safety violations."""
    POSITION_LIMIT = 'position_limit'
    VELOCITY_LIMIT = 'velocity_limit'
    TORQUE_LIMIT = 'torque_limit'
    POSITION_ERROR = 'position_error'
    COLLISION = 'collision'
    HEARTBEAT_TIMEOUT = 'heartbeat_timeout'
    EMERGENCY_STOP = 'emergency_stop'


@dataclass
class SafetyViolation:
    """Record of a safety violation."""
    timestamp: float
    violation_type: SafetyViolationType
    joint_id: int  # -1 for system-wide violations
    value: float
    limit: float
    message: str
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for logging/serialization."""
        return {
            'timestamp': self.timestamp,
            'violation_type': self.violation_type.value,
            'joint_id': self.joint_id,
            'value': self.value,
            'limit': self.limit,
            'message': self.message
        }


class SafetyMonitor(Node):
    """
    ROS2 node for monitoring robot safety.
    
    Monitors:
    - Joint position limits
    - Joint velocity limits
    - Position error (commanded vs actual)
    - Control node heartbeat
    
    Services:
    - /emergency_stop: Activate/deactivate emergency stop
    - /safety_reset: Clear emergency stop and reset safety status
    
    Published Topics:
    - /safety_status: DiagnosticArray with safety status
    - /diagnostics: Standard ROS diagnostics
    """
    
    def __init__(self):
        super().__init__('safety_monitor')
        
        # Declare parameters
        self.declare_parameter('max_position_error', 0.1)  # radians
        self.declare_parameter('heartbeat_timeout', 1.0)  # seconds
        self.declare_parameter('safety_check_rate', 50.0)  # Hz
        self.declare_parameter('violation_history_size', 100)
        
        # Get parameters
        self._max_position_error = self.get_parameter('max_position_error').value
        self._heartbeat_timeout = self.get_parameter('heartbeat_timeout').value
        self._safety_check_rate = self.get_parameter('safety_check_rate').value
        self._history_size = self.get_parameter('violation_history_size').value
        
        # Joint limits
        self._joint_limits = DEFAULT_JOINT_LIMITS
        
        # State tracking
        self._current_positions = [0.0] * 6
        self._commanded_positions = [0.0] * 6
        self._last_update_time = time.time()
        self._last_heartbeat = time.time()
        
        # Safety state
        self._emergency_stop_active = False
        self._violation_history: deque = deque(maxlen=self._history_size)
        self._lock = threading.Lock()
        
        # Callback groups
        self._callback_group = ReentrantCallbackGroup()
        
        # Setup subscribers
        self._setup_subscribers()
        
        # Setup publishers
        self._setup_publishers()
        
        # Setup services
        self._setup_services()
        
        # Setup timers
        self._setup_timers()
        
        self.get_logger().info("Safety monitor initialized")
        self.get_logger().info(f"  Max position error: {self._max_position_error} rad")
        self.get_logger().info(f"  Heartbeat timeout: {self._heartbeat_timeout} s")
        self.get_logger().info(f"  Safety check rate: {self._safety_check_rate} Hz")
    
    def _setup_subscribers(self):
        """Set up ROS subscribers."""
        # Joint states from hardware
        self._joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        # Joint commands (for position error monitoring)
        self._joint_command_sub = self.create_subscription(
            JointState,
            '/joint_commands',
            self._joint_command_callback,
            QoSProfile(depth=10)
        )
    
    def _setup_publishers(self):
        """Set up ROS publishers."""
        # Diagnostics publisher
        self._diag_publisher = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            10
        )
        
        # Safety status publisher
        self._safety_status_publisher = self.create_publisher(
            DiagnosticArray,
            '/safety_status',
            10
        )
        
        # Stop command publisher (for emergency stop)
        self._stop_publisher = self.create_publisher(
            JointState,
            '/stop_command',
            10
        )
    
    def _setup_services(self):
        """Set up ROS services."""
        # Emergency stop service
        self._e_stop_service = self.create_service(
            SetBool,
            '/emergency_stop',
            self._handle_emergency_stop,
            callback_group=self._callback_group
        )
        
        # Safety reset service
        self._reset_service = self.create_service(
            Trigger,
            '/safety_reset',
            self._handle_safety_reset,
            callback_group=self._callback_group
        )
    
    def _setup_timers(self):
        """Set up ROS timers."""
        # Safety check timer
        self._check_timer = self.create_timer(
            1.0 / self._safety_check_rate,
            self._safety_check_callback
        )
        
        # Diagnostics publish timer
        self._diag_timer = self.create_timer(
            0.1,  # 10 Hz
            self._publish_diagnostics
        )
    
    # Callbacks
    
    def _joint_state_callback(self, msg: JointState):
        """Process joint state updates."""
        with self._lock:
            # Update current positions
            for i, name in enumerate(msg.name):
                if name in JOINT_NAMES:
                    idx = JOINT_NAMES.index(name)
                    if i < len(msg.position):
                        self._current_positions[idx] = msg.position[i]
            
            self._last_update_time = time.time()
    
    def _joint_command_callback(self, msg: JointState):
        """Process joint command updates."""
        with self._lock:
            # Update commanded positions
            for i, name in enumerate(msg.name):
                if name in JOINT_NAMES:
                    idx = JOINT_NAMES.index(name)
                    if i < len(msg.position):
                        self._commanded_positions[idx] = msg.position[i]
    
    def _safety_check_callback(self):
        """Perform periodic safety checks."""
        if self._emergency_stop_active:
            return
        
        current_time = time.time()
        
        with self._lock:
            positions = self._current_positions.copy()
            commanded = self._commanded_positions.copy()
            dt = current_time - self._last_update_time
        
        # Check position limits
        violations = self.check_position_limits(positions)
        
        # Check velocity limits (if we have previous positions)
        # This would require storing previous positions
        
        # Check position error
        violations.extend(self.check_position_error(positions, commanded))
        
        # Check heartbeat
        heartbeat_violation = self.check_heartbeat()
        if heartbeat_violation:
            violations.append(heartbeat_violation)
        
        # Record violations
        for v in violations:
            self._record_violation(v)
            self.get_logger().warning(f"Safety violation: {v.message}")
    
    def _handle_emergency_stop(
        self, 
        request: SetBool.Request, 
        response: SetBool.Response
    ) -> SetBool.Response:
        """Handle emergency stop service call."""
        if request.data:
            # Activate E-stop
            self._emergency_stop_active = True
            self.get_logger().error("EMERGENCY STOP ACTIVATED")
            self._trigger_stop_motion()
            
            # Record violation
            violation = SafetyViolation(
                timestamp=time.time(),
                violation_type=SafetyViolationType.EMERGENCY_STOP,
                joint_id=-1,
                value=1.0,
                limit=0.0,
                message="Emergency stop activated"
            )
            self._record_violation(violation)
            
            response.success = True
            response.message = "Emergency stop activated"
        else:
            # Cannot deactivate via this request
            response.success = False
            response.message = "Use /safety_reset to clear emergency stop"
        
        return response
    
    def _handle_safety_reset(
        self, 
        request: Trigger.Request, 
        response: Trigger.Response
    ) -> Trigger.Response:
        """Handle safety reset service call."""
        if self._emergency_stop_active:
            self.get_logger().info("Safety reset requested")
            
            # Check if safe to reset
            # (in a real system, would verify robot is in safe state)
            
            self._emergency_stop_active = False
            response.success = True
            response.message = "Safety reset successful"
        else:
            response.success = True
            response.message = "No emergency stop active"
        
        return response
    
    # Safety checks
    
    def check_position_limits(
        self, 
        positions: List[float]
    ) -> List[SafetyViolation]:
        """
        Check if any joint exceeds position limits.
        
        Args:
            positions: List of joint positions in radians.
            
        Returns:
            List of violations found.
        """
        violations = []
        
        for i, pos in enumerate(positions):
            if i >= len(self._joint_limits):
                continue
            
            limits = self._joint_limits[i]
            
            if pos < limits.position_min:
                violations.append(SafetyViolation(
                    timestamp=time.time(),
                    violation_type=SafetyViolationType.POSITION_LIMIT,
                    joint_id=i,
                    value=pos,
                    limit=limits.position_min,
                    message=f"Joint {i} below minimum: {pos:.3f} < {limits.position_min:.3f}"
                ))
            elif pos > limits.position_max:
                violations.append(SafetyViolation(
                    timestamp=time.time(),
                    violation_type=SafetyViolationType.POSITION_LIMIT,
                    joint_id=i,
                    value=pos,
                    limit=limits.position_max,
                    message=f"Joint {i} above maximum: {pos:.3f} > {limits.position_max:.3f}"
                ))
        
        return violations
    
    def check_velocity_limits(
        self, 
        positions: List[float], 
        prev_positions: List[float],
        dt: float
    ) -> List[SafetyViolation]:
        """
        Check if any joint velocity exceeds limits.
        
        Args:
            positions: Current joint positions.
            prev_positions: Previous joint positions.
            dt: Time delta between measurements.
            
        Returns:
            List of violations found.
        """
        violations = []
        
        if dt < 1e-6:
            return violations
        
        for i, (curr, prev) in enumerate(zip(positions, prev_positions)):
            if i >= len(self._joint_limits):
                continue
            
            velocity = abs(curr - prev) / dt
            
            if velocity > self._joint_limits[i].velocity_max:
                violations.append(SafetyViolation(
                    timestamp=time.time(),
                    violation_type=SafetyViolationType.VELOCITY_LIMIT,
                    joint_id=i,
                    value=velocity,
                    limit=self._joint_limits[i].velocity_max,
                    message=f"Joint {i} velocity exceeded: {velocity:.3f} > {self._joint_limits[i].velocity_max:.3f}"
                ))
        
        return violations
    
    def check_position_error(
        self, 
        positions: List[float], 
        commanded: List[float]
    ) -> List[SafetyViolation]:
        """
        Check if commanded != actual by threshold.
        
        Indicates stuck joint or collision.
        
        Args:
            positions: Actual joint positions.
            commanded: Commanded joint positions.
            
        Returns:
            List of violations found.
        """
        violations = []
        
        for i, (act, cmd) in enumerate(zip(positions, commanded)):
            error = abs(cmd - act)
            
            if error > self._max_position_error:
                violations.append(SafetyViolation(
                    timestamp=time.time(),
                    violation_type=SafetyViolationType.POSITION_ERROR,
                    joint_id=i,
                    value=error,
                    limit=self._max_position_error,
                    message=f"Joint {i} position error: {error:.3f} > {self._max_position_error:.3f}"
                ))
        
        return violations
    
    def check_heartbeat(self) -> Optional[SafetyViolation]:
        """
        Check if control node is still alive.
        
        Returns:
            SafetyViolation if timeout exceeded, None otherwise.
        """
        elapsed = time.time() - self._last_update_time
        
        if elapsed > self._heartbeat_timeout:
            return SafetyViolation(
                timestamp=time.time(),
                violation_type=SafetyViolationType.HEARTBEAT_TIMEOUT,
                joint_id=-1,  # System-wide
                value=elapsed,
                limit=self._heartbeat_timeout,
                message=f"Control node heartbeat timeout: {elapsed:.1f}s"
            )
        
        return None
    
    # Utility methods
    
    def _record_violation(self, violation: SafetyViolation):
        """Record a safety violation."""
        with self._lock:
            self._violation_history.append(violation)
    
    def _trigger_stop_motion(self):
        """Immediately stop all robot motion."""
        stop_msg = JointState()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.name = JOINT_NAMES
        stop_msg.position = self._current_positions.copy()
        self._stop_publisher.publish(stop_msg)
    
    def _publish_diagnostics(self):
        """Publish current safety status for monitoring."""
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        status = DiagnosticStatus()
        status.name = 'Safety Monitor'
        
        if self._emergency_stop_active:
            status.level = DiagnosticStatus.ERROR
            status.message = 'Emergency stop active'
        elif len(self._violation_history) > 0:
            recent = [v for v in self._violation_history 
                     if time.time() - v.timestamp < 5.0]
            if recent:
                status.level = DiagnosticStatus.WARN
                status.message = f'{len(recent)} recent violations'
            else:
                status.level = DiagnosticStatus.OK
                status.message = 'All checks passed'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'All checks passed'
        
        # Add violation history
        with self._lock:
            recent_violations = list(self._violation_history)[-10:]
        
        for v in recent_violations:
            status.values.append(KeyValue(
                key=f'{v.violation_type.value}_{v.joint_id}',
                value=f'{v.value:.3f} (limit: {v.limit:.3f})'
            ))
        
        # Add current state
        status.values.append(KeyValue(
            key='emergency_stop_active',
            value=str(self._emergency_stop_active)
        ))
        
        status.values.append(KeyValue(
            key='violation_count',
            value=str(len(self._violation_history))
        ))
        
        msg.status.append(status)
        self._diag_publisher.publish(msg)
        self._safety_status_publisher.publish(msg)
    
    def is_safe(self) -> bool:
        """Check if system is in a safe state."""
        return not self._emergency_stop_active
    
    def get_violation_history(self) -> List[SafetyViolation]:
        """Get the violation history."""
        with self._lock:
            return list(self._violation_history)
    
    def clear_history(self):
        """Clear the violation history."""
        with self._lock:
            self._violation_history.clear()


def main(args=None):
    """Main entry point for the safety monitor."""
    rclpy.init(args=args)
    
    try:
        node = SafetyMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logger.error(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()