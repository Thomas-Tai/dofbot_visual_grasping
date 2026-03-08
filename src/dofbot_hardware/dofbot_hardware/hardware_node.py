# Copyright (c) 2024 DOFBOT Project
# SPDX-License-Identifier: BSD-3-Clause

"""
ROS2 Lifecycle Hardware Node for DOFBOT robot arm.

This module implements the hardware interface node that bridges MoveIt2's
trajectory planning with the physical servo controllers. It follows the
ROS2 lifecycle node pattern for proper state management.

Key Features:
    - Lifecycle-managed hardware connection
    - Joint state publishing at configurable rate
    - Trajectory command subscription
    - Support for both real and mock hardware
    - Diagnostics publishing for monitoring

Architecture:
    - Publishers: /joint_states, /diagnostics
    - Subscribers: /joint_trajectory, /joint_commands
    - Parameters: use_mock, publish_rate, joint_names, etc.
"""

import logging
import threading
import time
from typing import List, Optional

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy_lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import Publisher as LifecyclePublisher
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from .arm_driver import DofbotDriver
from .mock_driver import MockDofbotDriver, MockConfig
from .exceptions import DofbotError, DofbotConnectionError, DofbotCommunicationError

# Configure module logger
logger = logging.getLogger(__name__)


class DofbotHardwareNode(LifecycleNode):
    """ROS2 Lifecycle hardware interface node for DOFBOT.
    
    This node manages the connection to the DOFBOT hardware and provides:
        - Joint state publishing at configurable rate (default 50Hz)
        - Trajectory command subscription for MoveIt2 integration
        - Lifecycle management for safe startup/shutdown
        - Diagnostics for monitoring hardware health
    
    Lifecycle States:
        - Unconfigured: Node created, resources not allocated
        - Inactive: Resources allocated, hardware not active
        - Active: Hardware active, publishing enabled
        - Finalized: Node destroyed
    
    Parameters:
        - use_mock (bool): Use mock driver for simulation (default: False)
        - publish_rate (double): Joint state publish rate in Hz (default: 50.0)
        - joint_names (string_array): List of joint names
        - velocity_scaling (double): Scale factor for velocities (default: 0.5)
        - max_velocity (double): Maximum joint velocity in rad/s (default: 1.0)
        - communication_timeout (double): Timeout for hardware reads (default: 0.1)
    
    Example:
        >>> # Launch from command line:
        >>> ros2 run dofbot_hardware hardware_node --ros-args -p use_mock:=true
    """
    
    # Default joint names (must match URDF)
    DEFAULT_JOINT_NAMES = [
        'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper'
    ]
    
    def __init__(self, node_name: str = 'dofbot_hardware') -> None:
        """Initialize the hardware node.
        
        Args:
            node_name: Name for the ROS2 node.
        """
        super().__init__(node_name)
        
        # Declare parameters
        self.declare_parameter('use_mock', False)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('joint_names', self.DEFAULT_JOINT_NAMES)
        self.declare_parameter('velocity_scaling', 0.5)
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('communication_timeout', 0.1)
        
        # Initialize member variables
        self._driver = None
        self._joint_names: List[str] = []
        self._publish_rate: float = 50.0
        self._read_timer = None
        self._diagnostics_timer = None
        
        # Thread safety
        self._lock = threading.Lock()
        
        # Current state tracking
        self._current_positions: List[float] = [0.0] * 6
        self._commanded_positions: List[float] = [0.0] * 6
        
        # Publishers (lifecycle-managed)
        self._joint_state_pub: Optional[LifecyclePublisher] = None
        self._diagnostics_pub: Optional[LifecyclePublisher] = None
        
        # Subscribers
        self._trajectory_sub = None
        self._joint_command_sub = None
        
        # Diagnostics tracking
        self._read_errors = 0
        self._write_errors = 0
        self._last_read_time = 0.0
        
        self.get_logger().info("DofbotHardwareNode created")
    
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the hardware node.
        
        Creates driver, publishers, and subscribers but does not activate hardware.
        
        Args:
            state: Current lifecycle state.
            
        Returns:
            SUCCESS if configuration successful, FAILURE otherwise.
        """
        self.get_logger().info("Configuring hardware node...")
        
        try:
            # Get parameters
            use_mock = self.get_parameter('use_mock').value
            self._publish_rate = self.get_parameter('publish_rate').value
            self._joint_names = list(self.get_parameter('joint_names').value)
            
            # Create driver
            if use_mock:
                self.get_logger().info("Using mock driver for simulation")
                config = MockConfig(
                    max_velocity=self.get_parameter('max_velocity').value * 180.0 / 3.14159
                )
                self._driver = MockDofbotDriver(config)
            else:
                self.get_logger().info("Using real hardware driver")
                self._driver = DofbotDriver()
            
            # Create lifecycle publishers
            self._joint_state_pub = self.create_lifecycle_publisher(
                JointState,
                '/joint_states',
                10
            )
            
            self._diagnostics_pub = self.create_lifecycle_publisher(
                DiagnosticArray,
                '/diagnostics',
                10
            )
            
            # Create subscribers
            callback_group = ReentrantCallbackGroup()
            
            self._trajectory_sub = self.create_subscription(
                JointTrajectory,
                '/joint_trajectory',
                self._trajectory_callback,
                10,
                callback_group=callback_group
            )
            
            self._joint_command_sub = self.create_subscription(
                JointState,
                '/joint_commands',
                self._joint_command_callback,
                10,
                callback_group=callback_group
            )
            
            self.get_logger().info("Hardware node configured successfully")
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Failed to configure hardware node: {e}")
            return TransitionCallbackReturn.FAILURE
    
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate the hardware node.
        
        Connects to hardware and starts publishing.
        
        Args:
            state: Current lifecycle state.
            
        Returns:
            SUCCESS if activation successful, FAILURE otherwise.
        """
        self.get_logger().info("Activating hardware node...")
        
        try:
            # Connect to hardware
            if not self._driver.connect():
                self.get_logger().error("Failed to connect to hardware")
                return TransitionCallbackReturn.FAILURE
            
            # Read initial positions
            self._current_positions = self._driver.read_joint_positions()
            self._commanded_positions = self._current_positions.copy()
            
            # Start read timer
            period = 1.0 / self._publish_rate
            self._read_timer = self.create_timer(
                period,
                self._read_hardware_callback
            )
            
            # Start diagnostics timer (1 Hz)
            self._diagnostics_timer = self.create_timer(
                1.0,
                self._publish_diagnostics
            )
            
            # Call parent to activate publishers
            super().on_activate(state)
            
            self.get_logger().info("Hardware node activated successfully")
            return TransitionCallbackReturn.SUCCESS
            
        except DofbotConnectionError as e:
            self.get_logger().error(f"Hardware connection failed: {e}")
            return TransitionCallbackReturn.FAILURE
        except Exception as e:
            self.get_logger().error(f"Failed to activate hardware node: {e}")
            return TransitionCallbackReturn.FAILURE
    
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate the hardware node.
        
        Stops publishing and disconnects from hardware.
        
        Args:
            state: Current lifecycle state.
            
        Returns:
            SUCCESS if deactivation successful.
        """
        self.get_logger().info("Deactivating hardware node...")
        
        # Stop timers
        if self._read_timer:
            self._read_timer.cancel()
            self.destroy_timer(self._read_timer)
            self._read_timer = None
        
        if self._diagnostics_timer:
            self._diagnostics_timer.cancel()
            self.destroy_timer(self._diagnostics_timer)
            self._diagnostics_timer = None
        
        # Disconnect from hardware
        if self._driver:
            self._driver.disconnect()
        
        # Call parent to deactivate publishers
        super().on_deactivate(state)
        
        self.get_logger().info("Hardware node deactivated")
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Clean up the hardware node.
        
        Releases all resources.
        
        Args:
            state: Current lifecycle state.
            
        Returns:
            SUCCESS if cleanup successful.
        """
        self.get_logger().info("Cleaning up hardware node...")
        
        # Destroy subscribers
        if self._trajectory_sub:
            self.destroy_subscription(self._trajectory_sub)
            self._trajectory_sub = None
        
        if self._joint_command_sub:
            self.destroy_subscription(self._joint_command_sub)
            self._joint_command_sub = None
        
        # Destroy publishers
        if self._joint_state_pub:
            self.destroy_publisher(self._joint_state_pub)
            self._joint_state_pub = None
        
        if self._diagnostics_pub:
            self.destroy_publisher(self._diagnostics_pub)
            self._diagnostics_pub = None
        
        # Release driver
        self._driver = None
        
        self.get_logger().info("Hardware node cleaned up")
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Shutdown the hardware node.
        
        Final cleanup on node shutdown.
        
        Args:
            state: Current lifecycle state.
            
        Returns:
            SUCCESS if shutdown successful.
        """
        self.get_logger().info("Shutting down hardware node...")
        return TransitionCallbackReturn.SUCCESS
    
    def _read_hardware_callback(self) -> None:
        """Timer callback to read hardware and publish joint states.
        
        This runs at the configured publish_rate (default 50Hz).
        """
        try:
            with self._lock:
                if not self._driver or not self._driver.is_connected():
                    return
                
                # Read positions from hardware
                positions = self._driver.read_joint_positions()
                self._current_positions = positions
                self._last_read_time = time.time()
            
            # Publish joint state
            self._publish_joint_state(positions)
            
        except DofbotCommunicationError as e:
            self._read_errors += 1
            self.get_logger().warning(f"Hardware read error: {e}")
        except Exception as e:
            self._read_errors += 1
            self.get_logger().error(f"Unexpected error in read callback: {e}")
    
    def _publish_joint_state(self, positions: List[float]) -> None:
        """Publish joint state message.
        
        Args:
            positions: List of joint positions in radians.
        """
        if self._joint_state_pub is None or not self._joint_state_pub.is_activated():
            return
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names
        msg.position = positions
        msg.velocity = [0.0] * len(positions)  # Velocity estimation in separate module
        msg.effort = [0.0] * len(positions)  # No effort feedback from servos
        
        self._joint_state_pub.publish(msg)
    
    def _trajectory_callback(self, msg: JointTrajectory) -> None:
        """Handle incoming trajectory command.
        
        Args:
            msg: JointTrajectory message from MoveIt2.
        """
        self.get_logger().info(f"Received trajectory with {len(msg.points)} points")
        
        # Validate trajectory
        if not self._validate_trajectory(msg):
            self.get_logger().error("Invalid trajectory received")
            return
        
        try:
            with self._lock:
                if not self._driver or not self._driver.is_connected():
                    self.get_logger().error("Hardware not connected")
                    return
                
                # Execute trajectory points
                for i, point in enumerate(msg.points):
                    # Apply velocity scaling
                    time_ms = self._calculate_execution_time(point)
                    
                    # Write positions
                    success = self._driver.write_joint_positions(
                        list(point.positions),
                        time_ms
                    )
                    
                    if not success:
                        self._write_errors += 1
                        self.get_logger().error(f"Failed to execute trajectory point {i}")
                        return
                    
                    self._commanded_positions = list(point.positions)
                    
                    # Wait for execution (simplified - proper implementation
                    # would track position and provide feedback)
                    time.sleep(time_ms / 1000.0)
                
                self.get_logger().info("Trajectory execution completed")
                
        except DofbotError as e:
            self._write_errors += 1
            self.get_logger().error(f"Trajectory execution error: {e}")
        except Exception as e:
            self._write_errors += 1
            self.get_logger().error(f"Unexpected error executing trajectory: {e}")
    
    def _joint_command_callback(self, msg: JointState) -> None:
        """Handle incoming joint command.
        
        Args:
            msg: JointState message with target positions.
        """
        try:
            with self._lock:
                if not self._driver or not self._driver.is_connected():
                    self.get_logger().error("Hardware not connected")
                    return
                
                # Write positions with default execution time
                self._driver.write_joint_positions(msg.position, 500)
                self._commanded_positions = list(msg.position)
                
        except DofbotError as e:
            self._write_errors += 1
            self.get_logger().error(f"Joint command error: {e}")
    
    def _validate_trajectory(self, trajectory: JointTrajectory) -> bool:
        """Validate trajectory is safe to execute.
        
        Args:
            trajectory: Trajectory to validate.
            
        Returns:
            True if trajectory is valid, False otherwise.
        """
        # Check joint names match
        if set(trajectory.joint_names) != set(self._joint_names):
            self.get_logger().error(
                f"Joint names mismatch: got {trajectory.joint_names}, "
                f"expected {self._joint_names}"
            )
            return False
        
        # Check we have points
        if not trajectory.points:
            self.get_logger().error("Empty trajectory")
            return False
        
        # Check position counts
        for i, point in enumerate(trajectory.points):
            if len(point.positions) != len(self._joint_names):
                self.get_logger().error(
                    f"Point {i} has {len(point.positions)} positions, "
                    f"expected {len(self._joint_names)}"
                )
                return False
        
        return True
    
    def _calculate_execution_time(self, point) -> int:
        """Calculate execution time for a trajectory point.
        
        Args:
            point: JointTrajectoryPoint.
            
        Returns:
            Execution time in milliseconds.
        """
        # Use time_from_start if specified
        if point.time_from_start.sec > 0 or point.time_from_start.nanosec > 0:
            time_ms = point.time_from_start.sec * 1000 + point.time_from_start.nanosec // 1000000
            # Apply velocity scaling
            time_ms = int(time_ms / self.get_parameter('velocity_scaling').value)
            return max(100, time_ms)  # Minimum 100ms
        
        # Default execution time
        return 500
    
    def _publish_diagnostics(self) -> None:
        """Publish diagnostic information."""
        if self._diagnostics_pub is None or not self._diagnostics_pub.is_activated():
            return
        
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        status = DiagnosticStatus()
        status.name = 'DOFBOT Hardware'
        status.hardware_id = 'dofbot_001'
        
        # Determine status level
        if self._driver and self._driver.is_connected():
            if self._read_errors > 10:
                status.level = DiagnosticStatus.WARN
                status.message = 'Hardware communication issues'
            else:
                status.level = DiagnosticStatus.OK
                status.message = 'Hardware operating normally'
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = 'Hardware not connected'
        
        # Add diagnostic values
        status.values.append(KeyValue(
            key='publish_rate',
            value=str(self._publish_rate)
        ))
        status.values.append(KeyValue(
            key='read_errors',
            value=str(self._read_errors)
        ))
        status.values.append(KeyValue(
            key='write_errors',
            value=str(self._write_errors)
        ))
        status.values.append(KeyValue(
            key='last_read_time',
            value=f'{self._last_read_time:.3f}'
        ))
        
        # Add current positions
        for i, (name, pos) in enumerate(zip(self._joint_names, self._current_positions)):
            status.values.append(KeyValue(
                key=f'{name}_position',
                value=f'{pos:.4f}'
            ))
        
        msg.status.append(status)
        self._diagnostics_pub.publish(msg)
    
    def get_current_positions(self) -> List[float]:
        """Get current joint positions.
        
        Returns:
            List of joint positions in radians.
        """
        with self._lock:
            return self._current_positions.copy()
    
    def get_commanded_positions(self) -> List[float]:
        """Get commanded joint positions.
        
        Returns:
            List of commanded positions in radians.
        """
        with self._lock:
            return self._commanded_positions.copy()


def main(args=None):
    """Main entry point for the hardware node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    
    rclpy.init(args=args)
    
    executor = MultiThreadedExecutor()
    node = DofbotHardwareNode()
    
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()