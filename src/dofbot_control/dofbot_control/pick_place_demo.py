#!/usr/bin/env python3
# Copyright (c) 2024 DOFBOT Project
# SPDX-License-Identifier: BSD-3-Clause

"""
End-to-End Pick and Place Demo for DOFBOT.

This module implements a complete pick-and-place demonstration that integrates:
- Vision detection (object detection and pose estimation)
- Motion planning (MoveIt2 trajectory planning)
- Hardware control (real or simulated)

Usage:
    # Simulation mode (default)
    ros2 run dofbot_control pick_place_demo --color green
    
    # Real hardware mode
    ros2 run dofbot_control pick_place_demo --color green --mode hardware
    
    # Custom place location
    ros2 run dofbot_control pick_place_demo --place-x 0.2 --place-y -0.1
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, PoseArray
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint

import time
import math
import threading
import argparse
import logging
from enum import Enum, auto
from typing import Optional, List, Dict, Any
from dataclasses import dataclass


logger = logging.getLogger(__name__)


class PickPlaceState(Enum):
    """States for the pick and place state machine."""
    INIT = auto()
    HOME = auto()
    DETECT = auto()
    PLAN = auto()
    APPROACH = auto()
    GRASP = auto()
    PICK = auto()
    TRANSPORT = auto()
    PLACE = auto()
    RELEASE = auto()
    RETURN = auto()
    SUCCESS = auto()
    FAILED = auto()


@dataclass
class DetectionResult:
    """Result from object detection."""
    x: float  # meters
    y: float  # meters
    z: float  # meters
    color: str
    confidence: float = 1.0
    timestamp: float = 0.0


@dataclass
class DemoConfig:
    """Configuration for the pick and place demo."""
    # Target object
    target_color: str = 'green'
    
    # Place location (relative to base)
    place_x: float = 0.15
    place_y: float = 0.10
    place_z: float = 0.05  # Height for placing
    
    # Approach height above object
    approach_height: float = 0.05  # 5cm above target
    
    # Timing
    grasp_time: float = 1.0
    release_time: float = 0.5
    
    # Retry
    max_retries: int = 3
    
    # Motion parameters
    velocity_scaling: float = 0.3
    acceleration_scaling: float = 0.3


class PickPlaceFSM:
    """
    State machine for pick and place operations.
    
    State Flow:
    INIT -> HOME -> DETECT -> PLAN -> APPROACH -> GRASP -> PICK -> 
    TRANSPORT -> PLACE -> RELEASE -> RETURN -> SUCCESS
    
    Any state can transition to FAILED on error.
    """
    
    STATE_HANDLERS: Dict[PickPlaceState, str] = {
        PickPlaceState.INIT: '_handle_init',
        PickPlaceState.HOME: '_handle_home',
        PickPlaceState.DETECT: '_handle_detect',
        PickPlaceState.PLAN: '_handle_plan',
        PickPlaceState.APPROACH: '_handle_approach',
        PickPlaceState.GRASP: '_handle_grasp',
        PickPlaceState.PICK: '_handle_pick',
        PickPlaceState.TRANSPORT: '_handle_transport',
        PickPlaceState.PLACE: '_handle_place',
        PickPlaceState.RELEASE: '_handle_release',
        PickPlaceState.RETURN: '_handle_return',
        PickPlaceState.FAILED: '_handle_failure',
    }
    
    def __init__(
        self, 
        node: Node,
        motion_interface: Any,
        config: DemoConfig
    ):
        """
        Initialize the state machine.
        
        Args:
            node: ROS2 node for logging and ROS operations.
            motion_interface: Interface for robot motion control.
            config: Demo configuration.
        """
        self.node = node
        self.motion = motion_interface
        self.config = config
        
        # State
        self.state = PickPlaceState.INIT
        self.target_pose: Optional[PoseStamped] = None
        self.approach_pose: Optional[PoseStamped] = None
        self.place_pose: Optional[PoseStamped] = None
        self.failure_count = 0
        
        # Detection subscriber
        self._detection_received = threading.Event()
        self._latest_detection: Optional[DetectionResult] = None
        
        self.node.get_logger().info("PickPlaceFSM initialized")
    
    def setup_detection_subscriber(self):
        """Set up subscriber for vision detections."""
        self._detection_sub = self.node.create_subscription(
            PoseStamped,
            '/detected_object_pose',
            self._detection_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
    
    def _detection_callback(self, msg: PoseStamped):
        """Handle incoming detection."""
        self._latest_detection = DetectionResult(
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            z=msg.pose.position.z,
            color=self.config.target_color,
            timestamp=time.time()
        )
        self._detection_received.set()
    
    def wait_for_detection(self, timeout: float = 5.0) -> Optional[DetectionResult]:
        """Wait for a detection result."""
        self._detection_received.clear()
        if self._detection_received.wait(timeout):
            return self._latest_detection
        return None
    
    def step(self) -> PickPlaceState:
        """
        Execute one state machine step.
        
        Returns:
            The new state after the step.
        """
        handler_name = self.STATE_HANDLERS.get(self.state)
        if handler_name is None:
            self.node.get_logger().error(f"No handler for state: {self.state}")
            return PickPlaceState.FAILED
        
        handler = getattr(self, handler_name, None)
        if handler is None:
            self.node.get_logger().error(f"Handler not found: {handler_name}")
            return PickPlaceState.FAILED
        
        try:
            new_state = handler()
            self.state = new_state
            return new_state
        except Exception as e:
            self.node.get_logger().error(f"Error in {self.state}: {e}")
            self.state = PickPlaceState.FAILED
            return PickPlaceState.FAILED
    
    def run(self) -> bool:
        """
        Run the complete pick and place sequence.
        
        Returns:
            True if successful, False otherwise.
        """
        self.node.get_logger().info("Starting pick and place sequence")
        
        while self.state not in (PickPlaceState.SUCCESS, PickPlaceState.FAILED):
            self.node.get_logger().info(f"State: {self.state.name}")
            self.step()
        
        return self.state == PickPlaceState.SUCCESS
    
    # State handlers
    
    def _handle_init(self) -> PickPlaceState:
        """Initialize the robot."""
        self.node.get_logger().info("Initializing robot...")
        
        if not self.motion.is_connected():
            if not self.motion.connect():
                self.node.get_logger().error("Failed to connect to motion interface")
                return PickPlaceState.FAILED
        
        return PickPlaceState.HOME
    
    def _handle_home(self) -> PickPlaceState:
        """Move to home position."""
        self.node.get_logger().info("Moving to home position...")
        
        if self.motion.move_to_named_pose('home', time_ms=2000):
            return PickPlaceState.DETECT
        else:
            self.node.get_logger().error("Failed to move to home")
            return PickPlaceState.FAILED
    
    def _handle_detect(self) -> PickPlaceState:
        """Detect target object using vision."""
        self.node.get_logger().info(f"Waiting for {self.config.target_color} object detection...")
        
        detection = self.wait_for_detection(timeout=10.0)
        
        if detection is None:
            self.node.get_logger().error("No object detected within timeout")
            return PickPlaceState.FAILED
        
        self.node.get_logger().info(
            f"Detected object at: ({detection.x:.3f}, {detection.y:.3f}, {detection.z:.3f})"
        )
        
        # Create target pose
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = 'base_link'
        self.target_pose.pose.position.x = detection.x
        self.target_pose.pose.position.y = detection.y
        self.target_pose.pose.position.z = detection.z
        self.target_pose.pose.orientation.w = 1.0  # Default orientation
        
        # Create approach pose (above target)
        self.approach_pose = PoseStamped()
        self.approach_pose.header.frame_id = 'base_link'
        self.approach_pose.pose.position.x = detection.x
        self.approach_pose.pose.position.y = detection.y
        self.approach_pose.pose.position.z = detection.z + self.config.approach_height
        self.approach_pose.pose.orientation.w = 1.0
        
        return PickPlaceState.PLAN
    
    def _handle_plan(self) -> PickPlaceState:
        """Plan pick and place trajectories."""
        self.node.get_logger().info("Planning trajectories...")
        
        # Define place pose
        self.place_pose = PoseStamped()
        self.place_pose.header.frame_id = 'base_link'
        self.place_pose.pose.position.x = self.config.place_x
        self.place_pose.pose.position.y = self.config.place_y
        self.place_pose.pose.position.z = self.config.place_z
        self.place_pose.pose.orientation.w = 1.0
        
        # Planning is done on-the-fly during execution for now
        return PickPlaceState.APPROACH
    
    def _handle_approach(self) -> PickPlaceState:
        """Move to approach position above target."""
        self.node.get_logger().info("Moving to approach position...")
        
        if self.approach_pose is None:
            self.node.get_logger().error("No approach pose set")
            return PickPlaceState.FAILED
        
        # Open gripper before approach
        self.motion.set_gripper(closed=False, time_ms=500)
        time.sleep(0.5)
        
        # Move to approach pose
        success = self.motion.move_to_pose(
            self.approach_pose.pose.position.x,
            self.approach_pose.pose.position.y,
            self.approach_pose.pose.position.z
        )
        
        if success:
            return PickPlaceState.GRASP
        else:
            self.node.get_logger().error("Failed to approach target")
            return PickPlaceState.FAILED
    
    def _handle_grasp(self) -> PickPlaceState:
        """Close gripper to grasp object."""
        self.node.get_logger().info("Grasping object...")
        
        # Move down to target
        if self.target_pose is None:
            self.node.get_logger().error("No target pose set")
            return PickPlaceState.FAILED
        
        success = self.motion.move_to_pose(
            self.target_pose.pose.position.x,
            self.target_pose.pose.position.y,
            self.target_pose.pose.position.z
        )
        
        if not success:
            self.node.get_logger().error("Failed to reach target")
            return PickPlaceState.FAILED
        
        time.sleep(0.3)
        
        # Close gripper
        success = self.motion.set_gripper(closed=True, time_ms=500)
        time.sleep(self.config.grasp_time)
        
        if success:
            return PickPlaceState.PICK
        else:
            self.node.get_logger().error("Failed to close gripper")
            return PickPlaceState.FAILED
    
    def _handle_pick(self) -> PickPlaceState:
        """Lift object after grasping."""
        self.node.get_logger().info("Picking up object...")
        
        # Move back up to approach height
        if self.target_pose is None:
            return PickPlaceState.FAILED
        
        success = self.motion.move_to_pose(
            self.target_pose.pose.position.x,
            self.target_pose.pose.position.y,
            self.target_pose.pose.position.z + self.config.approach_height
        )
        
        if success:
            return PickPlaceState.TRANSPORT
        else:
            self.node.get_logger().error("Failed to lift object")
            return PickPlaceState.FAILED
    
    def _handle_transport(self) -> PickPlaceState:
        """Transport object to place location."""
        self.node.get_logger().info("Transporting object to place location...")
        
        # Move to place position (approach height first)
        success = self.motion.move_to_pose(
            self.config.place_x,
            self.config.place_y,
            self.config.place_z + self.config.approach_height
        )
        
        if success:
            return PickPlaceState.PLACE
        else:
            self.node.get_logger().error("Failed to transport object")
            return PickPlaceState.FAILED
    
    def _handle_place(self) -> PickPlaceState:
        """Lower object to place location."""
        self.node.get_logger().info("Placing object...")
        
        success = self.motion.move_to_pose(
            self.config.place_x,
            self.config.place_y,
            self.config.place_z
        )
        
        if success:
            return PickPlaceState.RELEASE
        else:
            self.node.get_logger().error("Failed to place object")
            return PickPlaceState.FAILED
    
    def _handle_release(self) -> PickPlaceState:
        """Release object."""
        self.node.get_logger().info("Releasing object...")
        
        success = self.motion.set_gripper(closed=False, time_ms=500)
        time.sleep(self.config.release_time)
        
        if success:
            return PickPlaceState.RETURN
        else:
            self.node.get_logger().error("Failed to release object")
            return PickPlaceState.FAILED
    
    def _handle_return(self) -> PickPlaceState:
        """Return to home position."""
        self.node.get_logger().info("Returning to home...")
        
        # Lift up first
        self.motion.move_to_pose(
            self.config.place_x,
            self.config.place_y,
            self.config.place_z + self.config.approach_height
        )
        
        # Return home
        if self.motion.move_to_named_pose('home', time_ms=2000):
            return PickPlaceState.SUCCESS
        else:
            self.node.get_logger().warning("Failed to return home, but task complete")
            return PickPlaceState.SUCCESS
    
    def _handle_failure(self) -> PickPlaceState:
        """Handle failure with recovery attempt."""
        self.failure_count += 1
        
        if self.failure_count < self.config.max_retries:
            self.node.get_logger().warning(
                f"Failure {self.failure_count}/{self.config.max_retries}, retrying..."
            )
            # Open gripper if closed
            self.motion.set_gripper(closed=False)
            # Return home for retry
            return PickPlaceState.HOME
        
        self.node.get_logger().error("Max retries exceeded, giving up")
        return PickPlaceState.FAILED


class PickPlaceDemoNode(Node):
    """
    ROS2 node for the pick and place demo.
    
    This node wraps the state machine and provides ROS integration.
    """
    
    def __init__(self):
        super().__init__('pick_place_demo')
        
        # Declare parameters
        self.declare_parameter('target_color', 'green')
        self.declare_parameter('place_x', 0.15)
        self.declare_parameter('place_y', 0.10)
        self.declare_parameter('mode', 'simulation')
        self.declare_parameter('max_retries', 3)
        
        # Get parameters
        target_color = self.get_parameter('target_color').value
        place_x = self.get_parameter('place_x').value
        place_y = self.get_parameter('place_y').value
        mode = self.get_parameter('mode').value
        max_retries = self.get_parameter('max_retries').value
        
        # Create configuration
        self.config = DemoConfig(
            target_color=target_color,
            place_x=place_x,
            place_y=place_y,
            max_retries=max_retries
        )
        
        # Create motion interface
        try:
            from dofbot_control.unified_interface import (
                UnifiedMotionInterface, 
                HardwareMode,
                create_interface
            )
            
            self.motion_interface = create_interface(mode=mode)
            
        except ImportError as e:
            self.get_logger().error(f"Failed to import motion interface: {e}")
            raise
        
        # Create state machine
        self.fsm = PickPlaceFSM(
            node=self,
            motion_interface=self.motion_interface,
            config=self.config
        )
        self.fsm.setup_detection_subscriber()
        
        self.get_logger().info(f"PickPlaceDemoNode initialized with mode: {mode}")
    
    def run_demo(self) -> bool:
        """Run the pick and place demo."""
        return self.fsm.run()


def main(args=None):
    """Main entry point for the pick and place demo."""
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='DOFBOT Pick and Place Demo')
    parser.add_argument(
        '--color', 
        type=str, 
        default='green',
        choices=['green', 'red', 'blue'],
        help='Target object color'
    )
    parser.add_argument(
        '--mode', 
        type=str, 
        default='simulation',
        choices=['simulation', 'hardware', 'hybrid_motion', 'hybrid_vision'],
        help='Hardware mode (simulation, hardware, hybrid_motion, or hybrid_vision)'
    )
    parser.add_argument(
        '--place-x', 
        type=float, 
        default=0.15,
        help='Place position X (meters)'
    )
    parser.add_argument(
        '--place-y', 
        type=float, 
        default=0.10,
        help='Place position Y (meters)'
    )
    parser.add_argument(
        '--retries', 
        type=int, 
        default=3,
        help='Maximum retry attempts'
    )
    
    cli_args = parser.parse_args()
    
    # Initialize ROS with CLI arguments as parameter overrides
    # Format: --ros-args -p param_name:=value
    ros_args = [
        '--ros-args',
        '-p', f'target_color:={cli_args.color}',
        '-p', f'place_x:={cli_args.place_x}',
        '-p', f'place_y:={cli_args.place_y}',
        '-p', f'mode:={cli_args.mode}',
        '-p', f'max_retries:={cli_args.retries}',
    ]
    rclpy.init(args=ros_args)
    
    try:
        # Create node (parameters are already set from CLI)
        node = PickPlaceDemoNode()
        
        # Create executor
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        # Run demo in a thread
        demo_thread = threading.Thread(target=lambda: node.run_demo())
        demo_thread.start()
        
        # Spin
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        
        demo_thread.join(timeout=5.0)
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()