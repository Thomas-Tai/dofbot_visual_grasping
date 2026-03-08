#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, MotionPlanRequest
from sensor_msgs.msg import JointState
import math
import os
import numpy as np
import ikpy.chain
from ament_index_python.packages import get_package_share_directory

class MoveItInterface(Node):
    def __init__(self):
        super().__init__('moveit_interface')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        
        # Hardcoded poses for convenience (since we don't have moveit_commander to read SRDF)
        self.named_poses = {
            'home': [0.0, 0.0, 0.0, 0.0, 0.0],
            'ready': [0.0, 0.72, 0.74, 0.0, 0.0], # Approximate values from SRDF
            'down':  [0.0, 1.57, 0.0, 0.0, 0.0],
        }

        # Load IKPY Chain
        # Note: Active links mask will be inferred from URDF (revolute=True, fixed=False)
        urdf_path = os.path.join(get_package_share_directory('dofbot_description'), 'urdf', 'dofbot.urdf')
        self.chain = ikpy.chain.Chain.from_urdf_file(urdf_path)
        self.get_logger().info(f"IKPY Chain loaded with {len(self.chain.links)} links")

    def wait_for_server(self):
        self.get_logger().info('Waiting for MoveGroup action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('MoveGroup action server available!')

    def move_to_joint_state(self, joint_values):
        """Move the robot to a specified list of joint angles (radians)."""
        goal_msg = MoveGroup.Goal()
        
        # Create request
        request = MotionPlanRequest()
        request.workspace_parameters.header.frame_id = 'world'
        request.workspace_parameters.min_corner.x = -1.0
        request.workspace_parameters.min_corner.y = -1.0
        request.workspace_parameters.min_corner.z = -1.0
        request.workspace_parameters.max_corner.x = 1.0
        request.workspace_parameters.max_corner.y = 1.0
        request.workspace_parameters.max_corner.z = 1.0
        
        request.start_state.is_diff = True
        request.group_name = 'dofbot_arm'
        request.max_velocity_scaling_factor = 0.5
        request.max_acceleration_scaling_factor = 0.5
        
        # Create joint constraints
        constraints = Constraints()
        for i, name in enumerate(self.joint_names):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(joint_values[i])
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
            
        request.goal_constraints.append(constraints)
        goal_msg.request = request
        goal_msg.planning_options.plan_only = False  # Set to True if you only want to plan without execution
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 3
        
        # Send goal
        self.get_logger().info(f'Sending goal: {joint_values}')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, self._send_goal_future)
        
        goal_handle = self._send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            return False
            
        self.get_logger().info('Goal accepted, executing...')
        self._get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self._get_result_future)
        
        result = self._get_result_future.result().result
        if result.error_code.val == 1: # SUCCESS
            self.get_logger().info('Motion succeeded!')
            return True
        else:
            self.get_logger().error(f'Motion failed with error code: {result.error_code.val}')
            return False

    def move_to_named_target(self, name):
        if name not in self.named_poses:
            self.get_logger().error(f'Unknown pose name: {name}')
            return False
        return self.move_to_joint_state(self.named_poses[name])

    def move_to_pose(self, x, y, z):
        """
        Move the end-effector to a specific Cartesian position using IKPY for IK solving.
        
        Args:
            x, y, z: Target position in meters (relative to world frame)
        
        Returns:
            bool: True if motion succeeded, False otherwise
        """
        self.get_logger().info(f'Computing IK for target: x={x:.3f}, y={y:.3f}, z={z:.3f}')
        
        try:
            # IKPY inverse_kinematics returns a list of angles for ALL links (including fixed base)
            # We typically default orientation_mode to None for 5-DOF to avoid over-constraining
            target_position = [x, y, z]
            
            # Using basic Newton-Raphson
            ik_solution = self.chain.inverse_kinematics(target_position)
            
            # Helper to extract active joints only
            # Check the chain structure: usually index 0 is base (fixed). 
            # Our 5 joints are likely indices 1, 2, 3, 4, 5. 
            # But ikpy solution length matches chain.links.
            # We need to map `ik_solution` to `self.joint_names`.
            
            # Simple heuristic: filter out the first (base) if it's 0.0 and fixed? 
            # Better: Use the chain.links to check 'joint_type'.
            
            active_joints = []
            for i, link in enumerate(self.chain.links):
                if link.name != 'Base link' and link.name != 'base_link': # Skip base
                     # Wait, ikpy Chain object handles this cleaner? 
                     # Usually we just take indices 1 to 6?
                     pass
            
            # For DOFBOT URDF:
            # Link 0: base_link (Fixed)
            # Link 1: link1 (Revolute)
            # ...
            # Link 5: link5 (Revolute)
            # Link 6: tool0 (Fixed)
            
            # Assuming 5 DOFs, we need 5 values.
            # ik_solution likely has 7 elements (if 7 links/joints defined).
            
            # DEBUG: Log the full solution
            # self.get_logger().info(f'Full IK Solution: {ik_solution}')
            
            # Extract the 5 revolute joints. 
            # Based on URDF, they should be indices 1, 2, 3, 4, 5.
            if len(ik_solution) >= 6:
                joint_values = ik_solution[1:6] 
            else:
                 self.get_logger().error(f'IK Solution too short: {len(ik_solution)}')
                 return False

            self.get_logger().info(f'Computed Joint Values: {joint_values}')
            
            # Check if valid (not NaN)
            if np.isnan(joint_values).any():
                self.get_logger().error('IK Solution contains NaNs')
                return False

            # Execute
            return self.move_to_joint_state(joint_values)
            
        except Exception as e:
            self.get_logger().error(f'IK Computation Failed: {e}')
            return False
