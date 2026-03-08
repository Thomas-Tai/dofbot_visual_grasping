#!/usr/bin/env python3
"""
Test script for Cartesian space control.
Moves the end-effector to specific XYZ coordinates.
"""
import rclpy
import time
from dofbot_control.moveit_interface import MoveItInterface

def main(args=None):
    rclpy.init(args=args)
    
    controller = MoveItInterface()
    controller.wait_for_server()
    
    # First go to home position
    print("Moving to HOME...")
    controller.move_to_named_target('home')
    time.sleep(1.0)
    
    # Move to positions within DOFBOT's small workspace (~20cm reach)
    # Using coordinates closer to the robot base
    print("Moving to Cartesian position (0.1, 0.0, 0.15)...")
    controller.move_to_pose(0.1, 0.0, 0.15)
    time.sleep(1.0)
    
    # Move to another position
    print("Moving to Cartesian position (0.08, 0.08, 0.12)...")
    controller.move_to_pose(0.08, 0.08, 0.12)
    time.sleep(1.0)
    
    # Return home
    print("Returning to HOME...")
    controller.move_to_named_target('home')
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
