#!/usr/bin/env python3
import rclpy
import time
from dofbot_control.moveit_interface import MoveItInterface

def main(args=None):
    rclpy.init(args=args)
    
    controller = MoveItInterface()
    controller.wait_for_server()
    
    print("Moving to HOME...")
    controller.move_to_named_target('home')
    time.sleep(1.0)
    
    print("Moving to READY...")
    controller.move_to_named_target('ready')
    time.sleep(1.0)

    print("Moving to DOWN...")
    controller.move_to_named_target('down')
    time.sleep(1.0)
    
    print("Back to HOME...")
    controller.move_to_named_target('home')
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
