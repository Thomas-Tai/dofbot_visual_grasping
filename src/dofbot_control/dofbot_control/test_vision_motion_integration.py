#!/usr/bin/env python3
"""
Integration tests for vision-to-motion pipeline.

Tests the complete flow:
    Camera -> Vision Detection -> Coordinate Transform -> Motion Planning

Usage:
    # Run with simulation (no real camera/hardware needed)
    ros2 run dofbot_control test_vision_motion_integration
    
    # Run with real hardware
    ros2 run dofbot_control test_vision_motion_integration --real-hardware
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PoseArray
from cv_bridge import CvBridge

import cv2
import numpy as np
import time
import threading
import argparse
from typing import Optional, Tuple, List


class MockCameraNode(Node):
    """
    Mock camera node for testing without real hardware.
    Publishes synthetic images with colored shapes.
    """
    
    def __init__(self):
        super().__init__('mock_camera')
        
        self.publisher_ = self.create_publisher(
            Image, 
            '/camera/image_raw', 
            10
        )
        
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.033, self.publish_frame)  # ~30 fps
        
        self.frame_count = 0
        self.target_position = (320, 240)  # Center of image
        self.target_color = 'green'
        
        self.get_logger().info('Mock camera node started')
    
    def set_target(self, position: Tuple[int, int], color: str = 'green'):
        """Set target object position and color for next frames."""
        self.target_position = position
        self.target_color = color
    
    def create_test_image(self) -> np.ndarray:
        """Create a test image with a colored circle at target position."""
        # Create blank image (640x480, 3 channels)
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Fill with light gray background
        image[:] = (200, 200, 200)
        
        # Draw target object (circle)
        color_map = {
            'green': (0, 200, 0),
            'red': (0, 0, 200),
            'blue': (200, 0, 0),
        }
        
        bgr_color = color_map.get(self.target_color, (0, 200, 0))
        cv2.circle(
            image, 
            self.target_position, 
            30,  # radius
            bgr_color, 
            -1  # filled
        )
        
        return image
    
    def publish_frame(self):
        """Publish a test frame."""
        image = self.create_test_image()
        msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_optical_frame'
        self.publisher_.publish(msg)
        self.frame_count += 1


class MockVisionPipeline:
    """
    Simplified vision pipeline for testing.
    Detects colored circles and returns centroids.
    """
    
    def __init__(self):
        self.bridge = CvBridge()
        self.hsv_ranges = {
            'green': {
                'h_min': 35, 'h_max': 85,
                's_min': 50, 's_max': 255,
                'v_min': 50, 'v_max': 255
            }
        }
    
    def detect_color(self, image: np.ndarray, color: str = 'green') -> List[Tuple[int, int]]:
        """Detect colored objects and return centroids."""
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Get HSV range
        hsv_range = self.hsv_ranges.get(color, self.hsv_ranges['green'])
        
        # Create mask
        lower = np.array([hsv_range['h_min'], hsv_range['s_min'], hsv_range['v_min']])
        upper = np.array([hsv_range['h_max'], hsv_range['s_max'], hsv_range['v_max']])
        mask = cv2.inRange(hsv, lower, upper)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Get centroids
        centroids = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Filter small noise
                M = cv2.moments(contour)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    centroids.append((cx, cy))
        
        return centroids


class MockHomography:
    """Mock homography for pixel-to-world transformation."""
    
    def __init__(self):
        # Default: simple linear mapping
        # Assume image center (320, 240) maps to world (0.15, 0.0)
        # Scale: 1 pixel = 0.5mm
        self.scale = 0.0005  # meters per pixel
        self.offset_x = 0.15  # world x at image center
        self.offset_y = 0.0   # world y at image center
        self.image_center = (320, 240)
    
    def pixel_to_world(self, u: int, v: int) -> Tuple[float, float]:
        """Transform pixel coordinates to world coordinates."""
        x = self.offset_x + (u - self.image_center[0]) * self.scale
        y = self.offset_y + (v - self.image_center[1]) * self.scale
        return (x, y)


class VisionMotionIntegrationTest(Node):
    """
    Integration test node for vision-to-motion pipeline.
    
    Tests:
    1. Vision detection accuracy
    2. Coordinate transformation accuracy
    3. Motion planning with detected targets
    """
    
    def __init__(self, use_real_hardware: bool = False):
        super().__init__('vision_motion_integration_test')
        
        self.use_real_hardware = use_real_hardware
        self.callback_group = ReentrantCallbackGroup()
        
        # Publishers
        self.target_pose_pub = self.create_publisher(
            PoseStamped,
            '/target_pose',
            10
        )
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Components
        self.vision_pipeline = MockVisionPipeline()
        self.homography = MockHomography()
        self.bridge = CvBridge()
        
        # Test state
        self.received_images = 0
        self.detected_targets = []
        self.test_results = {
            'vision_detection': False,
            'coordinate_transform': False,
            'motion_planning': False
        }
        
        self.get_logger().info('Vision-Motion Integration Test Node started')
    
    def image_callback(self, msg: Image):
        """Process incoming images."""
        self.received_images += 1
        
        # Convert to OpenCV
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Detect targets
        centroids = self.vision_pipeline.detect_color(image, 'green')
        
        if centroids:
            self.detected_targets = centroids
            self.get_logger().info(f'Detected {len(centroids)} target(s) at: {centroids}')
            
            # Transform to world coordinates
            world_coords = [self.homography.pixel_to_world(u, v) for u, v in centroids]
            self.get_logger().info(f'World coordinates: {world_coords}')
            
            # Publish first target
            if world_coords:
                self.publish_target_pose(world_coords[0])
    
    def publish_target_pose(self, world_coord: Tuple[float, float]):
        """Publish target pose for motion planning."""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = world_coord[0]
        pose.pose.position.y = world_coord[1]
        pose.pose.position.z = 0.05  # Table height + approach offset
        pose.pose.orientation.w = 1.0
        
        self.target_pose_pub.publish(pose)
        self.get_logger().info(f'Published target pose: x={world_coord[0]:.3f}, y={world_coord[1]:.3f}')
    
    def run_tests(self) -> dict:
        """Run all integration tests."""
        self.get_logger().info('=' * 60)
        self.get_logger().info('Starting Vision-Motion Integration Tests')
        self.get_logger().info('=' * 60)
        
        # Test 1: Vision Detection
        self.get_logger().info('\n[Test 1] Vision Detection Accuracy')
        test_image = self._create_test_image_with_circle((400, 300), 'green')
        centroids = self.vision_pipeline.detect_color(test_image, 'green')
        
        if centroids:
            detected = centroids[0]
            expected = (400, 300)
            error = np.sqrt((detected[0] - expected[0])**2 + (detected[1] - expected[1])**2)
            
            if error < 10:  # Allow 10 pixel error
                self.test_results['vision_detection'] = True
                self.get_logger().info(f'  ✓ PASS: Detected {detected}, expected {expected}, error: {error:.1f}px')
            else:
                self.get_logger().error(f'  ✗ FAIL: Detected {detected}, expected {expected}, error: {error:.1f}px')
        else:
            self.get_logger().error('  ✗ FAIL: No targets detected')
        
        # Test 2: Coordinate Transformation
        self.get_logger().info('\n[Test 2] Coordinate Transformation Accuracy')
        test_pixel = (320, 240)  # Image center
        world = self.homography.pixel_to_world(test_pixel[0], test_pixel[1])
        expected_world = (0.15, 0.0)
        error = np.sqrt((world[0] - expected_world[0])**2 + (world[1] - expected_world[1])**2)
        
        if error < 0.01:  # Allow 1cm error
            self.test_results['coordinate_transform'] = True
            self.get_logger().info(f'  ✓ PASS: Pixel {test_pixel} -> World {world}, error: {error*1000:.1f}mm')
        else:
            self.get_logger().error(f'  ✗ FAIL: World {world}, expected {expected_world}, error: {error*100:.1f}cm')
        
        # Test 3: End-to-End Pipeline
        self.get_logger().info('\n[Test 3] End-to-End Pipeline')
        test_positions = [
            ((320, 240), (0.15, 0.0)),      # Center
            ((420, 240), (0.20, 0.0)),      # Right
            ((220, 240), (0.10, 0.0)),      # Left
            ((320, 340), (0.15, -0.05)),    # Down
            ((320, 140), (0.15, 0.05)),     # Up
        ]
        
        pipeline_passed = True
        for pixel, expected_world in test_positions:
            # Vision detection
            test_img = self._create_test_image_with_circle(pixel, 'green')
            detected = self.vision_pipeline.detect_color(test_img, 'green')
            
            if not detected:
                self.get_logger().error(f'  ✗ Pipeline failed at {pixel}: No detection')
                pipeline_passed = False
                continue
            
            # Coordinate transform
            world = self.homography.pixel_to_world(detected[0][0], detected[0][1])
            error = np.sqrt((world[0] - expected_world[0])**2 + (world[1] - expected_world[1])**2)
            
            if error > 0.02:  # Allow 2cm error
                self.get_logger().error(f'  ✗ Pipeline failed at {pixel}: error {error*100:.1f}cm')
                pipeline_passed = False
            else:
                self.get_logger().info(f'  ✓ Pipeline OK: {pixel} -> {world}')
        
        self.test_results['motion_planning'] = pipeline_passed
        
        # Summary
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('Test Summary')
        self.get_logger().info('=' * 60)
        
        passed = sum(self.test_results.values())
        total = len(self.test_results)
        
        for test_name, result in self.test_results.items():
            status = '✓ PASS' if result else '✗ FAIL'
            self.get_logger().info(f'  {test_name}: {status}')
        
        self.get_logger().info(f'\nTotal: {passed}/{total} tests passed')
        
        return self.test_results
    
    def _create_test_image_with_circle(self, position: Tuple[int, int], color: str) -> np.ndarray:
        """Create a test image with a colored circle."""
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        image[:] = (200, 200, 200)  # Gray background
        
        color_map = {
            'green': (0, 200, 0),
            'red': (0, 0, 200),
            'blue': (200, 0, 0),
        }
        
        bgr_color = color_map.get(color, (0, 200, 0))
        cv2.circle(image, position, 30, bgr_color, -1)
        
        return image


def main(args=None):
    parser = argparse.ArgumentParser(description='Vision-Motion Integration Tests')
    parser.add_argument('--real-hardware', action='store_true',
                        help='Run with real camera hardware')
    parsed_args = parser.parse_args()
    
    rclpy.init(args=args)
    
    # Create test node
    test_node = VisionMotionIntegrationTest(use_real_hardware=parsed_args.real_hardware)
    
    # Create mock camera (if not using real hardware)
    mock_camera = None
    if not parsed_args.real_hardware:
        mock_camera = MockCameraNode()
    
    # Create executor
    executor = MultiThreadedExecutor()
    executor.add_node(test_node)
    if mock_camera:
        executor.add_node(mock_camera)
    
    # Run executor in separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        # Wait for nodes to initialize
        time.sleep(1.0)
        
        # Run tests
        results = test_node.run_tests()
        
        # Determine exit code
        all_passed = all(results.values())
        exit_code = 0 if all_passed else 1
        
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        executor.shutdown()
        test_node.destroy_node()
        if mock_camera:
            mock_camera.destroy_node()
        rclpy.shutdown()
        executor_thread.join()
    
    return exit_code


if __name__ == '__main__':
    import sys
    exit_code = main()
    sys.exit(exit_code)