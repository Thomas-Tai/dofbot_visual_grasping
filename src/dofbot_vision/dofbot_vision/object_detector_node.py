"""
ROS2 node for color-based object detection.

This node subscribes to camera images and publishes detected object positions.

Integration:
    - Subscribes: /camera/image_raw (sensor_msgs/Image)
    - Publishes: /vision/detections (custom message array)
    - Publishes: /vision/target_pose (geometry_msgs/PoseStamped)

Usage:
    ros2 run dofbot_vision vision_node --ros-args -p target_color:=green
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import Optional, List
import os

from dofbot_vision.vision_processor import VisionProcessor, DetectionResult


class ObjectDetectorNode(Node):
    """
    ROS2 node that performs color-based object detection.
    
    This node receives camera images, detects colored objects using HSV
    segmentation, and publishes detection results including pixel coordinates
    and visualization images.
    
    Topics:
        Subscribers:
            - /camera/image_raw: Raw camera images
            - /camera/camera_info: Camera calibration info (optional)
        
        Publishers:
            - /vision/target_pose: Target pose for pick-and-place
            - /vision/debug_image: Debug visualization image
    
    Parameters:
        - target_color: Color to detect (default: 'green')
        - min_contour_area: Minimum detection area in pixels
        - publish_debug: Whether to publish debug images
        - detection_rate: Detection loop rate in Hz
    """
    
    def __init__(self):
        super().__init__('object_detector')
        
        # Declare parameters
        self.declare_parameter('target_color', 'green')
        self.declare_parameter('min_contour_area', 500)
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('detection_rate', 10.0)
        self.declare_parameter('hsv_config_path', '')
        
        # Get parameters
        self.target_color = self.get_parameter('target_color').value
        self.min_contour_area = self.get_parameter('min_contour_area').value
        self.publish_debug = self.get_parameter('publish_debug').value
        self.detection_rate = self.get_parameter('detection_rate').value
        hsv_config_path = self.get_parameter('hsv_config_path').value
        
        self.get_logger().info(f"Initializing ObjectDetectorNode")
        self.get_logger().info(f"Target color: {self.target_color}")
        self.get_logger().info(f"Min contour area: {self.min_contour_area}")
        
        # Initialize vision processor
        self.processor = VisionProcessor(min_contour_area=self.min_contour_area)
        
        # Load custom HSV config if provided
        if hsv_config_path and os.path.exists(hsv_config_path):
            self.processor.load_hsv_config(self.target_color, hsv_config_path)
            self.get_logger().info(f"Loaded HSV config from: {hsv_config_path}")
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # QoS profile for camera images (best effort for real-time)
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            image_qos
        )
        
        # Publishers
        self.target_pose_pub = self.create_publisher(
            PoseStamped,
            '/vision/target_pose',
            10
        )
        
        if self.publish_debug:
            self.debug_image_pub = self.create_publisher(
                Image,
                '/vision/debug_image',
                image_qos
            )
        
        # State variables
        self.latest_detection: Optional[DetectionResult] = None
        self.frame_count = 0
        
        # Create timer for detection loop
        self.timer = self.create_timer(1.0 / self.detection_rate, self.timer_callback)
        
        self.get_logger().info("ObjectDetectorNode initialized successfully")
    
    def image_callback(self, msg: Image) -> None:
        """Process incoming camera image.
        
        Args:
            msg: ROS2 Image message
        """
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Perform detection
            detections = self.processor.detect(cv_image, self.target_color)
            
            if detections:
                # Store latest detection (largest by area)
                self.latest_detection = detections[0]
                
                # Update frame for debug publishing
                if self.publish_debug:
                    debug_image = self.processor.draw_detection(
                        cv_image, self.latest_detection
                    )
                    debug_msg = self.bridge.cv2_to_imgmsg(
                        debug_image, encoding='bgr8'
                    )
                    debug_msg.header = msg.header
                    self.debug_image_pub.publish(debug_msg)
            else:
                self.latest_detection = None
            
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
    
    def timer_callback(self) -> None:
        """Timer callback for publishing detection results."""
        if self.latest_detection is not None:
            # Create PoseStamped message for pick-and-place
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'camera_optical_frame'
            
            # Pixel coordinates (z=0 for 2D detection)
            pose_msg.pose.position.x = float(self.latest_detection.centroid[0])
            pose_msg.pose.position.y = float(self.latest_detection.centroid[1])
            pose_msg.pose.position.z = 0.0
            
            # Default orientation (identity)
            pose_msg.pose.orientation.w = 1.0
            
            self.target_pose_pub.publish(pose_msg)
            
            self.get_logger().debug(
                f"Published target at pixel ({self.latest_detection.centroid[0]}, "
                f"{self.latest_detection.centroid[1]}), area: {self.latest_detection.area}"
            )
    
    def set_target_color(self, color_name: str) -> bool:
        """Set the target color for detection.
        
        Args:
            color_name: Name of color configuration to use
            
        Returns:
            True if color was set successfully
        """
        if color_name in self.processor.hsv_configs:
            self.target_color = color_name
            self.get_logger().info(f"Target color changed to: {color_name}")
            return True
        else:
            self.get_logger().warn(f"Unknown color: {color_name}")
            return False


def main(args=None):
    """Main entry point for object detector node."""
    rclpy.init(args=args)
    
    try:
        node = ObjectDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in object detector node: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()