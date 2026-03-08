"""
DOFBOT Vision Package

This package provides vision perception capabilities for the DOFBOT 5-DOF robot arm,
enabling visual grasping through color-based object detection.

Modules:
    - vision_processor: Core HSV color segmentation and contour detection
    - object_detector_node: ROS2 node for publishing detection results
    - coordinate_transform_node: Pixel-to-world coordinate transformation
    - calibrate_hsv: Interactive HSV color calibration tool
    - calibration_tool: Hand-eye calibration utility

Topics:
    Published:
        - /vision/target_pose (geometry_msgs/PoseStamped): Target object pose
        - /vision/world_pose (geometry_msgs/PoseStamped): World coordinates
        - /vision/detection_image (sensor_msgs/Image): Annotated detection image
    
    Subscribed:
        - /camera/image_raw (sensor_msgs/Image): Raw camera feed
        - /camera/camera_info (sensor_msgs/CameraInfo): Camera calibration

Parameters:
    - hsv_config_path: Path to HSV threshold configuration file
    - min_contour_area: Minimum contour area for valid detection
    - publish_debug_image: Whether to publish annotated debug images

Usage:
    # Launch full vision pipeline
    ros2 launch dofbot_vision vision_pipeline.launch.py
    
    # Run HSV calibration tool
    ros2 run dofbot_vision calibrate-hsv --color green
"""

__version__ = '0.1.0'
__author__ = 'Thomas Tai'
__email__ = 'thomastai.uni@gmail.com'

# Import main classes for convenient access
from .vision_processor import VisionProcessor, DetectionResult

__all__ = ['VisionProcessor', 'DetectionResult']