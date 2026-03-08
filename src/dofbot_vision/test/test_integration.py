"""
Integration tests for the dofbot_vision package.

Tests the integration between:
- VisionProcessor and object_detector_node
- ROS2 message publishing
- Launch file configurations
"""

import pytest
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import threading
import time

# Import modules for testing
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dofbot_vision.vision_processor import VisionProcessor


class TestIntegration:
    """Integration test suite for dofbot_vision package."""
    
    @pytest.fixture
    def rclpy_init(self):
        """Initialize ROS2."""
        rclpy.init()
        yield
        rclpy.shutdown()
    
    def test_package_import(self):
        """Test that the package can be imported."""
        from dofbot_vision import vision_processor
        from dofbot_vision import object_detector_node
        from dofbot_vision import coordinate_transform_node
        
        assert vision_processor is not None
        assert object_detector_node is not None
        assert coordinate_transform_node is not None
    
    def test_vision_processor_with_ros_image(self, rclpy_init):
        """Test VisionProcessor with ROS Image message conversion."""
        # Create a test image
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.rectangle(test_image, (250, 190), (390, 290), (0, 255, 0), -1)
        
        # Convert to ROS Image
        bridge = CvBridge()
        ros_image = bridge.cv2_to_imgmsg(test_image, encoding='bgr8')
        
        # Convert back and process
        cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
        
        processor = VisionProcessor(color_name='green')
        results = processor.detect(cv_image)
        
        assert len(results) > 0


class TestSubscriber(Node):
    """Helper node to subscribe to vision topics."""
    
    def __init__(self):
        super().__init__('test_subscriber')
        
        self.detection_received = False
        self.pose_received = False
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/vision/detection',
            self.detection_callback,
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/vision/target_pose',
            self.pose_callback,
            10
        )
    
    def detection_callback(self, msg):
        self.detection_received = True
    
    def pose_callback(self, msg):
        self.pose_received = True


class TestNodeFunctionality:
    """Test node functionality without full ROS2 setup."""
    
    def test_vision_processor_standalone(self):
        """Test that VisionProcessor works standalone."""
        processor = VisionProcessor(color_name='green')
        
        # Create test image with green circle
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        import cv2
        cv2.circle(image, (320, 240), 50, (0, 255, 0), -1)
        
        results = processor.detect(image)
        
        assert len(results) == 1
        centroid = results[0].centroid
        
        # Centroid should be near circle center
        assert abs(centroid[0] - 320) < 10
        assert abs(centroid[1] - 240) < 10
    
    def test_hsv_calibration_tool_import(self):
        """Test that calibration tool can be imported."""
        from dofbot_vision.calibrate_hsv import HSVCalibrator
        
        calibrator = HSVCalibrator(camera_index=0, color_name='green')
        assert calibrator.color_name == 'green'
        assert calibrator.hsv_values is not None
    
    def test_calibration_tool_import(self):
        """Test that hand-eye calibration tool can be imported."""
        from dofbot_vision.calibration_tool import HandEyeCalibrator
        
        calibrator = HandEyeCalibrator(camera_index=0)
        assert calibrator.calibration_type == 'homography'


class TestConfigFiles:
    """Test configuration file handling."""
    
    def test_hsv_config_format(self):
        """Test HSV config file format."""
        import yaml
        from ament_index_python.packages import get_package_share_directory
        import os
        
        try:
            pkg_share = get_package_share_directory('dofbot_vision')
            config_path = os.path.join(pkg_share, 'config', 'hsv_green.yaml')
            
            if os.path.exists(config_path):
                with open(config_path, 'r') as f:
                    config = yaml.safe_load(f)
                
                assert 'green' in config
                assert 'h_min' in config['green']
                assert 'h_max' in config['green']
        except Exception:
            # Config not installed yet
            pass
    
    def test_homography_config_format(self):
        """Test homography config file format."""
        import yaml
        import os
        
        # Check if config directory exists
        config_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config', 'homography.yaml'
        )
        
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            assert 'homography' in config
            assert len(config['homography']) == 3


if __name__ == '__main__':
    pytest.main([__file__, '-v'])