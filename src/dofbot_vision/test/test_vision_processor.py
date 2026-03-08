"""
Unit tests for the VisionProcessor class.

Tests the core vision processing functionality including:
- HSV color thresholding
- Contour detection
- Centroid computation
- Detection result generation
"""

import pytest
import numpy as np
import cv2
from typing import List

# Import the module under test
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dofbot_vision.vision_processor import (
    VisionProcessor, 
    DetectionResult,
    load_hsv_config
)


class TestVisionProcessor:
    """Test suite for VisionProcessor class."""
    
    @pytest.fixture
    def processor(self) -> VisionProcessor:
        """Create a default VisionProcessor instance for testing."""
        return VisionProcessor(color_name='green', min_contour_area=100)
    
    @pytest.fixture
    def green_test_image(self) -> np.ndarray:
        """Create a test image with a green rectangle."""
        # Create black background
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Draw a green rectangle in the center
        cv2.rectangle(image, (250, 190), (390, 290), (0, 255, 0), -1)
        
        return image
    
    @pytest.fixture
    def multi_color_image(self) -> np.ndarray:
        """Create a test image with multiple colored shapes."""
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Green circle
        cv2.circle(image, (160, 240), 50, (0, 255, 0), -1)
        
        # Red rectangle
        cv2.rectangle(image, (290, 190), (350, 290), (0, 0, 255), -1)
        
        # Blue circle
        cv2.circle(image, (480, 240), 50, (255, 0, 0), -1)
        
        return image
    
    def test_initialization_default(self, processor: VisionProcessor):
        """Test default initialization."""
        assert processor.color_name == 'green'
        assert processor.min_contour_area == 100
        assert processor.hsv_config is not None
        assert 'h_min' in processor.hsv_config
    
    def test_initialization_with_color(self):
        """Test initialization with different colors."""
        for color in ['green', 'red', 'blue']:
            proc = VisionProcessor(color_name=color)
            assert proc.color_name == color
    
    def test_initialization_with_config(self):
        """Test initialization with custom config path."""
        # Use non-existent path, should use defaults
        proc = VisionProcessor(config_path='/nonexistent/path.yaml')
        assert proc.hsv_config is not None
    
    def test_set_hsv_range(self, processor: VisionProcessor):
        """Test setting HSV range directly."""
        processor.set_hsv_range(30, 90, 40, 255, 40, 255)
        
        assert processor.hsv_config['h_min'] == 30
        assert processor.hsv_config['h_max'] == 90
        assert processor.hsv_config['s_min'] == 40
        assert processor.hsv_config['s_max'] == 255
        assert processor.hsv_config['v_min'] == 40
        assert processor.hsv_config['v_max'] == 255
    
    def test_create_mask(self, processor: VisionProcessor, green_test_image: np.ndarray):
        """Test mask creation."""
        hsv_image = cv2.cvtColor(green_test_image, cv2.COLOR_BGR2HSV)
        mask = processor.create_mask(hsv_image)
        
        assert mask.shape == green_test_image.shape[:2]
        assert mask.dtype == np.uint8
        
        # Check that mask has white pixels (green detected)
        assert np.any(mask > 0)
    
    def test_find_contours(self, processor: VisionProcessor, green_test_image: np.ndarray):
        """Test contour finding."""
        hsv_image = cv2.cvtColor(green_test_image, cv2.COLOR_BGR2HSV)
        mask = processor.create_mask(hsv_image)
        contours = processor.find_contours(mask)
        
        assert len(contours) > 0
        # Contours should be sorted by area
        if len(contours) > 1:
            assert cv2.contourArea(contours[0]) >= cv2.contourArea(contours[1])
    
    def test_find_contours_min_area(self, green_test_image: np.ndarray):
        """Test that small contours are filtered out."""
        processor = VisionProcessor(min_contour_area=10000)  # Large minimum
        hsv_image = cv2.cvtColor(green_test_image, cv2.COLOR_BGR2HSV)
        mask = processor.create_mask(hsv_image)
        contours = processor.find_contours(mask)
        
        # No contours should pass the large area threshold
        assert len(contours) == 0
    
    def test_compute_centroid(self, processor: VisionProcessor, green_test_image: np.ndarray):
        """Test centroid computation."""
        hsv_image = cv2.cvtColor(green_test_image, cv2.COLOR_BGR2HSV)
        mask = processor.create_mask(hsv_image)
        contours = processor.find_contours(mask)
        
        if contours:
            centroid = processor.compute_centroid(contours[0])
            
            assert isinstance(centroid, tuple)
            assert len(centroid) == 2
            assert 0 <= centroid[0] < 640  # x within image bounds
            assert 0 <= centroid[1] < 480  # y within image bounds
    
    def test_detect_single_object(self, processor: VisionProcessor, green_test_image: np.ndarray):
        """Test detection of a single green object."""
        results = processor.detect(green_test_image)
        
        assert len(results) == 1
        assert isinstance(results[0], DetectionResult)
        assert results[0].color_name == 'green'
        assert results[0].area > 0
        
        # Centroid should be near center of green rectangle (320, 240)
        cx, cy = results[0].centroid
        assert 300 < cx < 340  # Near x center
        assert 220 < cy < 260  # Near y center
    
    def test_detect_no_objects(self, processor: VisionProcessor):
        """Test detection when no matching objects exist."""
        # All-black image
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        results = processor.detect(image)
        
        assert len(results) == 0
    
    def test_detect_multiple_objects(self, multi_color_image: np.ndarray):
        """Test detection with multiple objects of same color."""
        processor = VisionProcessor(color_name='green', min_contour_area=100)
        results = processor.detect(multi_color_image, max_detections=5)
        
        # Should only detect green objects
        for result in results:
            assert result.color_name == 'green'
    
    def test_detect_max_detections(self, processor: VisionProcessor):
        """Test that max_detections limits the number of results."""
        # Create image with multiple green shapes
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        for i in range(5):
            cv2.circle(image, (100 + i * 100, 240), 40, (0, 255, 0), -1)
        
        results = processor.detect(image, max_detections=2)
        
        assert len(results) <= 2
    
    def test_detect_with_debug(self, processor: VisionProcessor, green_test_image: np.ndarray):
        """Test detection with debug visualization."""
        results, debug_image = processor.detect_with_debug(green_test_image)
        
        assert len(results) >= 0
        assert debug_image.shape == green_test_image.shape
    
    def test_detection_result_dataclass(self):
        """Test DetectionResult dataclass."""
        result = DetectionResult(
            centroid=(320, 240),
            area=1000,
            color_name='test',
            bounding_box=(270, 190, 100, 100),
            confidence=0.95
        )
        
        assert result.centroid == (320, 240)
        assert result.area == 1000
        assert result.color_name == 'test'
        assert result.bounding_box == (270, 190, 100, 100)
        assert result.confidence == 0.95
    
    def test_compute_confidence(self, processor: VisionProcessor, green_test_image: np.ndarray):
        """Test confidence computation."""
        hsv_image = cv2.cvtColor(green_test_image, cv2.COLOR_BGR2HSV)
        mask = processor.create_mask(hsv_image)
        contours = processor.find_contours(mask)
        
        if contours:
            confidence = processor.compute_confidence(contours[0], mask)
            
            assert 0.0 <= confidence <= 1.0
    
    def test_get_mask_for_visualization(self, processor: VisionProcessor, green_test_image: np.ndarray):
        """Test mask retrieval for visualization."""
        mask = processor.get_mask_for_visualization(green_test_image)
        
        assert mask.shape == green_test_image.shape[:2]
        assert mask.dtype == np.uint8


class TestHSVConfig:
    """Test HSV configuration loading."""
    
    def test_default_hsv_values(self):
        """Test that default HSV values are valid."""
        for color in ['green', 'red', 'blue', 'yellow']:
            proc = VisionProcessor(color_name=color)
            
            assert 0 <= proc.hsv_config['h_min'] <= 180
            assert 0 <= proc.hsv_config['h_max'] <= 180
            assert proc.hsv_config['h_min'] <= proc.hsv_config['h_max']
            
            assert 0 <= proc.hsv_config['s_min'] <= 255
            assert 0 <= proc.hsv_config['s_max'] <= 255
            
            assert 0 <= proc.hsv_config['v_min'] <= 255
            assert 0 <= proc.hsv_config['v_max'] <= 255


if __name__ == '__main__':
    pytest.main([__file__, '-v'])