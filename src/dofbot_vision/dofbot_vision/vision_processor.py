"""
Vision processor module for DOFBOT color-based object detection.

This module provides the VisionProcessor class for detecting colored objects
in camera images using HSV color space segmentation.

Integration:
    - Input: sensor_msgs/Image from camera
    - Output: Pixel coordinates (u, v) of detected objects
    - Used by: object_detector_node for ROS2 integration
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass
import yaml
import os


@dataclass
class DetectionResult:
    """Result of object detection containing pixel coordinates.
    
    Attributes:
        centroid: (u, v) pixel coordinates of the object center
        area: Contour area in pixels
        color_name: Detected color label (e.g., 'green', 'red', 'blue')
        bbox: Bounding box as (x, y, width, height)
        confidence: Detection confidence (0.0 to 1.0)
    """
    centroid: Tuple[int, int]
    area: int
    color_name: str
    bbox: Tuple[int, int, int, int]
    confidence: float = 1.0


class HSVConfig:
    """HSV color threshold configuration.
    
    Stores and manages HSV threshold values for color detection.
    Values are in OpenCV format: H: 0-180, S: 0-255, V: 0-255.
    """
    
    def __init__(
        self,
        h_min: int = 0,
        h_max: int = 180,
        s_min: int = 0,
        s_max: int = 255,
        v_min: int = 0,
        v_max: int = 255,
        color_name: str = "unknown"
    ):
        """Initialize HSV configuration.
        
        Args:
            h_min: Minimum hue value (0-180)
            h_max: Maximum hue value (0-180)
            s_min: Minimum saturation value (0-255)
            s_max: Maximum saturation value (0-255)
            v_min: Minimum value/brightness (0-255)
            v_max: Maximum value/brightness (0-255)
            color_name: Label for this color configuration
        """
        self.h_min = h_min
        self.h_max = h_max
        self.s_min = s_min
        self.s_max = s_max
        self.v_min = v_min
        self.v_max = v_max
        self.color_name = color_name
    
    @property
    def lower(self) -> np.ndarray:
        """Get lower HSV bound as numpy array."""
        return np.array([self.h_min, self.s_min, self.v_min], dtype=np.uint8)
    
    @property
    def upper(self) -> np.ndarray:
        """Get upper HSV bound as numpy array."""
        return np.array([self.h_max, self.s_max, self.v_max], dtype=np.uint8)
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert configuration to dictionary."""
        return {
            'h_min': self.h_min,
            'h_max': self.h_max,
            's_min': self.s_min,
            's_max': self.s_max,
            'v_min': self.v_min,
            'v_max': self.v_max,
            'color_name': self.color_name
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any], color_name: str = None) -> 'HSVConfig':
        """Create configuration from dictionary.
        
        Args:
            data: Dictionary containing HSV values
            color_name: Optional color name override
            
        Returns:
            HSVConfig instance
        """
        return cls(
            h_min=data.get('h_min', 0),
            h_max=data.get('h_max', 180),
            s_min=data.get('s_min', 0),
            s_max=data.get('s_max', 255),
            v_min=data.get('v_min', 0),
            v_max=data.get('v_max', 255),
            color_name=color_name or data.get('color_name', 'unknown')
        )


class VisionProcessor:
    """
    Core vision processing class for color-based object detection.
    
    This class implements HSV color space segmentation to detect colored
    objects in images. It supports multiple color configurations and provides
    filtering based on contour area.
    
    Example:
        >>> processor = VisionProcessor()
        >>> processor.load_hsv_config('green', 'config/hsv_green.yaml')
        >>> results = processor.detect(frame, 'green')
        >>> for result in results:
        ...     print(f"Found {result.color_name} at {result.centroid}")
    
    Attributes:
        hsv_configs: Dictionary mapping color names to HSVConfig objects
        min_contour_area: Minimum contour area for valid detection
        morph_kernel: Kernel for morphological operations
    """
    
    # Default HSV configurations for common colors
    DEFAULT_CONFIGS = {
        'green': HSVConfig(
            h_min=35, h_max=85,
            s_min=50, s_max=255,
            v_min=50, v_max=255,
            color_name='green'
        ),
        'red': HSVConfig(
            h_min=0, h_max=10,  # Note: Red wraps around in HSV
            s_min=50, s_max=255,
            v_min=50, v_max=255,
            color_name='red'
        ),
        'red_upper': HSVConfig(
            h_min=170, h_max=180,
            s_min=50, s_max=255,
            v_min=50, v_max=255,
            color_name='red'
        ),
        'blue': HSVConfig(
            h_min=100, h_max=130,
            s_min=50, s_max=255,
            v_min=50, v_max=255,
            color_name='blue'
        ),
    }
    
    def __init__(
        self,
        min_contour_area: int = 500,
        morph_kernel_size: Tuple[int, int] = (5, 5)
    ):
        """Initialize the vision processor.
        
        Args:
            min_contour_area: Minimum contour area in pixels for valid detection
            morph_kernel_size: Size of morphological operation kernel
        """
        self.hsv_configs: Dict[str, HSVConfig] = {}
        self.min_contour_area = min_contour_area
        self.morph_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, morph_kernel_size
        )
        
        # Load default configurations
        for color_name, config in self.DEFAULT_CONFIGS.items():
            self.hsv_configs[color_name] = config
    
    def load_hsv_config(self, color_name: str, config_path: str) -> bool:
        """Load HSV configuration from YAML file.
        
        Args:
            color_name: Name to identify this color configuration
            config_path: Path to YAML configuration file
            
        Returns:
            True if loaded successfully, False otherwise
        """
        try:
            if not os.path.exists(config_path):
                return False
            
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)
            
            if data and color_name in data:
                self.hsv_configs[color_name] = HSVConfig.from_dict(
                    data[color_name], color_name
                )
                return True
            return False
        except Exception as e:
            print(f"Error loading HSV config: {e}")
            return False
    
    def save_hsv_config(self, color_name: str, config_path: str) -> bool:
        """Save HSV configuration to YAML file.
        
        Args:
            color_name: Name of the color configuration to save
            config_path: Path to output YAML file
            
        Returns:
            True if saved successfully, False otherwise
        """
        try:
            if color_name not in self.hsv_configs:
                return False
            
            config = self.hsv_configs[color_name]
            data = {color_name: config.to_dict()}
            
            # Create directory if it doesn't exist
            os.makedirs(os.path.dirname(config_path), exist_ok=True)
            
            with open(config_path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
            
            return True
        except Exception as e:
            print(f"Error saving HSV config: {e}")
            return False
    
    def set_hsv_config(self, color_name: str, config: HSVConfig) -> None:
        """Set HSV configuration programmatically.
        
        Args:
            color_name: Name to identify this color configuration
            config: HSVConfig object with threshold values
        """
        self.hsv_configs[color_name] = config
    
    def preprocess_frame(self, frame: np.ndarray) -> np.ndarray:
        """Preprocess frame for detection.
        
        Applies Gaussian blur to reduce noise.
        
        Args:
            frame: BGR input image
            
        Returns:
            Preprocessed image
        """
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        return blurred
    
    def create_mask(
        self,
        hsv_frame: np.ndarray,
        color_name: str
    ) -> np.ndarray:
        """Create binary mask for specified color.
        
        Args:
            hsv_frame: Image in HSV color space
            color_name: Name of color configuration to use
            
        Returns:
            Binary mask where white pixels match the color
        """
        if color_name not in self.hsv_configs:
            return np.zeros(hsv_frame.shape[:2], dtype=np.uint8)
        
        config = self.hsv_configs[color_name]
        mask = cv2.inRange(hsv_frame, config.lower, config.upper)
        
        # Handle red color wrap-around (H: 0-10 and 170-180)
        if color_name == 'red' and 'red_upper' in self.hsv_configs:
            upper_config = self.hsv_configs['red_upper']
            upper_mask = cv2.inRange(hsv_frame, upper_config.lower, upper_config.upper)
            mask = cv2.bitwise_or(mask, upper_mask)
        
        # Apply morphological operations to clean up the mask
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_kernel)
        
        return mask
    
    def find_contours(
        self,
        mask: np.ndarray
    ) -> List[Tuple[np.ndarray, float, Tuple[int, int, int, int]]]:
        """Find contours in binary mask.
        
        Args:
            mask: Binary mask image
            
        Returns:
            List of tuples (contour, area, bounding_box)
        """
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        
        results = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area >= self.min_contour_area:
                x, y, w, h = cv2.boundingRect(contour)
                results.append((contour, area, (x, y, w, h)))
        
        # Sort by area (largest first)
        results.sort(key=lambda x: x[1], reverse=True)
        return results
    
    def detect(
        self,
        frame: np.ndarray,
        color_name: str,
        max_detections: int = 1
    ) -> List[DetectionResult]:
        """Detect colored objects in frame.
        
        Main detection method that processes an input frame and returns
        detected objects of the specified color.
        
        Args:
            frame: BGR input image
            color_name: Name of color to detect
            max_detections: Maximum number of objects to return
            
        Returns:
            List of DetectionResult objects sorted by area (largest first)
        """
        if color_name not in self.hsv_configs:
            return []
        
        # Preprocess
        preprocessed = self.preprocess_frame(frame)
        
        # Convert to HSV
        hsv = cv2.cvtColor(preprocessed, cv2.COLOR_BGR2HSV)
        
        # Create mask
        mask = self.create_mask(hsv, color_name)
        
        # Find contours
        contours = self.find_contours(mask)
        
        # Create detection results
        results = []
        for contour, area, bbox in contours[:max_detections]:
            # Calculate centroid using moments
            M = cv2.moments(contour)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                
                # Calculate confidence based on contour circularity
                perimeter = cv2.arcLength(contour, True)
                if perimeter > 0:
                    circularity = 4 * np.pi * area / (perimeter ** 2)
                    confidence = min(1.0, circularity)
                else:
                    confidence = 0.0
                
                result = DetectionResult(
                    centroid=(cx, cy),
                    area=int(area),
                    color_name=color_name,
                    bbox=bbox,
                    confidence=confidence
                )
                results.append(result)
        
        return results
    
    def detect_multiple(
        self,
        frame: np.ndarray,
        color_names: List[str],
        max_per_color: int = 1
    ) -> Dict[str, List[DetectionResult]]:
        """Detect multiple colors in a single frame.
        
        Args:
            frame: BGR input image
            color_names: List of color names to detect
            max_per_color: Maximum detections per color
            
        Returns:
            Dictionary mapping color names to detection results
        """
        results = {}
        
        # Preprocess once for all colors
        preprocessed = self.preprocess_frame(frame)
        hsv = cv2.cvtColor(preprocessed, cv2.COLOR_BGR2HSV)
        
        for color_name in color_names:
            if color_name in self.hsv_configs:
                mask = self.create_mask(hsv, color_name)
                contours = self.find_contours(mask)
                
                color_results = []
                for contour, area, bbox in contours[:max_per_color]:
                    M = cv2.moments(contour)
                    if M['m00'] > 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                        
                        perimeter = cv2.arcLength(contour, True)
                        if perimeter > 0:
                            circularity = 4 * np.pi * area / (perimeter ** 2)
                            confidence = min(1.0, circularity)
                        else:
                            confidence = 0.0
                        
                        color_results.append(DetectionResult(
                            centroid=(cx, cy),
                            area=int(area),
                            color_name=color_name,
                            bbox=bbox,
                            confidence=confidence
                        ))
                
                results[color_name] = color_results
        
        return results
    
    def draw_detection(
        self,
        frame: np.ndarray,
        detection: DetectionResult,
        color: Tuple[int, int, int] = (0, 255, 0)
    ) -> np.ndarray:
        """Draw detection result on frame.
        
        Args:
            frame: BGR image to draw on
            detection: DetectionResult to visualize
            color: BGR color for drawing
            
        Returns:
            Frame with detection visualization
        """
        output = frame.copy()
        
        # Draw bounding box
        x, y, w, h = detection.bbox
        cv2.rectangle(output, (x, y), (x + w, y + h), color, 2)
        
        # Draw centroid
        cx, cy = detection.centroid
        cv2.circle(output, (cx, cy), 5, color, -1)
        cv2.drawMarker(output, (cx, cy), color, cv2.MARKER_CROSS, 20, 2)
        
        # Draw label
        label = f"{detection.color_name} ({detection.confidence:.2f})"
        cv2.putText(
            output, label, (x, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2
        )
        
        return output
    
    def get_mask_visualization(
        self,
        frame: np.ndarray,
        color_name: str
    ) -> np.ndarray:
        """Create visualization of color mask overlay.
        
        Args:
            frame: BGR input image
            color_name: Color to visualize
            
        Returns:
            BGR image with mask overlay
        """
        preprocessed = self.preprocess_frame(frame)
        hsv = cv2.cvtColor(preprocessed, cv2.COLOR_BGR2HSV)
        mask = self.create_mask(hsv, color_name)
        
        # Create colored mask overlay
        colored_mask = np.zeros_like(frame)
        colored_mask[mask > 0] = (0, 255, 0)  # Green for detected areas
        
        # Blend with original
        result = cv2.addWeighted(frame, 0.7, colored_mask, 0.3, 0)
        
        return result


def main():
    """Test the vision processor with camera input."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Test VisionProcessor')
    parser.add_argument('--camera', type=int, default=0, help='Camera index')
    parser.add_argument('--color', type=str, default='green', help='Color to detect')
    args = parser.parse_args()
    
    # Initialize processor
    processor = VisionProcessor(min_contour_area=500)
    
    # Open camera
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"Error: Could not open camera {args.camera}")
        return
    
    print(f"Detecting {args.color} objects. Press 'q' to quit.")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Detect objects
        results = processor.detect(frame, args.color)
        
        # Draw results
        for result in results:
            frame = processor.draw_detection(frame, result)
            print(f"Detected at: {result.centroid}, Area: {result.area}")
        
        # Show frame
        cv2.imshow('Vision Processor Test', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()