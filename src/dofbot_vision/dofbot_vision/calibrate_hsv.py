"""
Interactive HSV color calibration tool for DOFBOT vision system.

This standalone tool provides a GUI for tuning HSV color thresholds for
object detection. It allows real-time adjustment of threshold values and
saves configurations to YAML files.

Usage:
    ros2 run dofbot_vision calibrate-hsv
    ros2 run dofbot_vision calibrate-hsv --color green
    ros2 run dofbot_vision calibrate-hsv --camera 0 --color red

Keys:
    - 's': Save current configuration
    - 'q': Quit
    - 'r': Reset to defaults
"""

import cv2
import numpy as np
import argparse
import os
import yaml
from datetime import datetime
from typing import Dict, Tuple, Optional


class HSVCalibrator:
    """
    Interactive HSV color calibration tool for robot vision.
    
    This class provides a GUI window with trackbars for adjusting HSV
    thresholds. It displays the original camera feed alongside a masked
    result for immediate feedback.
    
    Attributes:
        camera_index: Index of the camera device
        color_name: Name of the color being calibrated
        window_name: Name of the OpenCV window
        initial_hsv: Initial HSV values to load
        output_path: Path to save configuration
    
    Example:
        >>> calibrator = HSVCalibrator(camera_index=0, color_name='green')
        >>> calibrator.run()
        >>> # Adjust trackbars, press 's' to save, 'q' to quit
    """
    
    # Trackbar constants
    TRACKBAR_NAMES = [
        'H_MIN', 'H_MAX',
        'S_MIN', 'S_MAX',
        'V_MIN', 'V_MAX'
    ]
    
    # Default HSV values for common colors
    DEFAULT_HSV = {
        'green': {'h_min': 35, 'h_max': 85, 's_min': 50, 's_max': 255, 'v_min': 50, 'v_max': 255},
        'red': {'h_min': 0, 'h_max': 10, 's_min': 50, 's_max': 255, 'v_min': 50, 'v_max': 255},
        'red_upper': {'h_min': 170, 'h_max': 180, 's_min': 50, 's_max': 255, 'v_min': 50, 'v_max': 255},
        'blue': {'h_min': 100, 'h_max': 130, 's_min': 50, 's_max': 255, 'v_min': 50, 'v_max': 255},
        'yellow': {'h_min': 20, 'h_max': 35, 's_min': 50, 's_max': 255, 'v_min': 50, 'v_max': 255},
    }
    
    # Maximum values for HSV in OpenCV
    MAX_H = 180
    MAX_S = 255
    MAX_V = 255
    
    def __init__(
        self,
        camera_index: int = 0,
        color_name: str = 'green',
        output_path: Optional[str] = None,
        config_path: Optional[str] = None
    ):
        """Initialize the HSV calibrator.
        
        Args:
            camera_index: Index of the camera device (e.g., 0 for /dev/video0)
            color_name: Name of the color being calibrated
            output_path: Path to save configuration (auto-generated if None)
            config_path: Path to existing config to load
        """
        self.camera_index = camera_index
        self.color_name = color_name
        self.window_name = f"HSV Calibration - {color_name.upper()}"
        
        # Set output path
        if output_path:
            self.output_path = output_path
        else:
            # Default to config directory
            self.output_path = f"config/hsv_{color_name}.yaml"
        
        # Initialize HSV values
        self.hsv_values = self.DEFAULT_HSV.get(color_name, {
            'h_min': 0, 'h_max': 180,
            's_min': 0, 's_max': 255,
            'v_min': 0, 'v_max': 255
        }).copy()
        
        # Load existing config if provided
        if config_path and os.path.exists(config_path):
            self.load_config(config_path)
        
        # Camera and window state
        self.cap: Optional[cv2.VideoCapture] = None
        self.running = False
        self.fps = 0.0
        
        # Morphological kernel for mask cleanup
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    
    def create_trackbars(self) -> None:
        """Create OpenCV trackbars for HSV adjustment."""
        cv2.namedWindow(self.window_name)
        
        # Create trackbars with callbacks
        cv2.createTrackbar('H_MIN', self.window_name, self.hsv_values['h_min'], self.MAX_H, self._on_trackbar)
        cv2.createTrackbar('H_MAX', self.window_name, self.hsv_values['h_max'], self.MAX_H, self._on_trackbar)
        cv2.createTrackbar('S_MIN', self.window_name, self.hsv_values['s_min'], self.MAX_S, self._on_trackbar)
        cv2.createTrackbar('S_MAX', self.window_name, self.hsv_values['s_max'], self.MAX_S, self._on_trackbar)
        cv2.createTrackbar('V_MIN', self.window_name, self.hsv_values['v_min'], self.MAX_V, self._on_trackbar)
        cv2.createTrackbar('V_MAX', self.window_name, self.hsv_values['v_max'], self.MAX_V, self._on_trackbar)
    
    def _on_trackbar(self, val: int) -> None:
        """Callback for trackbar changes."""
        pass  # Values are read directly in get_trackbar_values()
    
    def get_trackbar_values(self) -> Dict[str, int]:
        """Get current trackbar positions.
        
        Returns:
            Dictionary with HSV threshold values
        """
        return {
            'h_min': cv2.getTrackbarPos('H_MIN', self.window_name),
            'h_max': cv2.getTrackbarPos('H_MAX', self.window_name),
            's_min': cv2.getTrackbarPos('S_MIN', self.window_name),
            's_max': cv2.getTrackbarPos('S_MAX', self.window_name),
            'v_min': cv2.getTrackbarPos('V_MIN', self.window_name),
            'v_max': cv2.getTrackbarPos('V_MAX', self.window_name),
        }
    
    def set_trackbar_values(self, values: Dict[str, int]) -> None:
        """Set trackbar positions.
        
        Args:
            values: Dictionary with HSV threshold values
        """
        cv2.setTrackbarPos('H_MIN', self.window_name, values.get('h_min', 0))
        cv2.setTrackbarPos('H_MAX', self.window_name, values.get('h_max', 180))
        cv2.setTrackbarPos('S_MIN', self.window_name, values.get('s_min', 0))
        cv2.setTrackbarPos('S_MAX', self.window_name, values.get('s_max', 255))
        cv2.setTrackbarPos('V_MIN', self.window_name, values.get('v_min', 0))
        cv2.setTrackbarPos('V_MAX', self.window_name, values.get('v_max', 255))
    
    def process_frame(self, frame: np.ndarray, hsv_values: Dict[str, int]) -> Tuple[np.ndarray, np.ndarray]:
        """Apply HSV thresholding to frame.
        
        Args:
            frame: BGR input image
            hsv_values: Dictionary of HSV threshold values
            
        Returns:
            Tuple of (HSV image, binary mask)
        """
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create mask
        lower = np.array([
            hsv_values['h_min'],
            hsv_values['s_min'],
            hsv_values['v_min']
        ], dtype=np.uint8)
        
        upper = np.array([
            hsv_values['h_max'],
            hsv_values['s_max'],
            hsv_values['v_max']
        ], dtype=np.uint8)
        
        mask = cv2.inRange(hsv, lower, upper)
        
        # Morphological operations to clean up
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        
        return hsv, mask
    
    def create_display(
        self,
        original: np.ndarray,
        mask: np.ndarray,
        hsv_values: Dict[str, int]
    ) -> np.ndarray:
        """Create side-by-side display with status overlay.
        
        Args:
            original: Original BGR image
            mask: Binary mask image
            hsv_values: Current HSV threshold values
            
        Returns:
            Combined display image
        """
        # Create colored mask visualization
        mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        mask_colored[mask > 0] = [0, 255, 0]  # Green for detected areas
        
        # Blend original with mask overlay
        overlay = cv2.addWeighted(original, 0.7, mask_colored, 0.3, 0)
        
        # Create side-by-side display
        display = np.hstack([original, overlay])
        
        # Add text overlay with HSV values
        y_offset = 30
        line_height = 25
        
        # Title
        cv2.putText(display, f"Color: {self.color_name.upper()}", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        y_offset += line_height
        cv2.putText(display, f"H: [{hsv_values['h_min']}, {hsv_values['h_max']}]", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        y_offset += line_height
        cv2.putText(display, f"S: [{hsv_values['s_min']}, {hsv_values['s_max']}]", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        y_offset += line_height
        cv2.putText(display, f"V: [{hsv_values['v_min']}, {hsv_values['v_max']}]", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # FPS display
        y_offset += line_height
        cv2.putText(display, f"FPS: {self.fps:.1f}", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        
        # Instructions
        cv2.putText(display, "'s': Save | 'q': Quit | 'r': Reset", 
                   (10, display.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Labels for each side
        label_y = display.shape[0] - 40
        cv2.putText(display, "Original", (original.shape[1] // 2 - 40, label_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(display, "Mask Overlay", (original.shape[1] + original.shape[1] // 2 - 50, label_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        return display
    
    def save_config(self, output_path: Optional[str] = None) -> bool:
        """Save HSV values to YAML file.
        
        Args:
            output_path: Path to save configuration (uses self.output_path if None)
            
        Returns:
            True if saved successfully
        """
        path = output_path or self.output_path
        
        try:
            # Create directory if needed
            os.makedirs(os.path.dirname(path), exist_ok=True)
            
            # Get current values from trackbars
            hsv_values = self.get_trackbar_values()
            
            # Create config data
            data = {
                self.color_name: {
                    'h_min': hsv_values['h_min'],
                    'h_max': hsv_values['h_max'],
                    's_min': hsv_values['s_min'],
                    's_max': hsv_values['s_max'],
                    'v_min': hsv_values['v_min'],
                    'v_max': hsv_values['v_max'],
                    'description': f"HSV thresholds for {self.color_name} color detection",
                    'last_updated': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                }
            }
            
            with open(path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
            
            print(f"[INFO] Configuration saved to: {path}")
            return True
            
        except Exception as e:
            print(f"[ERROR] Failed to save configuration: {e}")
            return False
    
    def load_config(self, config_path: str) -> bool:
        """Load HSV values from YAML file.
        
        Args:
            config_path: Path to YAML configuration file
            
        Returns:
            True if loaded successfully
        """
        try:
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)
            
            if data and self.color_name in data:
                self.hsv_values = data[self.color_name]
                print(f"[INFO] Loaded configuration from: {config_path}")
                return True
            else:
                print(f"[WARN] Color '{self.color_name}' not found in config file")
                return False
                
        except Exception as e:
            print(f"[ERROR] Failed to load configuration: {e}")
            return False
    
    def reset_to_defaults(self) -> None:
        """Reset trackbars to default values for current color."""
        default_values = self.DEFAULT_HSV.get(self.color_name, {
            'h_min': 0, 'h_max': 180,
            's_min': 0, 's_max': 255,
            'v_min': 0, 'v_max': 255
        })
        self.set_trackbar_values(default_values)
        print(f"[INFO] Reset to default values for {self.color_name}")
    
    def run(self) -> None:
        """Run the calibration loop.
        
        Main loop that:
        1. Reads frames from camera
        2. Processes with current HSV values
        3. Displays result
        4. Handles key presses
        """
        # Open camera
        self.cap = cv2.VideoCapture(self.camera_index)
        
        if not self.cap.isOpened():
            print(f"[ERROR] Could not open camera {self.camera_index}")
            print("[INFO] Make sure the camera is connected and not in use by another application")
            return
        
        # Set camera resolution (640x480)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # Create window and trackbars
        self.create_trackbars()
        
        # Set initial values
        self.set_trackbar_values(self.hsv_values)
        
        print(f"\n[INFO] HSV Calibration Tool Started")
        print(f"[INFO] Color: {self.color_name.upper()}")
        print(f"[INFO] Camera: {self.camera_index}")
        print(f"[INFO] Output: {self.output_path}")
        print(f"\n[INFO] Controls:")
        print(f"  's' - Save configuration")
        print(f"  'q' - Quit")
        print(f"  'r' - Reset to defaults")
        print(f"  ESC - Quit")
        
        self.running = True
        frame_times = []
        
        while self.running:
            # Read frame
            ret, frame = self.cap.read()
            if not ret:
                print("[ERROR] Failed to read frame from camera")
                break
            
            # Get current trackbar values
            hsv_values = self.get_trackbar_values()
            
            # Process frame
            _, mask = self.process_frame(frame, hsv_values)
            
            # Create display
            display = self.create_display(frame, mask, hsv_values)
            
            # Resize display if too large
            max_width = 1280
            if display.shape[1] > max_width:
                scale = max_width / display.shape[1]
                display = cv2.resize(display, None, fx=scale, fy=scale)
            
            # Show display
            cv2.imshow(self.window_name, display)
            
            # Calculate FPS
            import time
            frame_times.append(time.time())
            if len(frame_times) > 30:
                frame_times.pop(0)
            if len(frame_times) >= 2:
                self.fps = len(frame_times) / (frame_times[-1] - frame_times[0])
            
            # Handle key press
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q') or key == 27:  # q or ESC
                print("\n[INFO] Quitting...")
                self.running = False
            
            elif key == ord('s'):
                self.save_config()
            
            elif key == ord('r'):
                self.reset_to_defaults()
        
        # Cleanup
        self.cap.release()
        cv2.destroyAllWindows()
        print("[INFO] Calibration tool closed")


def main():
    """Main entry point for HSV calibration tool."""
    parser = argparse.ArgumentParser(
        description='HSV Color Calibration Tool for DOFBOT Vision',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  ros2 run dofbot_vision calibrate-hsv
  ros2 run dofbot_vision calibrate-hsv --color green
  ros2 run dofbot_vision calibrate-hsv --camera 1 --color red
  ros2 run dofbot_vision calibrate-hsv --load config/hsv_green.yaml
"""
    )
    
    parser.add_argument(
        '--camera', '-c',
        type=int,
        default=0,
        help='Camera device index (default: 0)'
    )
    
    parser.add_argument(
        '--color', '-n',
        type=str,
        default='green',
        choices=['green', 'red', 'blue', 'yellow'],
        help='Color name to calibrate (default: green)'
    )
    
    parser.add_argument(
        '--output', '-o',
        type=str,
        default=None,
        help='Output YAML file path (default: config/hsv_<color>.yaml)'
    )
    
    parser.add_argument(
        '--load', '-l',
        type=str,
        default=None,
        help='Load existing configuration file'
    )
    
    args = parser.parse_args()
    
    # Create and run calibrator
    calibrator = HSVCalibrator(
        camera_index=args.camera,
        color_name=args.color,
        output_path=args.output,
        config_path=args.load
    )
    
    calibrator.run()


if __name__ == '__main__':
    main()