"""
Hand-eye calibration tool for DOFBOT vision system.

This tool provides utilities for:
1. Camera-to-robot calibration (homography-based for planar workspace)
2. Camera intrinsics calibration (checkerboard pattern)
3. Verification of calibration accuracy

Usage:
    ros2 run dofbot_vision calibrate-handeye --mode homography
    ros2 run dofbot_vision calibrate-handeye --mode intrinsics
"""

import cv2
import numpy as np
import argparse
import os
import yaml
from datetime import datetime
from typing import List, Tuple, Optional, Dict, Any


class HomographyCalibrator:
    """
    Tool for calibrating the camera-to-robot coordinate transformation using homography.
    
    This calibration assumes a planar workspace where the robot picks objects from
    a flat surface. The homography maps pixel coordinates to robot XY coordinates.
    
    Procedure:
    1. Place calibration target at known robot positions
    2. Record pixel coordinates of target center
    3. Compute homography from point correspondences
    
    Example:
        >>> calibrator = HomographyCalibrator()
        >>> calibrator.add_point(pixel=(320, 240), world=(0.15, 0.10))
        >>> calibrator.add_point(pixel=(100, 100), world=(0.05, 0.05))
        >>> # ... add at least 4 points
        >>> H = calibrator.compute_homography()
        >>> calibrator.save_calibration('config/homography.yaml')
    """
    
    MIN_POINTS = 4  # Minimum points needed for homography
    
    def __init__(self):
        """Initialize the homography calibrator."""
        self.pixel_points: List[Tuple[float, float]] = []
        self.world_points: List[Tuple[float, float]] = []
        self.homography: Optional[np.ndarray] = None
        self.calibration_data: Dict[str, Any] = {}
    
    def add_point(
        self,
        pixel: Tuple[float, float],
        world: Tuple[float, float]
    ) -> None:
        """Add a calibration point pair.
        
        Args:
            pixel: (u, v) pixel coordinates from camera
            world: (x, y) world coordinates in robot frame (meters)
        """
        self.pixel_points.append(pixel)
        self.world_points.append(world)
        print(f"[INFO] Added point {len(self.pixel_points)}: "
              f"pixel={pixel}, world={world}")
    
    def clear_points(self) -> None:
        """Clear all calibration points."""
        self.pixel_points.clear()
        self.world_points.clear()
        self.homography = None
        print("[INFO] Cleared all calibration points")
    
    def compute_homography(self) -> Optional[np.ndarray]:
        """Compute homography matrix from collected point pairs.
        
        Returns:
            3x3 homography matrix, or None if insufficient points
        """
        if len(self.pixel_points) < self.MIN_POINTS:
            print(f"[ERROR] Need at least {self.MIN_POINTS} points, "
                  f"have {len(self.pixel_points)}")
            return None
        
        # Convert to numpy arrays
        src_points = np.array(self.pixel_points, dtype=np.float32)
        dst_points = np.array(self.world_points, dtype=np.float32)
        
        # Compute homography using RANSAC
        self.homography, mask = cv2.findHomography(src_points, dst_points, cv2.RANSAC, 0.01)
        
        if self.homography is None:
            print("[ERROR] Failed to compute homography")
            return None
        
        # Calculate reprojection error
        error = self._calculate_reprojection_error()
        
        print(f"[INFO] Homography computed successfully")
        print(f"[INFO] Reprojection error: {error:.4f} meters")
        
        return self.homography
    
    def _calculate_reprojection_error(self) -> float:
        """Calculate mean reprojection error in world units.
        
        Returns:
            Mean error in meters
        """
        if self.homography is None:
            return float('inf')
        
        src_points = np.array(self.pixel_points, dtype=np.float32)
        dst_points = np.array(self.world_points, dtype=np.float32)
        
        # Transform pixel points using homography
        transformed = cv2.perspectiveTransform(
            src_points.reshape(-1, 1, 2), self.homography
        ).reshape(-1, 2)
        
        # Calculate error
        errors = np.sqrt(np.sum((transformed - dst_points) ** 2, axis=1))
        return float(np.mean(errors))
    
    def transform_point(self, pixel: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        """Transform a pixel coordinate to world coordinates.
        
        Args:
            pixel: (u, v) pixel coordinates
            
        Returns:
            (x, y) world coordinates in meters, or None if not calibrated
        """
        if self.homography is None:
            return None
        
        point = np.array([[pixel]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(point, self.homography)
        return (float(transformed[0, 0, 0]), float(transformed[0, 0, 1]))
    
    def save_calibration(self, filepath: str, description: str = "") -> bool:
        """Save calibration to YAML file.
        
        Args:
            filepath: Output file path
            description: Optional description of calibration
            
        Returns:
            True if saved successfully
        """
        if self.homography is None:
            print("[ERROR] No calibration to save")
            return False
        
        try:
            os.makedirs(os.path.dirname(filepath), exist_ok=True)
            
            error = self._calculate_reprojection_error()
            
            data = {
                'homography': self.homography.tolist(),
                'description': description or "Camera-to-robot homography calibration",
                'created': datetime.now().isoformat(),
                'num_points': len(self.pixel_points),
                'reprojection_error_m': error,
                'pixel_points': [list(p) for p in self.pixel_points],
                'world_points': [list(p) for p in self.world_points],
            }
            
            with open(filepath, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
            
            print(f"[INFO] Calibration saved to: {filepath}")
            return True
            
        except Exception as e:
            print(f"[ERROR] Failed to save calibration: {e}")
            return False
    
    def load_calibration(self, filepath: str) -> bool:
        """Load calibration from YAML file.
        
        Args:
            filepath: Path to calibration file
            
        Returns:
            True if loaded successfully
        """
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
            
            self.homography = np.array(data['homography'])
            self.pixel_points = [tuple(p) for p in data.get('pixel_points', [])]
            self.world_points = [tuple(p) for p in data.get('world_points', [])]
            self.calibration_data = data
            
            print(f"[INFO] Loaded calibration from: {filepath}")
            print(f"[INFO] Reprojection error: {data.get('reprojection_error_m', 'N/A')} m")
            return True
            
        except Exception as e:
            print(f"[ERROR] Failed to load calibration: {e}")
            return False


class InteractiveHomographyCalibrator:
    """
    Interactive tool for collecting calibration points from camera feed.
    
    This class provides an interactive GUI where the user can:
    1. Click on the camera image to record pixel coordinates
    2. Enter corresponding robot coordinates
    3. Compute and save the homography
    """
    
    def __init__(self, camera_index: int = 0):
        """Initialize interactive calibrator.
        
        Args:
            camera_index: Camera device index
        """
        self.camera_index = camera_index
        self.calibrator = HomographyCalibrator()
        self.cap: Optional[cv2.VideoCapture] = None
        self.current_frame: Optional[np.ndarray] = None
        self.window_name = "Homography Calibration"
        self.running = False
        
        # Robot coordinate input
        self.input_mode = False
        self.input_text = ""
        self.pending_pixel: Optional[Tuple[int, int]] = None
    
    def _mouse_callback(self, event, x, y, flags, param) -> None:
        """Handle mouse events for point selection."""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.pending_pixel = (x, y)
            self.input_mode = True
            self.input_text = ""
            print(f"\n[INPUT] Clicked at pixel ({x}, {y})")
            print("[INPUT] Enter robot world coordinates (x y in meters, e.g., '0.15 0.10'):")
    
    def run(self) -> None:
        """Run the interactive calibration session."""
        # Open camera
        self.cap = cv2.VideoCapture(self.camera_index)
        
        if not self.cap.isOpened():
            print(f"[ERROR] Could not open camera {self.camera_index}")
            return
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Create window
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self._mouse_callback)
        
        print("\n" + "=" * 60)
        print("HOMOGRAPHY CALIBRATION")
        print("=" * 60)
        print("\nInstructions:")
        print("1. Move robot to known position on workspace")
        print("2. Click on the point in the camera image")
        print("3. Enter robot XY coordinates (meters)")
        print("4. Repeat for at least 4 different positions")
        print("5. Press 'c' to compute homography")
        print("6. Press 's' to save calibration")
        print("\nControls:")
        print("  'c' - Compute homography")
        print("  's' - Save calibration")
        print("  'r' - Reset points")
        print("  'q' - Quit")
        print("=" * 60 + "\n")
        
        self.running = True
        
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                break
            
            self.current_frame = frame.copy()
            display = frame.copy()
            
            # Draw existing points
            for i, (px, py) in enumerate(self.calibrator.pixel_points):
                cv2.circle(display, (int(px), int(py)), 5, (0, 255, 0), -1)
                cv2.putText(display, f"P{i+1}", (int(px) + 10, int(py) - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            # Draw pending point
            if self.pending_pixel and self.input_mode:
                cv2.circle(display, self.pending_pixel, 5, (0, 0, 255), -1)
                cv2.putText(display, f"Enter: {self.input_text}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Show status
            status = f"Points: {len(self.calibrator.pixel_points)}/{HomographyCalibrator.MIN_POINTS}"
            cv2.putText(display, status, (10, display.shape[0] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            cv2.imshow(self.window_name, display)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q') or key == 27:
                self.running = False
            
            elif key == ord('c'):
                self.calibrator.compute_homography()
            
            elif key == ord('s'):
                output_path = "config/homography.yaml"
                self.calibrator.save_calibration(output_path)
            
            elif key == ord('r'):
                self.calibrator.clear_points()
        
        self.cap.release()
        cv2.destroyAllWindows()
    
    def process_text_input(self, text: str) -> bool:
        """Process text input for robot coordinates.
        
        Args:
            text: Input string with 'x y' coordinates
            
        Returns:
            True if valid input processed
        """
        try:
            parts = text.strip().split()
            if len(parts) >= 2:
                x = float(parts[0])
                y = float(parts[1])
                
                if self.pending_pixel:
                    self.calibrator.add_point(
                        pixel=self.pending_pixel,
                        world=(x, y)
                    )
                    self.pending_pixel = None
                    self.input_mode = False
                    self.input_text = ""
                    return True
        except ValueError:
            print("[ERROR] Invalid input. Enter two numbers (x y)")
        
        return False


class CameraIntrinsicsCalibrator:
    """
    Tool for camera intrinsic calibration using checkerboard pattern.
    
    This follows OpenCV's standard camera calibration procedure:
    1. Capture multiple images of checkerboard pattern
    2. Detect corner points
    3. Compute camera matrix and distortion coefficients
    """
    
    def __init__(
        self,
        checkerboard_size: Tuple[int, int] = (9, 6),
        square_size: float = 0.025  # meters
    ):
        """Initialize camera intrinsics calibrator.
        
        Args:
            checkerboard_size: Number of inner corners (cols, rows)
            square_size: Size of each square in meters
        """
        self.checkerboard_size = checkerboard_size
        self.square_size = square_size
        
        # Prepare object points
        self.objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)
        self.objp *= square_size
        
        # Storage
        self.objpoints: List[np.ndarray] = []  # 3D points
        self.imgpoints: List[np.ndarray] = []  # 2D points
        
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
    
    def add_image(self, image: np.ndarray) -> bool:
        """Add an image for calibration.
        
        Args:
            image: BGR image containing checkerboard
            
        Returns:
            True if checkerboard detected successfully
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Find checkerboard corners
        ret, corners = cv2.findChessboardCorners(gray, self.checkerboard_size, None)
        
        if ret:
            # Refine corner locations
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            self.objpoints.append(self.objp)
            self.imgpoints.append(corners)
            
            print(f"[INFO] Checkerboard detected. Total images: {len(self.objpoints)}")
            return True
        else:
            print("[WARN] Checkerboard not detected in image")
            return False
    
    def calibrate(self, image_size: Tuple[int, int]) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Perform camera calibration.
        
        Args:
            image_size: (width, height) of images
            
        Returns:
            Tuple of (camera_matrix, distortion_coefficients)
        """
        if len(self.objpoints) < 5:
            print(f"[ERROR] Need at least 5 images, have {len(self.objpoints)}")
            return None, None
        
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            self.objpoints, self.imgpoints, image_size, None, None
        )
        
        if ret:
            self.camera_matrix = mtx
            self.dist_coeffs = dist
            
            # Calculate reprojection error
            mean_error = 0
            for i in range(len(self.objpoints)):
                imgpoints2, _ = cv2.projectPoints(
                    self.objpoints[i], rvecs[i], tvecs[i], mtx, dist
                )
                error = cv2.norm(self.imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
                mean_error += error
            mean_error /= len(self.objpoints)
            
            print(f"[INFO] Calibration successful!")
            print(f"[INFO] Reprojection error: {mean_error:.4f} pixels")
            print(f"\nCamera Matrix:\n{mtx}")
            print(f"\nDistortion Coefficients:\n{dist.ravel()}")
            
            return mtx, dist
        
        return None, None
    
    def save_calibration(self, filepath: str) -> bool:
        """Save camera calibration to YAML file.
        
        Args:
            filepath: Output file path
            
        Returns:
            True if saved successfully
        """
        if self.camera_matrix is None:
            print("[ERROR] No calibration to save")
            return False
        
        try:
            os.makedirs(os.path.dirname(filepath), exist_ok=True)
            
            data = {
                'camera_matrix': self.camera_matrix.tolist(),
                'distortion_coefficients': self.dist_coeffs.tolist(),
                'created': datetime.now().isoformat(),
                'num_images': len(self.objpoints),
                'checkerboard_size': list(self.checkerboard_size),
                'square_size_m': self.square_size,
            }
            
            with open(filepath, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
            
            print(f"[INFO] Camera calibration saved to: {filepath}")
            return True
            
        except Exception as e:
            print(f"[ERROR] Failed to save: {e}")
            return False


def main():
    """Main entry point for calibration tool."""
    parser = argparse.ArgumentParser(
        description='Hand-Eye Calibration Tool for DOFBOT Vision',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Modes:
  homography  - Interactive camera-to-robot calibration
  intrinsics  - Camera intrinsic calibration using checkerboard

Examples:
  ros2 run dofbot_vision calibrate-handeye --mode homography
  ros2 run dofbot_vision calibrate-handeye --mode intrinsics
"""
    )
    
    parser.add_argument(
        '--mode', '-m',
        type=str,
        default='homography',
        choices=['homography', 'intrinsics'],
        help='Calibration mode (default: homography)'
    )
    
    parser.add_argument(
        '--camera', '-c',
        type=int,
        default=0,
        help='Camera device index (default: 0)'
    )
    
    parser.add_argument(
        '--output', '-o',
        type=str,
        default=None,
        help='Output file path'
    )
    
    parser.add_argument(
        '--checkerboard',
        type=str,
        default='9x6',
        help='Checkerboard size as WxH (default: 9x6)'
    )
    
    args = parser.parse_args()
    
    if args.mode == 'homography':
        print("\n[INFO] Starting Homography Calibration Mode")
        calibrator = InteractiveHomographyCalibrator(camera_index=args.camera)
        calibrator.run()
    
    elif args.mode == 'intrinsics':
        print("\n[INFO] Starting Camera Intrinsics Calibration Mode")
        
        # Parse checkerboard size
        cb_size = tuple(map(int, args.checkerboard.split('x')))
        
        calibrator = CameraIntrinsicsCalibrator(checkerboard_size=cb_size)
        
        # Open camera
        cap = cv2.VideoCapture(args.camera)
        if not cap.isOpened():
            print(f"[ERROR] Could not open camera {args.camera}")
            return
        
        print("\nInstructions:")
        print("1. Show checkerboard pattern to camera")
        print("2. Press 'SPACE' to capture image")
        print("3. Capture at least 5 images from different angles")
        print("4. Press 'c' to compute calibration")
        print("5. Press 's' to save")
        print("6. Press 'q' to quit\n")
        
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            display = frame.copy()
            
            # Show preview
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret_cb, corners = cv2.findChessboardCorners(gray, cb_size, None)
            
            if ret_cb:
                cv2.drawChessboardCorners(display, cb_size, corners, ret_cb)
            
            # Status
            status = f"Images: {len(calibrator.objpoints)}"
            cv2.putText(display, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            cv2.imshow('Camera Intrinsics Calibration', display)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q') or key == 27:
                break
            elif key == ord(' '):  # Space to capture
                calibrator.add_image(frame)
            elif key == ord('c'):
                calibrator.calibrate((frame.shape[1], frame.shape[0]))
            elif key == ord('s'):
                output = args.output or 'config/camera_intrinsics.yaml'
                calibrator.save_calibration(output)
        
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()