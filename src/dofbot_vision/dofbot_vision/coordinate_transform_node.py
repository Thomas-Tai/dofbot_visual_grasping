"""
ROS2 node for transforming pixel coordinates to robot world coordinates.

This node performs the coordinate transformation from camera pixel space to
robot base_link frame using either:
1. Homography matrix (2D planar transformation)
2. TF2 with known camera-robot transform

Integration:
    - Subscribes: /vision/target_pose (geometry_msgs/PoseStamped) - pixel coords
    - Publishes: /vision/world_pose (geometry_msgs/PoseStamped) - robot coords
    - Uses: TF2 for coordinate frame transforms

Usage:
    ros2 run dofbot_vision transform_node
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from std_msgs.msg import Header
import numpy as np
from typing import Optional, Tuple
import os
import yaml

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose


class CoordinateTransformNode(Node):
    """
    ROS2 node that transforms pixel coordinates to robot world coordinates.
    
    This node receives pixel coordinates from the vision detector and transforms
    them to the robot's base_link frame using either:
    - A pre-computed homography matrix (for planar surfaces)
    - TF2 transforms (for 3D point transformation with known camera pose)
    
    Topics:
        Subscribers:
            - /vision/target_pose: Input pose in pixel coordinates
        
        Publishers:
            - /vision/world_pose: Output pose in robot base_link frame
    
    Parameters:
        - use_homography: Use homography matrix for transformation
        - homography_file: Path to homography matrix YAML file
        - camera_height: Camera height above workspace (for 3D estimation)
        - output_frame: Output coordinate frame (default: 'base_link')
    """
    
    def __init__(self):
        super().__init__('coordinate_transform')
        
        # Declare parameters
        self.declare_parameter('use_homography', True)
        self.declare_parameter('homography_file', '')
        self.declare_parameter('camera_height', 0.3)  # meters
        self.declare_parameter('output_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        
        # Get parameters
        self.use_homography = self.get_parameter('use_homography').value
        homography_file = self.get_parameter('homography_file').value
        self.camera_height = self.get_parameter('camera_height').value
        self.output_frame = self.get_parameter('output_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        
        self.get_logger().info(f"Initializing CoordinateTransformNode")
        self.get_logger().info(f"Use homography: {self.use_homography}")
        self.get_logger().info(f"Output frame: {self.output_frame}")
        
        # Load homography matrix if using
        self.homography: Optional[np.ndarray] = None
        if self.use_homography and homography_file:
            self._load_homography(homography_file)
        
        # Initialize TF2 if not using homography
        if not self.use_homography:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.target_sub = self.create_subscription(
            PoseStamped,
            '/vision/target_pose',
            self.target_callback,
            qos
        )
        
        # Publishers
        self.world_pose_pub = self.create_publisher(
            PoseStamped,
            '/vision/world_pose',
            qos
        )
        
        self.get_logger().info("CoordinateTransformNode initialized")
    
    def _load_homography(self, filepath: str) -> bool:
        """Load homography matrix from YAML file.
        
        Args:
            filepath: Path to YAML file containing homography matrix
            
        Returns:
            True if loaded successfully
        """
        try:
            if not os.path.exists(filepath):
                self.get_logger().warn(f"Homography file not found: {filepath}")
                return False
            
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
            
            if 'homography' in data:
                self.homography = np.array(data['homography'])
                self.get_logger().info(f"Loaded homography matrix from {filepath}")
                return True
            else:
                self.get_logger().error("No 'homography' key found in config file")
                return False
            
        except Exception as e:
            self.get_logger().error(f"Error loading homography: {e}")
            return False
    
    def set_homography(self, matrix: np.ndarray) -> None:
        """Set homography matrix programmatically.
        
        Args:
            matrix: 3x3 homography matrix
        """
        if matrix.shape == (3, 3):
            self.homography = matrix
            self.use_homography = True
            self.get_logger().info("Homography matrix set")
        else:
            self.get_logger().error(
                f"Invalid homography shape: {matrix.shape}, expected (3, 3)"
            )
    
    def transform_pixel_to_world(
        self,
        pixel_x: float,
        pixel_y: float
    ) -> Optional[Tuple[float, float]]:
        """Transform pixel coordinates to world coordinates using homography.
        
        Args:
            pixel_x: X coordinate in pixels
            pixel_y: Y coordinate in pixels
            
        Returns:
            Tuple of (world_x, world_y) in meters, or None if failed
        """
        if self.homography is None:
            return None
        
        # Create homogeneous pixel coordinate
        pixel_point = np.array([pixel_x, pixel_y, 1.0])
        
        # Apply homography transformation
        world_point = self.homography @ pixel_point
        
        # Normalize homogeneous coordinates
        if world_point[2] != 0:
            world_x = world_point[0] / world_point[2]
            world_y = world_point[1] / world_point[2]
            return (world_x, world_y)
        
        return None
    
    def target_callback(self, msg: PoseStamped) -> None:
        """Process incoming target pose (in pixel coordinates).
        
        Args:
            msg: PoseStamped message with pixel coordinates
        """
        try:
            pixel_x = msg.pose.position.x
            pixel_y = msg.pose.position.y
            
            world_pose = PoseStamped()
            world_pose.header.stamp = msg.header.stamp
            world_pose.header.frame_id = self.output_frame
            
            if self.use_homography and self.homography is not None:
                # Use homography transformation
                result = self.transform_pixel_to_world(pixel_x, pixel_y)
                
                if result:
                    world_x, world_y = result
                    world_pose.pose.position.x = world_x
                    world_pose.pose.position.y = world_y
                    world_pose.pose.position.z = 0.0  # Planar assumption
                    
                    # Default orientation for top-down grasp
                    world_pose.pose.orientation.x = 0.0
                    world_pose.pose.orientation.y = 0.0
                    world_pose.pose.orientation.z = 0.0
                    world_pose.pose.orientation.w = 1.0
                    
                    self.world_pose_pub.publish(world_pose)
                    
                    self.get_logger().debug(
                        f"Transformed ({pixel_x:.1f}, {pixel_y:.1f}) -> "
                        f"({world_x:.3f}, {world_y:.3f})"
                    )
                else:
                    self.get_logger().warn("Homography transformation failed")
            
            else:
                # Use TF2 transformation
                # This assumes the input pose is in 3D with proper camera frame
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.output_frame,
                        self.camera_frame,
                        rclpy.time.Time()
                    )
                    
                    # Transform the pose
                    world_pose = do_transform_pose(msg, transform)
                    self.world_pose_pub.publish(world_pose)
                    
                except Exception as e:
                    self.get_logger().warn(f"TF2 transform failed: {e}")
        
        except Exception as e:
            self.get_logger().error(f"Error in target_callback: {e}")


def compute_homography(
    pixel_points: list,
    world_points: list
) -> np.ndarray:
    """Compute homography matrix from corresponding point pairs.
    
    This function calculates the homography matrix that maps pixel
    coordinates to world coordinates. At least 4 point pairs are needed.
    
    Args:
        pixel_points: List of (u, v) pixel coordinates
        world_points: List of (x, y) world coordinates in meters
        
    Returns:
        3x3 homography matrix, or None if computation fails
        
    Example:
        >>> pixel_pts = [(100, 200), (300, 200), (300, 400), (100, 400)]
        >>> world_pts = [(0.1, 0.1), (0.3, 0.1), (0.3, 0.3), (0.1, 0.3)]
        >>> H = compute_homography(pixel_pts, world_pts)
    """
    import cv2
    
    if len(pixel_points) < 4 or len(world_points) < 4:
        print("Error: At least 4 point pairs required for homography")
        return None
    
    if len(pixel_points) != len(world_points):
        print("Error: Number of pixel and world points must match")
        return None
    
    # Convert to numpy arrays
    src_points = np.array(pixel_points, dtype=np.float32)
    dst_points = np.array(world_points, dtype=np.float32)
    
    # Compute homography using RANSAC for robustness
    H, mask = cv2.findHomography(src_points, dst_points, cv2.RANSAC, 5.0)
    
    return H


def save_homography(
    homography: np.ndarray,
    filepath: str,
    description: str = ""
) -> bool:
    """Save homography matrix to YAML file.
    
    Args:
        homography: 3x3 homography matrix
        filepath: Output file path
        description: Optional description string
        
    Returns:
        True if saved successfully
    """
    import os
    from datetime import datetime
    
    try:
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        
        data = {
            'homography': homography.tolist(),
            'description': description,
            'created': datetime.now().isoformat()
        }
        
        with open(filepath, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        
        return True
    except Exception as e:
        print(f"Error saving homography: {e}")
        return False


def main(args=None):
    """Main entry point for coordinate transform node."""
    rclpy.init(args=args)
    
    try:
        node = CoordinateTransformNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in coordinate transform node: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()