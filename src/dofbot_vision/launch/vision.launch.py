"""
Vision detection launch file for DOFBOT vision system.

This launch file starts the object detection and coordinate transformation
nodes for the DOFBOT visual grasping system.

Topics:
    Subscribed:
        /camera/image_raw (sensor_msgs/Image): Raw camera feed
        /camera/camera_info (sensor_msgs/CameraInfo): Camera calibration
    
    Published:
        /vision/target_pose (geometry_msgs/PoseStamped): Pixel coordinates
        /vision/world_pose (geometry_msgs/PoseStamped): World coordinates
        /vision/detection_image (sensor_msgs/Image): Annotated detection image

Usage:
    ros2 launch dofbot_vision vision.launch.py
    ros2 launch dofbot_vision vision.launch.py target_color:=red
    ros2 launch dofbot_vision vision.launch.py target_color:=blue use_homography:=true
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate the launch description for vision nodes."""
    
    # Get package share directory
    pkg_share = get_package_share_directory('dofbot_vision')
    
    # Declare configurable parameters
    target_color_arg = DeclareLaunchArgument(
        'target_color',
        default_value='green',
        description='Color to detect (green, red, blue)'
    )
    
    min_contour_area_arg = DeclareLaunchArgument(
        'min_contour_area',
        default_value='500',
        description='Minimum contour area for valid detection'
    )
    
    publish_debug_image_arg = DeclareLaunchArgument(
        'publish_debug_image',
        default_value='true',
        description='Whether to publish annotated detection image'
    )
    
    use_homography_arg = DeclareLaunchArgument(
        'use_homography',
        default_value='true',
        description='Use homography for coordinate transformation'
    )
    
    workspace_frame_arg = DeclareLaunchArgument(
        'workspace_frame',
        default_value='base_link',
        description='TF frame for the workspace'
    )
    
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera_optical_frame',
        description='TF frame for the camera'
    )
    
    # Build paths to config files
    hsv_config_path = os.path.join(pkg_share, 'config', 'hsv_green.yaml')
    homography_path = os.path.join(pkg_share, 'config', 'homography.yaml')
    
    # Object Detector Node
    object_detector_node = Node(
        package='dofbot_vision',
        executable='vision_node',
        name='object_detector',
        output='screen',
        parameters=[{
            'target_color': LaunchConfiguration('target_color'),
            'min_contour_area': LaunchConfiguration('min_contour_area'),
            'publish_debug_image': LaunchConfiguration('publish_debug_image'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            # HSV config path can be set based on target_color
            'hsv_config_path': hsv_config_path,
        }],
        remappings=[
            # Ensure correct topic names
            ('/camera/image_raw', '/camera/image_raw'),
        ],
    )
    
    # Coordinate Transform Node
    coordinate_transform_node = Node(
        package='dofbot_vision',
        executable='transform_node',
        name='coordinate_transform',
        output='screen',
        parameters=[{
            'use_homography': LaunchConfiguration('use_homography'),
            'homography_path': homography_path,
            'workspace_frame': LaunchConfiguration('workspace_frame'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'workspace_z': 0.02,  # Height of workspace surface
        }],
        remappings=[
            # Subscribe to vision output
            ('/vision/target_pose', '/vision/target_pose'),
            ('/camera/camera_info', '/camera/camera_info'),
        ],
    )
    
    return LaunchDescription([
        # Launch arguments
        target_color_arg,
        min_contour_area_arg,
        publish_debug_image_arg,
        use_homography_arg,
        workspace_frame_arg,
        camera_frame_arg,
        
        # Nodes
        object_detector_node,
        coordinate_transform_node,
    ])