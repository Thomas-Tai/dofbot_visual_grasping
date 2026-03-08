"""
Full vision pipeline launch file for DOFBOT visual grasping system.

This launch file starts the complete vision pipeline including:
- Camera driver (v4l2_camera)
- Object detection node
- Coordinate transformation node

This is the main launch file for running the complete visual grasping system.

Topics:
    Published:
        /camera/image_raw (sensor_msgs/Image): Raw camera feed
        /camera/camera_info (sensor_msgs/CameraInfo): Camera calibration
        /vision/target_pose (geometry_msgs/PoseStamped): Pixel coordinates
        /vision/world_pose (geometry_msgs/PoseStamped): World coordinates
        /vision/debug_image (sensor_msgs/Image): Annotated detection image

Usage:
    ros2 launch dofbot_vision vision_pipeline.launch.py
    ros2 launch dofbot_vision vision_pipeline.launch.py target_color:=red
    ros2 launch dofbot_vision vision_pipeline.launch.py camera_device:=/dev/video1
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate the launch description for the complete vision pipeline."""
    
    # Get package share directory
    pkg_share = get_package_share_directory('dofbot_vision')
    
    # ========== Camera Parameters ==========
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='Camera device path'
    )
    
    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='640',
        description='Image width in pixels'
    )
    
    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='480',
        description='Image height in pixels'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30',
        description='Frames per second'
    )
    
    camera_frame_id_arg = DeclareLaunchArgument(
        'camera_frame_id',
        default_value='camera_optical_frame',
        description='Frame ID for camera'
    )
    
    # ========== Vision Parameters ==========
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
        description='Whether to publish debug image'
    )
    
    # ========== Transform Parameters ==========
    use_homography_arg = DeclareLaunchArgument(
        'use_homography',
        default_value='true',
        description='Use homography for coordinate transformation'
    )
    
    workspace_frame_arg = DeclareLaunchArgument(
        'workspace_frame',
        default_value='base_link',
        description='Output coordinate frame'
    )
    
    # Build paths to config files
    hsv_config_path = os.path.join(pkg_share, 'config', 'hsv_green.yaml')
    homography_path = os.path.join(pkg_share, 'config', 'homography.yaml')
    
    # ========== Camera Node ==========
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('camera_device'),
            'image_size': [LaunchConfiguration('image_width'), 
                          LaunchConfiguration('image_height')],
            'time_per_frame': [1, LaunchConfiguration('fps')],
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
            'output_encoding': 'bgr8',
        }],
        remappings=[
            ('/image_raw', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info'),
        ],
    )
    
    # ========== Object Detector Node ==========
    object_detector_node = Node(
        package='dofbot_vision',
        executable='vision_node',
        name='object_detector',
        output='screen',
        parameters=[{
            'target_color': LaunchConfiguration('target_color'),
            'min_contour_area': LaunchConfiguration('min_contour_area'),
            'publish_debug': LaunchConfiguration('publish_debug_image'),
            'detection_rate': 10.0,
            'hsv_config_path': hsv_config_path,
        }],
    )
    
    # ========== Coordinate Transform Node ==========
    coordinate_transform_node = Node(
        package='dofbot_vision',
        executable='transform_node',
        name='coordinate_transform',
        output='screen',
        parameters=[{
            'use_homography': LaunchConfiguration('use_homography'),
            'homography_file': homography_path,
            'output_frame': LaunchConfiguration('workspace_frame'),
            'camera_frame': LaunchConfiguration('camera_frame_id'),
            'camera_height': 0.3,
        }],
    )
    
    return LaunchDescription([
        # Camera arguments
        camera_device_arg,
        image_width_arg,
        image_height_arg,
        fps_arg,
        camera_frame_id_arg,
        
        # Vision arguments
        target_color_arg,
        min_contour_area_arg,
        publish_debug_image_arg,
        
        # Transform arguments
        use_homography_arg,
        workspace_frame_arg,
        
        # Nodes
        camera_node,
        object_detector_node,
        coordinate_transform_node,
    ])