"""
Camera launch file for DOFBOT vision system.

This launch file starts the USB camera driver with proper configuration
for the DOFBOT visual grasping system.

Camera Hardware: Logitech C920 USB webcam
Required Resolution: 640x480 @ 30fps

Topics Published:
    /camera/image_raw (sensor_msgs/Image): Raw camera feed
    /camera/camera_info (sensor_msgs/CameraInfo): Camera calibration info

Usage:
    ros2 launch dofbot_vision camera.launch.py
    ros2 launch dofbot_vision camera.launch.py camera_device:=/dev/video1
    ros2 launch dofbot_vision camera.launch.py image_width:=1280 image_height:=720
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate the launch description for the camera driver."""
    
    # Declare configurable parameters
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='Camera device path (e.g., /dev/video0)'
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
        description='Frame ID for camera image header'
    )
    
    camera_info_url_arg = DeclareLaunchArgument(
        'camera_info_url',
        default_value='',
        description='URL to camera calibration file (package://... format)'
    )
    
    # Get package share directory for potential config files
    pkg_share = get_package_share_directory('dofbot_vision')
    
    # Camera node using v4l2_camera
    # This is the preferred driver for USB cameras in ROS2
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        output='screen',
        parameters=[{
            # Video device configuration
            'video_device': LaunchConfiguration('camera_device'),
            
            # Image format settings
            'image_size': [LaunchConfiguration('image_width'), 
                          LaunchConfiguration('image_height')],
            
            # Frame rate (time_per_frame as [numerator, denominator])
            # For 30 fps: [1, 30] means 1 frame per 30 time units
            'time_per_frame': [1, LaunchConfiguration('fps')],
            
            # Frame ID for TF integration
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
            
            # Camera info URL for calibration
            'camera_info_url': LaunchConfiguration('camera_info_url'),
            
            # Output encoding
            'output_encoding': 'bgr8',
            
            # Image processing options
            'brightness': 128,  # Default brightness (0-255)
            'contrast': 128,    # Default contrast (0-255)
            'saturation': 128,  # Default saturation (0-255)
            
            # QoS settings
            'image_qos': 'sensor_data',  # Use sensor_data profile for images
            
        }],
        remappings=[
            # Namespace camera topics under /camera for clarity
            ('/image_raw', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info'),
        ],
    )
    
    return LaunchDescription([
        # Launch arguments
        camera_device_arg,
        image_width_arg,
        image_height_arg,
        fps_arg,
        camera_frame_id_arg,
        camera_info_url_arg,
        
        # Nodes
        camera_node,
    ])