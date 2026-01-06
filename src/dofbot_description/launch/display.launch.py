import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('dofbot_description')
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_gui = LaunchConfiguration('use_gui', default='true')
    rviz_conf = LaunchConfiguration('rviz_conf', default=os.path.join(pkg_path, 'rviz', 'default.rviz'))

    # Read URDF file directly (avoiding xacro Command issues)
    urdf_file = os.path.join(pkg_path, 'urdf', 'dofbot.urdf')
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    robot_description = {'robot_description': robot_description_content}

    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_gui)
    )

    # Delay Rviz launch to ensure robot_description is published first
    rviz_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_conf],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('use_gui', default_value='true', description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument('rviz_conf', default_value=os.path.join(pkg_path, 'rviz', 'default.rviz'), description='Path to rviz configuration'),
        
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])