#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('robot_perception')  
    pkg_rplidar_ros = get_package_share_directory('rplidar_ros')
    
    # Create a parameter file path for hand detector
    params_file = os.path.join(pkg_dir, 'config', 'detector_params.yaml')
    
    # Launch RPLidar driver
    rplidar_node = Node(
        name='rplidar_composition',
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB1',
            'serial_baudrate': 115200,  # A1 / A2
            # 'serial_baudrate': 256000, # A3
            'frame_id': 'lidar_link',
            'inverted': False,
            'angle_compensate': True,
        }],
    )

    # Launch Raspberry Pi Camera node
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera_node',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [640, 480],
            'camera_frame_id': 'camera_link'
        }]
    )
    
     # Perception node
    perception_node = Node(
        package='robot_perception',
        executable='perception.py',
        output='screen'
    )
    
    emotion_recognition_node = Node(
        package='robot_perception',
        executable='Emotion_Recognition.py',
        output='screen'
    )
    
    
    return LaunchDescription([
        #rplidar_node,
        #camera_node,
        perception_node
        #emotion_recognition_node
       
    ])