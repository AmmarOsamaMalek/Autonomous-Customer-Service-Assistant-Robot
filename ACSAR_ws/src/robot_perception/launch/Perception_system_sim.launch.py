#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('robot_perception')  # Use your actual package name
    
    # Create a parameter file path for hand detector
    params_file = os.path.join(pkg_dir, 'config', 'detector_params.yaml')
    
    # Launch USB camera node with specific settings for simulation
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'yuyv',
            'camera_frame_id': 'camera_link',
            'framerate': 30.0,
        }],
        remappings=[
            ('image', '/image_raw'),
        ]
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
    
    # RQT for image debugging
    rqt_image_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        arguments=['/hand_detection_debug']
    )
  

   

    return LaunchDescription([
        camera_node,
        perception_node,
        rqt_image_view,
        emotion_recognition_node
       
    ])