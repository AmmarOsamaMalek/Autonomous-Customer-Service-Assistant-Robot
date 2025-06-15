#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    pkg_nav2_bringup = os.path.join(get_package_share_directory('nav2_bringup'))

    # Define paths
    map_file = '/home/ammar/Robotics/Graduation_Project/ACSAR_ws/class.yaml'
    nav2_params_file = '/home/ammar/Robotics/Graduation_Project/ACSAR_ws/src/robot_nav/config/nav2_params.yaml'

    # Launch Localization
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true',
            'params_file': nav2_params_file
        }.items()
    )

    # Launch Navigation
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': nav2_params_file,
            'use_sim_time': 'true'
        }.items()
    )
    
    # Autonomous Navigation Node
    autonomous_navigation_node = Node(
        package='robot_nav',
        executable='Autonomous_navigation.py',
        output='screen'
    )

    # Launch Audio recording node
    speech_node = Node(
        package='robot_nav',  
        executable='Speach_recorder.py',
        parameters=[{
            'audio_device': 'plughw:CARD=M1pro,DEV=0',  
            'sample_rate': 16000,  
        }]
    )
    
   
    return LaunchDescription([
        # Nav2 components
        localization_launch,
        navigation_launch,
        
        # Custom navigation nodes
        autonomous_navigation_node,
       
        # Audio processing
        speech_node
    ])