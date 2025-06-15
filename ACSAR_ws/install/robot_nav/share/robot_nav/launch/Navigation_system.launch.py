#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    use_slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value='false',
        description='Use SLAM for mapping (true) or localization with existing map (false)'
    )
    
    # Get the launch configuration
    use_slam = LaunchConfiguration('use_slam')
    
    # Get package directories
    pkg_nav2_bringup = os.path.join(get_package_share_directory('nav2_bringup'))
    pkg_slam_toolbox = os.path.join(get_package_share_directory('slam_toolbox'))
    
    # Define paths
    map_file = '/home/ammar/Robotics/Graduation_Project/ACSAR_ws/class.yaml'
    nav2_params_file = '/home/ammar/Robotics/Graduation_Project/ACSAR_ws/src/robot_nav/config/nav2_params.yaml'
    slam_params_file = '/home/ammar/Robotics/Graduation_Project/ACSAR_ws/src/service_robot_localization/config/mapper_params_online_async.yaml'
    
    # Launch SLAM Toolbox (only when use_slam=true)
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': 'false'
        }.items(),
        condition=IfCondition(use_slam)
    )
    
    # Launch Localization (only when use_slam=false)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'false',
            'params_file': nav2_params_file
        }.items(),
        condition=UnlessCondition(use_slam)
    )

    # Launch Navigation
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': nav2_params_file,
            'use_sim_time': 'false'
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
        # Launch arguments
        use_slam_arg,
        
        # SLAM Toolbox (conditional)
        slam_toolbox_launch,
        
        # Nav2 localization (conditional)
        localization_launch,
        
        # Nav2 navigation (always)
        navigation_launch,
        
        # Custom navigation nodes
        autonomous_navigation_node,
       
        # Audio processing
        speech_node
    ])