# real_robot.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    pkg_robot_control = get_package_share_directory('robot_control')
    pkg_robot_nav = get_package_share_directory('robot_nav')
    pkg_robot_perception = get_package_share_directory('robot_perception')
    
    # Launch Differential Drive Controller
    diff_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_robot_control, 'launch', 'diff_controller.launch.py'))
    )
    
    # Launch Perception System
    perception_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_robot_perception, 'launch', 'Perception_system_real.launch.py'))
    )
    
    # Launch Navigation System
    navigation_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_robot_nav, 'launch', 'Navigation_system.launch.py'))
    )

    return LaunchDescription([
        # Core robot control
        diff_controller_launch,
        
        # Perception system (LiDAR, camera)
        perception_system_launch,
        
        # Navigation system
        navigation_system_launch
    ])