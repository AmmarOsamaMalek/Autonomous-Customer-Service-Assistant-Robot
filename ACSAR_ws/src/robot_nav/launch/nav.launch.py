from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_nav2_bringup = os.path.join(get_package_share_directory('nav2_bringup'))

    # Define paths
    map_file = '/home/ammar/Robotics/Graduation_Project/ACSAR_ws/class.yaml'
    nav2_params_file = '/home/ammar/Robotics/Graduation_Project/ACSAR_ws/src/robot_nav/config/nav2_params.yaml'

    # Launch Localization
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'false',
            'params_file': nav2_params_file
        }.items()
    )

    # Launch Navigation
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': nav2_params_file,
            'use_sim_time': 'false'
        }.items()
    )
    
    return LaunchDescription([
        localization_launch,
        navigation_launch
    ])
