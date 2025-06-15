# simulation.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    pkg_robot_model = get_package_share_directory('robot_model')
    pkg_robot_control = get_package_share_directory('robot_control')
    pkg_robot_nav = get_package_share_directory('robot_nav')
    pkg_robot_perception = get_package_share_directory('robot_perception')

    # Launch Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_robot_model, 'launch', 'gazebo.launch.py'))
    )

    # Launch Differential Drive Controller with delay to allow Gazebo to fully start
    diff_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_robot_control, 'launch', 'diff_controller_sim.launch.py'))
    )
    
    diff_controller_delay = TimerAction(
        period=7.0,
        actions=[diff_controller_launch]
    )
    
    # Launch Perception System with delay
    perception_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_robot_perception, 'launch', 'Perception_system_sim.launch.py'))
    )
    
    perception_delay = TimerAction(
        period=9.0,
        actions=[perception_system_launch]
    )
    
    # Launch Navigation System with delay
    navigation_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_robot_nav, 'launch', 'Navigation_system_sim.launch.py'))
    )
    
    navigation_delay = TimerAction(
        period=12.0,
        actions=[navigation_system_launch]
    )

    # Launch RViz2 with delay
    rviz2_node = ExecuteProcess(
        cmd=['ros2', 'run', 'rviz2', 'rviz2'],
        output='screen'
    )
    
    rviz_delay = TimerAction(
        period=15.0,
        actions=[rviz2_node]
    )

    return LaunchDescription([
        # Start gazebo first
        gazebo_launch,
        
        # Launch controllers after gazebo is ready
        diff_controller_delay,
        
        # Launch perception after controllers are ready
        perception_delay,
        
        # Launch navigation after perception is ready
        navigation_delay,
        
        # Launch RViz2 last
        rviz_delay
    ])