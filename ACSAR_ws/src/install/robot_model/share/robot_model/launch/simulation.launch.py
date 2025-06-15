from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    pkg_robot_model = os.path.join(get_package_share_directory('robot_model'))
    pkg_robot_control = os.path.join(get_package_share_directory('robot_control'))

    # Launch Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_robot_model, 'launch', 'gazebo.launch.py'))
    )

    # Launch Differential Drive Controller
    diff_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_robot_control, 'launch', 'diff_controller.launch.py'))
    )
    
    control_delay = TimerAction(
        period=7.0,
        actions=[diff_controller_launch]
    )

    # Launch RViz2
    rviz2_node = ExecuteProcess(
        cmd=['ros2', 'run', 'rviz2', 'rviz2'],
        output='screen'
    )
    
    rviz_delay = TimerAction(
        period=6.0,
        actions=[rviz2_node]
    )

    return LaunchDescription([
        gazebo_launch,
        control_delay,
        rviz_delay
    ])