# advanced_diff_drive.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share1 = get_package_share_directory('robot_control')
    pkg_share2 = get_package_share_directory('robot_model')
    
    # Define paths to URDF and controller config file
    urdf_file = os.path.join(pkg_share2, 'urdf', 'service_robot.urdf.xml')
    controller_config_file = os.path.join(pkg_share1, 'config', 'diff_controller.yaml')
    
    # Define the launch description
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time if true'),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}, {'robot_description': open(urdf_file).read()}],
        ),
        
        # Controller Manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controller_config_file],
            output='screen',
        ),
        
        # Spawner for joint_state_broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        
        # Spawner for the differential drive controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller'],
            output='screen',
        ),
    ])
