from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share1 = get_package_share_directory('robot_control')
    pkg_share2 = get_package_share_directory('robot_model')
    
    # Define paths to URDF and controller config file
    urdf_file = os.path.join(pkg_share2, 'urdf', 'service_robot.urdf.xml')
    controller_config_file = os.path.join(pkg_share1, 'config', 'diff_controller.yaml')
    
    # Use xacro to process robot description
    robot_description = ParameterValue(Command(["xacro ", urdf_file]), value_type=str)
    
    # Define Simulation Time Launch Argument with more explicit configuration
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true', 
        description='Use simulation (Gazebo) time if true, else use system time',
        choices=['true', 'false']
    )
    
    # Define the launch description
    return LaunchDescription([
        use_sim_time_arg,
        
        # Robot State Publisher with explicit simulation time parameter
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}, 
                {'robot_description': robot_description}
            ]
        ),
        
        # Joint State Publisher GUI with simulation time
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        
        # Standard Joint State Publisher with simulation time
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        
        # Controller Manager with simulation time configuration
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                controller_config_file,
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen',
        ),
        
        # Joint State Broadcaster Spawner
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
        ),
        
        # Differential Drive Controller Spawner
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller'],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
        ),
    ])