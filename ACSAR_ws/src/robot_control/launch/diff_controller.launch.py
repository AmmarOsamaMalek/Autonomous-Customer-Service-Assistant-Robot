from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Get package directories
    pkg_share1 = get_package_share_directory('robot_control')
    pkg_share2 = get_package_share_directory('robot_model')
    
    # Define paths
    urdf_file = os.path.join(pkg_share2, 'urdf', 'service_robot_real.urdf.xml')
    controller_config_file = os.path.join(pkg_share1, 'config', 'diff_controller.yaml')
    
    # Verify file existence
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")
    if not os.path.exists(controller_config_file):
        raise FileNotFoundError(f"Controller config file not found: {controller_config_file}")
    
    # Robot description
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Controller manager with robot description parameter
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controller_config_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
        respawn=True,  # Automatically respawn if node crashes
    )
    
    # Robot state publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Joint state publisher
    joint_state_pub_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Spawner nodes with dependencies
    joint_state_broadcaster_spawner = TimerAction(
    period=3.0,
    actions=[
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', 
                       '--controller-manager', '/controller_manager'],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        )
    ]
)

    
    diff_drive_controller_spawner = TimerAction(
    period=5.0,
    actions=[
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller', 
                       '--controller-manager', '/controller_manager'],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        )
    ]
)
    # Create launch description with ordered node startup
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    ))
    
    # Add nodes in sequence
    ld.add_action(controller_manager_node)
    ld.add_action(robot_state_pub_node)
    ld.add_action(joint_state_pub_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(diff_drive_controller_spawner)
    
    return ld