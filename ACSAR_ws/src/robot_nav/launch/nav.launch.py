import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Find package directories
    try:
        robot_nav_pkg = get_package_share_directory('robot_nav')
    except Exception as e:
        return LaunchDescription([
            LogInfo(msg=f'Error finding robot_nav package: {str(e)}')
        ])
    
    # Navigation parameters path
    nav2_params_path = os.path.join(
        robot_nav_pkg, 
        'config', 
        'nav.yaml'
    )

    if not os.path.exists(nav2_params_path):
        return LaunchDescription([
            LogInfo(msg=f'Nav2 params file not found at: {nav2_params_path}')
        ])

    # Spin action server node (added)
    spin_server = Node(
        package='nav2_behaviors',
        executable='spin_action_server',
        name='spin_action_server',
        output='screen',
        parameters=[nav2_params_path]
    )

    # Backup action server node (added)
    backup_server = Node(
        package='nav2_behaviors',
        executable='backup_action_server',
        name='backup_action_server',
        output='screen',
        parameters=[nav2_params_path]
    )

    # Lifecycle manager node
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': [
                'spin_action_server',
                'backup_action_server',
                'amcl', 
                'controller_server', 
                'planner_server', 
                'bt_navigator'
            ]}
        ]
    )

    # Base nodes
    base_nodes = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0', '0', '0.55', '0', '0', '0', 'base_link', 'laser_link']
        ),
        Node(
            package='robot_nav',
            executable='navigator',
            name='navigator',
            parameters=[{
                'goal_offset': 0.5,
                'base_frame': 'base_link',
                'map_frame': 'odom',
                'max_retry_attempts': 10
            }]
        )
    ]

    # Nav2 nodes
    nav2_nodes = [
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[nav2_params_path],
            remappings=[('scan', '/scan')]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[nav2_params_path],
            output='screen'
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[nav2_params_path]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[nav2_params_path]
        )
    ]

    return LaunchDescription([
        use_sim_time,
        spin_server,  # Added spin action server
        backup_server,  # Added backup action server
        *base_nodes,
        *nav2_nodes,
        lifecycle_manager
    ])