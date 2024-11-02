from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.07"
    )
    
    wheel_width_arg = DeclareLaunchArgument(
        "wheel_width",
        default_value="0.35"
    )

    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_width = LaunchConfiguration("wheel_width")
    
    pkg_share1 = FindPackageShare(package='robot_model').find('robot_model')
    pkg_share2 = FindPackageShare(package='robot_control').find('robot_control')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([pkg_share1, 'urdf', 'service_robot.urdf.xml']),
            ' use_sim_time:=',
            LaunchConfiguration('use_sim_time')
        ]
    )

    # Robot description parameter
    robot_description = {'robot_description': robot_description_content}

    # Get controller configuration
    robot_controllers = PathJoinSubstitution(
        [pkg_share2, 'config', 'robot_controllers.yaml']
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )
    
    # Joint state broadcaster node
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]
    )

    service_robot_speed_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["service_robot_controller", "--controller-manager", "/controller_manager"]
        
    )
    
    # Main controller spawner
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["four_wheel_base_controller", "--controller-manager", "/controller_manager"],
    )

    # ROS2 Control node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="screen",
    )

    speed_controller_cpp = Node(
        package="robot_control",
        executable="speed_controller",
        parameters=[{"wheel_radius":wheel_radius,"wheel_width":wheel_width}]
    )
    

    nodes = [
        use_sim_time,
        wheel_radius_arg,
        wheel_width_arg,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        service_robot_speed_controller,
        robot_controller_spawner,
        control_node,
        speed_controller_cpp
    ]

    return LaunchDescription(nodes)