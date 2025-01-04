from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths to required launch files and parameters
    pkg_robot_model = os.path.join(get_package_share_directory('robot_model'))
    pkg_robot_control = os.path.join(get_package_share_directory('robot_control'))
    pkg_nav2_bringup = os.path.join(get_package_share_directory('nav2_bringup'))

    map_file = '/home/ammar/Robotics/Graduation_Project/ACSAR_ws/class.yaml'
    nav2_params_file = '/home/ammar/Robotics/Graduation_Project/ACSAR_ws/src/robot_nav/config/nav2_params.yaml'

    # Launch Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_robot_model, 'launch', 'gazebo.launch.py'))
    )

    # Delay for Gazebo to initialize
    gazebo_delay = TimerAction(
        period=10.0,  # 5 seconds delay
        actions=[gazebo_launch]
    )

    # Launch RViz2
    
    # Launch Differential Drive Controller
    diff_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_robot_control, 'launch', 'diff_controller.launch.py'))
    )
    
    control_delay = TimerAction(
        period=3.0,  # 5 seconds delay
        actions=[diff_controller_launch]
    )

    # Launch Localization
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true'
        }.items()
    )

    # Delay for Localization
    localization_delay = TimerAction(
        period=10.0,  # 10 seconds delay
        actions=[localization_launch]
    )

    # Publish Initial Pose
    initial_pose_publisher = ExecuteProcess(
    cmd=[
        'ros2', 'topic', 'pub', '--once', '/initialpose', 'geometry_msgs/PoseWithCovarianceStamped',
        '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}'
    ],
    output='screen'
    )

    
    # Launch Navigation
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': nav2_params_file,
            'use_sim_time': 'true'
        }.items()
    )
    
    nav_delay = TimerAction(
        period=5.0,  # 5 seconds delay
        actions=[navigation_launch]
    )
    
    rviz2_node = ExecuteProcess(
        cmd=['ros2', 'run', 'rviz2', 'rviz2'],
        output='screen'
    )
    
    rviz_delay = TimerAction(
        period=5.0,  # 5 seconds delay
        actions=[rviz2_node]
    )

    # USB Camera Node
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        output='screen'
    )

    # Perception Node
    perception_node = Node(
        package='robot_perception',
        executable='perception.py',
        output='screen'
    )

    # Autonomous Navigation Node
    autonomous_navigation_node = Node(
        package='robot_nav',
        executable='Autonomous_navigation.py',
        output='screen'
    )

    # Speech Recorder Node
    speech_recorder_node = Node(
        package='robot_nav',
        executable='Speach_recorder.py',
        output='screen'
    )
    
    

    # Combine all the processes with delays
    return LaunchDescription([
        gazebo_delay,
        control_delay,
        initial_pose_publisher,
        localization_delay,
        nav_delay,
        rviz_delay,
        usb_cam_node,
        perception_node,
        autonomous_navigation_node,
        speech_recorder_node
    ])
