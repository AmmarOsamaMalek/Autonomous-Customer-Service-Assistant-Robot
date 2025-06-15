# real_robot.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    pkg_robot_control = os.path.join(get_package_share_directory('robot_control'))
    pkg_rplidar_ros = os.path.join(get_package_share_directory('rplidar_ros'))

    # Launch RPLidar driver
    rplidar_node = Node(
        name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,  # A1 / A2
                # 'serial_baudrate': 256000, # A3
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
    )

    # Launch Raspberry Pi Camera node
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera_node',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [640, 480],
            'camera_frame_id': 'camera_link'
        }]
    )

    # Launch IMU 
    imu_node = Node(
        package='imu_tools',
        executable='imu_node',
        name='imu_node',
        parameters=[{
            'frame_id': 'imu_link',
            'port': '/dev/ttyACM0',
            'update_rate': 100.0
        }],
        output='screen'
    )


    # Launch Differential Drive Controller
    diff_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_robot_control, 'launch', 'diff_controller.launch.py'))
    )

    

    return LaunchDescription([
        rplidar_node,
        camera_node,
        imu_node,
        diff_controller_launch
    ])