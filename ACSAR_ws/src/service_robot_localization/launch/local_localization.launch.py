from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x","0","--y","0","--z","0.05","--qx","0","--qy","0","--qz","0","--qw","1",
                   "--frame-id","base_link_ekf","--child-frame-id","imu_link_ekf"]
    )
    
    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("service_robot_localization"),"config","ekf.yaml")]
    )
    
    imu_republisher_cpp = Node(
        package="robot_control",
        executable="imu_republisher"
    )
    
    return LaunchDescription([
        robot_localization
    ])