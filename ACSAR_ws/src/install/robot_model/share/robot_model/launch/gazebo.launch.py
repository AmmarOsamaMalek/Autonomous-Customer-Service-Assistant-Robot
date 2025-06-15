from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,SetEnvironmentVariable,IncludeLaunchDescription
import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

 
def generate_launch_description():
    
    robot_model = get_package_share_directory("robot_model")
    robot_model_prefix = get_package_prefix("robot_model")
    model_path = os.path.join(robot_model,"models")
    model_path += pathsep + os.path.join(robot_model_prefix,"share")
    
    env_variable = SetEnvironmentVariable("GAZEBO_MODEL_PATH",model_path)
    
    # Add world file argument
    world_arg = DeclareLaunchArgument(
        name="world",
        default_value="/home/ammar/Robotics/Graduation_Project/ACSAR_ws/src/robot_model/worlds/worlds/ACSAR.world",
        description="Absolute path to Gazebo world file"
    )
    
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(robot_model, "urdf", "service_robot.urdf.xml"),
        description="Absolute path to robot URDF file"
    )
    
    robot_description = ParameterValue(Command(["xacro ",LaunchConfiguration("model")]),value_type=str)
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )
    
    # Modify Gazebo server launch to include world file
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("gazebo_ros"),
            "launch",
            "gzserver.launch.py"
        )),
        launch_arguments=[('world', LaunchConfiguration('world'))]
    )
    
    start_gazebo_client = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("gazebo_ros"),
        "launch",
        "gzclient.launch.py")))
    
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity","service_robot","-topic","robot_description"],
        output="screen"
    )
    
    return LaunchDescription([
        env_variable,
        world_arg,
        model_arg,
        robot_state_publisher,
        start_gazebo_server,
        start_gazebo_client,
        spawn_robot
    ])