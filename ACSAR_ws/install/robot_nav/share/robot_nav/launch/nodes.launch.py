from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():

  
    
    # Autonomous Navigation Node with delay
    autonomous_navigation_node = Node(
        package='robot_nav',
        executable='Autonomous_navigation.py',
        output='screen'
    )

    # Launch Audio recording node (your speech recorder)
    speech_node = Node(
        package='robot_nav',
        executable='speech_recorder',
        name='speech_recorder',
        output='screen',
        parameters=[{
            'audio_device': 'plughw:CARD=M1pro,DEV=0',  
            'sample_rate': 16000,  
        }]
    )

    return LaunchDescription([
        autonomous_navigation_node,
        speech_node
    ])