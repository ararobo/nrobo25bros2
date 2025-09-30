from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='ararobo_control',
            executable='hand_node',
            name='hand_node',
            output='screen'
        ),
        Node(
            package='ararobo_control',
            executable='move_node',
            name='move_node',
            output='screen'
        ),
    ])