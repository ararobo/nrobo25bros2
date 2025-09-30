from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='ararobo_robot',
            executable='operation_node',
            name='operation_node',
            output='screen'
        ),
        Node(
            package='ararobo_robot',
            executable='feedback_node',
            name='feedback_node',
            output='screen'
        ),
    ])