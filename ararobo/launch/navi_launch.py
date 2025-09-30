from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='ararobo_navi',
            executable='planner_node',
            name='planner_node',
            output='screen'
        ),
        Node(
            package='ararobo_control',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen'
        ),
    ])