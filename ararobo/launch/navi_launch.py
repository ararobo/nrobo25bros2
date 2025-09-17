from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 別のlaunchファイルがあるパッケージの共有ディレクトリを取得
    ydlidar_share_dir = get_package_share_directory('ydlidar_ros2_driver')
    slam_share_dir = get_package_share_directory('slam_toolbox')

    # 起動したい子launchファイルのパスを指定
    ydlidar_launch_file_path = os.path.join(ydlidar_share_dir, 'launch', 'ydlidar_launch.py')
    slam_launch_file_path = os.path.join(slam_share_dir, 'launch', 'online_async_launch.py')

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