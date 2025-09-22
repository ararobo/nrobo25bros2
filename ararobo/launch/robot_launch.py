from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 別のlaunchファイルがあるパッケージの共有ディレクトリを取得
    ydlidar_share_dir = get_package_share_directory('ydlidar_ros2_driver')
    urg_share_dir = get_package_share_directory('urg_node2')
    slam_share_dir = get_package_share_directory('slam_toolbox')

    # 起動したい子launchファイルのパスを指定
    ydlidar_launch_file_path = os.path.join(ydlidar_share_dir, 'launch', 'ydlidar_launch.py')
    urg_launch_file_path = os.path.join(urg_share_dir, 'launch', 'urg_node2.launch.py')
    slam_launch_file_path = os.path.join(slam_share_dir, 'launch', 'online_async_launch.py')
    
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
        Node(package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser',
            arguments=['0', '-0.35', '0.02','-1.57075', '0', '-3.1415','base_link','laser'],
        ),
        Node(
            package='ararobo_core',
            executable='core_node',
            name='core_node',
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([urg_launch_file_path])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ydlidar_launch_file_path])
        ),
    ])