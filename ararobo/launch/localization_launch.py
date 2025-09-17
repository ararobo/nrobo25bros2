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

    tf2_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser',
                    arguments=['0', '-0.475', '0','0', '0', '0', '1','odom_link','base_link'],
                    )

    return LaunchDescription([
        tf2_node,
        # 子となるlaunchファイルをインクルード
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([slam_launch_file_path])
        )
    ])