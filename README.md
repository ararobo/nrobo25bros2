# NHK Kosen Robot Contest 2025 TMCIT Arakawa B Team's ROS2 packages
ロボット-PC間通信起動
```bash
ros2 launch ararobo robot_launch.py
```
コントローラー間通信起動
```bash
ros2 run ararobo_robot controller_node
```
手動制御起動
```bash
ros2 launch ararobo manual_launch.py
```
LiDAR起動
```bash
ros2 launch ararobo lidar_launch.py
```
自己位置推定起動
```bash
ros2 launch ararobo localization_launch.py
```
ナビゲーション起動
```bash
ros2 launch ararobo navi_launch.py
```
