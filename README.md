# NHK Kosen Robot Contest 2025 TMCIT Arakawa B Team's ROS2 packages
slam_toolbox起動
```rb
ros2 launch slam_toolbox online_async_launch.py 
```
LiDAR起動
```rb
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```
キーボード操作
```rb
ros2 run ararobo_teleop_keyboard teleop_keyboard
```
