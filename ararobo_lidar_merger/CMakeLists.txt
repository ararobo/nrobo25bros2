cmake_minimum_required(VERSION 3.8)
project(ararobo_lidar_merger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED) # PCL utilities

add_executable(lidar_merger_node src/lidar_merger_node.cpp)

ament_target_dependencies(lidar_merger_node
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
  laser_geometry
  message_filters
  pcl_conversions
  pcl_ros
)

install(TARGETS
  lidar_merger_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
  laser_geometry
  message_filters
  pcl_conversions
  pcl_ros
)

ament_package()