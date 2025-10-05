cmake_minimum_required(VERSION 3.8)
project(ararobo_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
)

# robot
add_executable(operation_node src/operation_node.cpp src/simple_udp.cpp)

ament_target_dependencies(operation_node
  ${dependencies}
)

target_include_directories(operation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# feedback
add_executable(feedback_node src/feedback_node.cpp src/simple_udp.cpp src/odom_calculator.cpp)

ament_target_dependencies(feedback_node
  ${dependencies}
)

target_include_directories(feedback_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# controller
add_executable(controller_node src/controller_node.cpp src/simple_udp.cpp)

ament_target_dependencies(controller_node
  ${dependencies}
)

target_include_directories(controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# param
add_executable(param_node src/param_node.cpp src/simple_udp.cpp)

ament_target_dependencies(param_node
  ${dependencies}
)

target_include_directories(param_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  operation_node
  feedback_node
  controller_node
  param_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
