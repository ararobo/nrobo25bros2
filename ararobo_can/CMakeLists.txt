cmake_minimum_required(VERSION 3.8)
project(ararobo_can)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
)

add_executable(robot src/robot_node.cpp src/can_config.cpp src/can_driver.cpp src/pc_data_slave.cpp)

ament_target_dependencies(robot
  ${dependencies}
)

target_include_directories(robot PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(TARGETS
  robot
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
