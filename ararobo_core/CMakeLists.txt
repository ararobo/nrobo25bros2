cmake_minimum_required(VERSION 3.8)
project(ararobo_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(core_node src/core_node.cpp src/simple_udp.cpp)

ament_target_dependencies(core_node
  rclcpp
  std_msgs
  geometry_msgs
)

target_include_directories(core_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  core_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
