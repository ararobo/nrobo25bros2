cmake_minimum_required(VERSION 3.8)
project(ararobo_navi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(navi_node src/navi_node.cpp)

target_include_directories(navi_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 💡 これを追加！
ament_target_dependencies(navi_node
  rclcpp
  tf2
  tf2_ros
)

install(TARGETS
  navi_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
