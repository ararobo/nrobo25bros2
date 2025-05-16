cmake_minimum_required(VERSION 3.8)
project(ararobo_navi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(algorithm REQUIRED)
add_executable(navi_node src/navi_node.cpp)
ament_target_dependencies(navi_node
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  algorithm
  
)
target_include_directories(navi_node PUBLIC # includeというフォルダがあるが、その中のヘッダファイルをソースコードで使用したりするため
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS # この中に書いたものはbuild時に実際にbuildされます。
  navi_node # my_nodeをビルドする
  DESTINATION lib/${PROJECT_NAME}
)


ament_package() 