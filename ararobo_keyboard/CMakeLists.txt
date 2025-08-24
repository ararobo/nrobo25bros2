cmake_minimum_required(VERSION 3.8)
project(ararobo_keyboard)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ararobo_msgs REQUIRED)

add_executable(teleop_keyboard src/teleop_keyboard.cpp src/linux_keyboard_driver.cpp)

ament_target_dependencies(teleop_keyboard
  rclcpp
  geometry_msgs
  ararobo_msgs
)

target_include_directories(teleop_keyboard PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  teleop_keyboard
  DESTINATION lib/${PROJECT_NAME})
ament_package()
