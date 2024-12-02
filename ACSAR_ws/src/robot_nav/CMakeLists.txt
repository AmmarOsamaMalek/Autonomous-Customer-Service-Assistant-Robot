cmake_minimum_required(VERSION 3.8)
project(robot_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add executable
add_executable(navigator src/navigator.cpp)

ament_target_dependencies(navigator
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install
install(TARGETS
  navigator
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
