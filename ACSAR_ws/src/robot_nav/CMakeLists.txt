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
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_behavior_tree REQUIRED)



install(PROGRAMS
  scripts/Autonomous_navigation.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/simple_nav.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/Speach_recorder.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)




ament_package()
