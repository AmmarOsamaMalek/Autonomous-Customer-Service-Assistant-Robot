cmake_minimum_required(VERSION 3.8)
project(robot_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Install Python scripts to lib/package_name
install(PROGRAMS
  scripts/perception.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/process_image.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/detect_ball.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()