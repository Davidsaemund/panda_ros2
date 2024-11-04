cmake_minimum_required(VERSION 3.8)
project(humble_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tutorial_interfaces REQUIRED)

# Add executable
add_executable(humble_control_node src/humble_control.cpp)
ament_target_dependencies(humble_control_node rclcpp std_msgs tutorial_interfaces)

# Install the executable
install(TARGETS
  humble_control_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
