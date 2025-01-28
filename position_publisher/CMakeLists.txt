cmake_minimum_required(VERSION 3.5)
project(position_publisher)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Add executables
add_executable(position_publisher src/position_publisher.cpp)
ament_target_dependencies(position_publisher rclcpp std_msgs trajectory_msgs)

add_executable(position_subscriber src/position_subscriber.cpp)
ament_target_dependencies(position_subscriber rclcpp std_msgs trajectory_msgs) 

add_executable(kinect_to_position_publisher src/kinect_to_position_publisher.cpp)
ament_target_dependencies(kinect_to_position_publisher rclcpp std_msgs trajectory_msgs) 

# Install targets
install(TARGETS
  position_publisher
  position_subscriber
  kinect_to_position_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()