cmake_minimum_required(VERSION 3.10)
project(map_server)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Add executable
# add_executable(map_server_node src/map_server_node.cpp)
# ament_target_dependencies(map_server_node rclcpp std_msgs nav_msgs)

# Install
# install(TARGETS map_server_node
#   DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch maps config
  DESTINATION share/${PROJECT_NAME})

ament_package()