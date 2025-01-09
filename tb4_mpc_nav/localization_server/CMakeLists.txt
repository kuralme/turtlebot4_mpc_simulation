cmake_minimum_required(VERSION 3.10)
project(tb4_localization_server)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Add executable
# add_executable(localization_server_node src/localization_server_node.cpp)
# ament_target_dependencies(localization_server_node rclcpp std_msgs sensor_msgs nav_msgs)

# Install
# install(TARGETS localization_server_node
#   DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()