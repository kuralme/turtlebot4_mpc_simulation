cmake_minimum_required(VERSION 3.10)
project(path_planner_server)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_bt_navigator REQUIRED)

# add_executable(path_planner_server src/path_planner_server.cpp)
# ament_target_dependencies(path_planner_server rclcpp nav2_controller nav2_bt_navigator nav2_recoveries)

# install(TARGETS path_planner_server
#   DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()