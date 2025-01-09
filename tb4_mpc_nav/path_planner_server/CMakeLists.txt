cmake_minimum_required(VERSION 3.10)
project(tb4_path_planner_server)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)

# add_executable(path_planner_server src/path_planner_server.cpp)
# ament_target_dependencies(path_planner_server rclcpp nav2_controller nav2_bt_navigator nav2_recoveries)

# install(TARGETS path_planner_server
#   DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()