cmake_minimum_required(VERSION 3.8)
project(second_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# ── Goal Publisher Node ──────────────────────────────────────────────────────
add_executable(goal_publisher src/goal_publisher.cpp)
ament_target_dependencies(goal_publisher
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ── Install launch, config, csv, map folders ─────────────────────────────────
install(DIRECTORY
  launch
  config
  csv
  map
  rviz
  src
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS goal_publisher
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
