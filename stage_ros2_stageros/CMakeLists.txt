cmake_minimum_required(VERSION 3.16)
project(stage_ros2_stageros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Stage has appeared under different CMake/pkg-config layouts across distros.
find_package(stage QUIET)
if(NOT stage_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(STAGE REQUIRED stage)
endif()

add_executable(stageros src/stageros.cpp)

ament_target_dependencies(stageros
  rclcpp
  geometry_msgs
  nav_msgs
  rosgraph_msgs
  sensor_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

if(stage_FOUND)
  target_include_directories(stageros PRIVATE ${STAGE_INCLUDE_DIRS})
  target_link_libraries(stageros ${STAGE_LIBRARIES})
else()
  target_include_directories(stageros PRIVATE ${STAGE_INCLUDE_DIRS})
  target_link_directories(stageros PRIVATE ${STAGE_LIBRARY_DIRS})
  target_link_libraries(stageros ${STAGE_LIBRARIES})
endif()

if(UNIX AND NOT APPLE)
  target_link_libraries(stageros dl)
endif()

install(TARGETS stageros
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch world config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
