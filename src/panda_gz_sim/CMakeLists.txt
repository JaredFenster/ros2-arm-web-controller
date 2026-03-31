cmake_minimum_required(VERSION 3.22)
project(panda_gz_sim)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)

add_executable(map_points scripts/map_points.cpp)
ament_target_dependencies(map_points
  rclcpp
  sensor_msgs
  std_srvs
  tf2_ros
  tf2_sensor_msgs
)

add_library(store_points_panel SHARED
  src/store_points_panel.cpp
)
target_include_directories(store_points_panel PUBLIC src)
ament_target_dependencies(store_points_panel
  rclcpp
  std_srvs
  rviz_common
  pluginlib
)


install(TARGETS map_points store_points_panel
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch urdf config worlds
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
