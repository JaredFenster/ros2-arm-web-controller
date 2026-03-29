cmake_minimum_required(VERSION 3.22)
project(panda_gz_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf config worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/floor_image_projector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
