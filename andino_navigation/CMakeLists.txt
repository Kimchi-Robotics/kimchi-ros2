cmake_minimum_required(VERSION 3.8)

project(andino_navigation)
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    rviz
    maps
    params
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
