cmake_minimum_required(VERSION 3.5)
project(kimchi_state)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kimchi_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)

add_executable(kimchi_state_server src/kimchi_state_server.cpp)
ament_target_dependencies(kimchi_state_server PUBLIC
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "std_msgs"
  "geometry_msgs"
  "kimchi_interfaces"
  "std_srvs"
  "nav2_msgs"
  "lifecycle_msgs"
  "nav2_lifecycle_manager")

target_include_directories(kimchi_state_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

 install(TARGETS
 kimchi_state_server
  DESTINATION lib/${PROJECT_NAME})

ament_package()
