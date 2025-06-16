cmake_minimum_required(VERSION 3.8)

project(kimchi_navigation)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)

find_package(kimchi_interfaces REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Include directories
include_directories(include)

add_executable(amcl_initial_pose_trigger src/amcl_initial_pose_trigger.cpp)
ament_target_dependencies(amcl_initial_pose_trigger rclcpp geometry_msgs)

add_executable(amcl_global_trigger_correct src/amcl_global_trigger_correct.cpp)
ament_target_dependencies(amcl_global_trigger_correct rclcpp geometry_msgs tf2 tf2_ros)

add_executable(global_localization src/global_localization.cpp)
target_compile_definitions(global_localization
  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL")
ament_target_dependencies(global_localization
  rclcpp
  rclcpp_action
  geometry_msgs
  kimchi_interfaces)

# Install executables
install(TARGETS
  amcl_initial_pose_trigger
  amcl_global_trigger_correct
  global_localization
  DESTINATION lib/${PROJECT_NAME}
)

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
