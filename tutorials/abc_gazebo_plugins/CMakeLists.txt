cmake_minimum_required(VERSION 3.8)
project(abc_gazebo_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)

# Add the plugin with it's dependencies
add_library(abc_sample_plugin SHARED
src/abc_sample_plugin.cpp
)

target_include_directories(abc_sample_plugin PUBLIC include)
ament_target_dependencies(abc_sample_plugin
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "std_msgs"
)
ament_export_libraries(abc_sample_plugin)
target_link_libraries(abc_sample_plugin)

install(DIRECTORY include
DESTINATION include/
)

install(TARGETS
abc_sample_plugin
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
)

ament_package()
