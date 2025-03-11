cmake_minimum_required(VERSION 3.8)

project(andino_navigation)

find_package(ament_cmake REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(andino_interfaces REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    launch
    rviz
    maps
    params
  DESTINATION
    share/${PROJECT_NAME}/
)

# Battery observer library
add_library(battery_observer_lib SHARED
  src/battery_observer.cpp 
)

target_include_directories(battery_observer_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(battery_observer_lib
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(battery_observer_lib PUBLIC
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "std_msgs"
  "nav2_msgs"
  "andino_interfaces")

rclcpp_components_register_node(battery_observer_lib PLUGIN "BatteryObserver" EXECUTABLE battery_observer)

install(TARGETS
battery_observer_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Dock plugin
add_library(nav2_dock_plugin SHARED
  src/dock.cpp
)

target_include_directories(nav2_dock_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(nav2_dock_plugin PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(nav2_dock_plugin
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "nav2_behaviors"
  "andino_interfaces"
  "nav2_core"
  "nav2_msgs"
  "pluginlib"
)

pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)

install(TARGETS nav2_dock_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES behavior_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
