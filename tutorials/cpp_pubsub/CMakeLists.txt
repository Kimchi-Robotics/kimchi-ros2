cmake_minimum_required(VERSION 3.10.2)
project(cpp_pubsub)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/listener.cpp)
ament_target_dependencies(listener rclcpp std_msgs)


install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_find_gtest()

  set(TEST_NAME test_example)
  ament_add_gtest_executable(${TEST_NAME} test/${TEST_NAME}.cpp)

  include_directories(${CMAKE_CURRENT_BINARY_DIR})
  install(
    TARGETS ${TEST_NAME}
    DESTINATION lib/${PROJECT_NAME}
  )
  add_launch_test(launch/${TEST_NAME}.launch.py
    TIMEOUT 200
  )
endif()

ament_package()
