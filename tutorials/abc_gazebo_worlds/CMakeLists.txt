cmake_minimum_required(VERSION 3.8)
project(abc_gazebo_worlds)

find_package(ament_cmake REQUIRED)

include_directories(
    dynamic_worlds
    launch
    models
    worlds
)

install(
    DIRECTORY
        dynamic_worlds
        launch
        models
        worlds
    DESTINATION share/${PROJECT_NAME}
)

# Include the world/ and model/ to gazebo_path
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
