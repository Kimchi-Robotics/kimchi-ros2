cmake_minimum_required(VERSION 3.10.2)
project(abc_rostest_sample)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
)

catkin_package(CATKIN_DEPENDS rospy std_msgs)

catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest)
  find_package(abc_gazebo_worlds)
  add_rostest(test/test-gazebo-ros.test)
endif()
