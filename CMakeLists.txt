cmake_minimum_required(VERSION 3.0.2)
project(auv_homework)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES ros_cv_homework
  #  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs
  #  DEPENDS system_lib
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/img_processor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
