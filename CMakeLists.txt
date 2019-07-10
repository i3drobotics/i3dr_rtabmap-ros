cmake_minimum_required(VERSION 2.8.3)
project(i3dr_rtabmap)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    sensor_msgs
    imu_filter_madgwick
    rtabmap
    tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
