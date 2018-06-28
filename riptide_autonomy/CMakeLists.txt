cmake_minimum_required(VERSION 2.8.3)
project(riptide_autonomy)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  rospy
  smach
  smach_ros
  image_transport
  cv_bridge
  rosconsole
  sensor_msgs
  std_msgs
  geometry_msgs
  riptide_msgs
  message_generation
)

find_package(catkin REQUIRED genmsg actionlib_msgs actionlib)

generate_messages(DEPENDENCIES std_msgs geometry_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS actionlib_msgs)
