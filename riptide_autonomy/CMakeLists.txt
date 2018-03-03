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
)

find_package(catkin REQUIRED genmsg actionlib_msgs actionlib)
add_action_files(
    DIRECTORY action
    FILES Find.action LinearAlign.action)
generate_messages(DEPENDENCIES std_msgs geometry_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS actionlib_msgs)
