cmake_minimum_required(VERSION 2.8.3)
project(riptide_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs actionlib_msgs)

add_action_files(DIRECTORY action FILES Navigate.action)

add_message_files(FILES Thrust.msg ThrustStamped.msg Pwm.msg PwmStamped.msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS message_runtime geometry_msgs actionlib_msgs)

include_directories(${catkin_INCLUDE_DIRS})
