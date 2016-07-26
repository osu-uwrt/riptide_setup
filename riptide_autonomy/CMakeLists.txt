cmake_minimum_required(VERSION 2.8.3)
project(riptide_autonomy)

find_package(catkin REQUIRED COMPONENTS smach_ros roscpp actionlib actionlib_msgs riptide_msgs)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

add_executable(navigate_action_server src/navigate_action_server.cpp)

target_link_libraries(navigate_action_server ${catkin_LIBRARIES})

include_directories(${catkin_INCLUDE_DIRS})


