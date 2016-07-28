cmake_minimum_required(VERSION 2.8.3)
project(riptide_autonomy)

find_package(catkin REQUIRED COMPONENTS smach_ros roscpp actionlib actionlib_msgs riptide_msgs)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(CATKIN_DEPENDS riptide_msgs)

add_executable(action_server_template src/action_server_template.cpp)

target_link_libraries(action_server_template ${catkin_LIBRARIES})

add_dependencies(action_server_template ${catkin_EXPORTED_TARGETS})

include_directories(${catkin_INCLUDE_DIRS})


