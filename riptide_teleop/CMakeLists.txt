cmake_minimum_required(VERSION 2.8.3)
project(riptide_teleop)

find_package(catkin REQUIRED COMPONENTS tf joy roslaunch riptide_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(accel_cmdr src/accel_cmd.cpp)
target_link_libraries(accel_cmdr ${catkin_LIBRARIES})

add_executable(commander src/commander.cpp)
target_link_libraries(commander ${catkin_LIBRARIES})
add_dependencies(commander riptide_msgs_gencpp)
