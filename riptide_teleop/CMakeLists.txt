cmake_minimum_required(VERSION 2.8.3)
project(riptide_teleop)

find_package(catkin REQUIRED COMPONENTS tf joy roslaunch)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(accel_cmdr src/accel_cmd.cpp)
target_link_libraries(accel_cmdr ${catkin_LIBRARIES})

add_executable(attitude_cmdr src/att_cmd.cpp)
target_link_libraries(attitude_cmdr ${catkin_LIBRARIES})

add_executable(joy_accel_set src/accel_set.cpp)
target_link_libraries(joy_accel_set ${catkin_LIBRARIES})
