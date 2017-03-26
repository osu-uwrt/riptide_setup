cmake_minimum_required(VERSION 2.8.3)
project(riptide_hardware)

find_package(catkin REQUIRED COMPONENTS roscpp roslint riptide_msgs)

catkin_package()

roslint_cpp()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(thrust_cal src/thrust_cal.cpp)
add_dependencies(thrust_cal riptide_msgs_gencpp)
target_link_libraries(thrust_cal ${catkin_LIBRARIES})
