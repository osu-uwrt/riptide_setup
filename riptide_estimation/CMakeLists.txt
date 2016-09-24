cmake_minimum_required(VERSION 2.8.3)
project(riptide_estimation)

find_package(catkin REQUIRED COMPONENTS roslint roscpp tf sensor_msgs geometry_msgs)

catkin_package()

roslint_cpp()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(rpy src/rpy.cpp)
target_link_libraries(rpy ${catkin_LIBRARIES})
