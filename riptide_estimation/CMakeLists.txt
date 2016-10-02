cmake_minimum_required(VERSION 2.8.3)
project(riptide_estimation)

find_package(catkin REQUIRED COMPONENTS roscpp tf sensor_msgs geometry_msgs)

catkin_package(INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

add_executable(rpy src/rpy.cpp)
target_link_libraries(rpy ${catkin_LIBRARIES})
