cmake_minimum_required(VERSION 2.8.3)
project(riptide_solver)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs riptide_msgs)
find_package(Ceres REQUIRED)

catkin_package()

#include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

#add_executable(thrust_mapper src/thrust_mapper.cpp)

#target_link_libraries(thrust_mapper ${catkin_LIBRARIES} ${CERES_LIBRARIES})

