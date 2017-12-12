cmake_minimum_required(VERSION 2.8.3)
project(riptide_behaviors)

find_package(catkin REQUIRED
    COMPONENTS
    geometry_msgs
    imu_3dm_gx4
    riptide_msgs
    roscpp
    roslint
    sensor_msgs
    tf
)

roslint_cpp()

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(go_straight src/go_straight.cpp)
target_link_libraries(go_straight ${catkin_LIBRARIES})
add_dependencies(go_straight riptide_msgs_gencpp)
