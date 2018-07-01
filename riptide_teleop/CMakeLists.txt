cmake_minimum_required(VERSION 2.8.3)
project(riptide_teleop)

find_package(catkin REQUIRED
    COMPONENTS
    joy
    riptide_msgs
    roscpp
    roslint
    tf
    sensor_msgs
    cv_bridge
)

catkin_package(INCLUDE_DIRS include)
find_package(OpenCV REQUIRED) # To include, OpenCV, this MUST go BEFORE include_directories()
include_directories(include ${catkin_INCLUDE_DIRS} $(OpenCV_INCLUDE_DIRS))
roslint_cpp()

add_executable(ps3_controller src/ps3_controller.cpp)
target_link_libraries(ps3_controller ${catkin_LIBRARIES})
add_dependencies(ps3_controller ${catkin_EXPORTED_TARGETS})
