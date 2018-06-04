cmake_minimum_required(VERSION 2.8.3)
project(riptide_teleop)

find_package(catkin REQUIRED
    COMPONENTS
    joy
    riptide_msgs
    roscpp
    roslint
    tf
)

catkin_package()

roslint_cpp()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ps3_controller src/ps3_controller.cpp)
target_link_libraries(ps3_controller ${catkin_LIBRARIES})
add_dependencies(ps3_controller ${catkin_EXPORTED_TARGETS})
