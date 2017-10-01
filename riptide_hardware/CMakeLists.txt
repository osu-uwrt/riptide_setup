cmake_minimum_required(VERSION 2.8.3)
project(riptide_hardware)

find_package(catkin REQUIRED
    COMPONENTS
    roslint
    roscpp
    riptide_msgs
)

catkin_package(INCLUDE_DIRS include)

roslint_cpp()

add_executable(imu_processor src/imu_processor.cpp)
target_link_libraries(imu_processor ${catkin_LIBRARIES})
