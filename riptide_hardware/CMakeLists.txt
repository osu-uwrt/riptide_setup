cmake_minimum_required(VERSION 2.8.3)
project(riptide_hardware)

find_package(catkin REQUIRED
    COMPONENTS
    roslint
    roscpp
    riptide_msgs
    geometry_msgs
    imu_3dm_gx4
    pointgrey_camera_driver
)

catkin_package(INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS})

roslint_cpp()

add_executable(imu_processor src/imu_processor.cpp)
target_link_libraries(imu_processor ${catkin_LIBRARIES})
add_dependencies(imu_processor riptide_msgs_gencpp ${PROJECT_NAME}_gencfg)

add_executable(depth_processor src/imu_processor.cpp)
target_link_libraries(depth_processor ${catkin_LIBRARIES})
add_dependencies(depth_processor riptide_msgs_gencpp ${PROJECT_NAME}_gencfg)
