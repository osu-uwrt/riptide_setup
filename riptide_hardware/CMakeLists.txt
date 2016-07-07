cmake_minimum_required(VERSION 2.8.3)
project(riptide_hardware)

find_package(catkin REQUIRED COMPONENTS roscpp tf imu_3dm_gx4 geometry_msgs sensor_msgs riptide_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(thrust_cal src/thrust_cal.cpp)
add_dependencies(thrust_cal riptide_msgs_gencpp)
target_link_libraries(thrust_cal ${catkin_LIBRARIES})

add_executable(imu_state src/imu_state.cpp)
add_dependencies(imu_state imu_3dm_gx4_gencpp)
target_link_libraries(imu_state ${catkin_LIBRARIES})
