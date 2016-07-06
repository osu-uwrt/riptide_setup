cmake_minimum_required(VERSION 2.8.3)
project(riptide_hardware)

find_package(catkin REQUIRED COMPONENTS roscpp tf imu_3dm_gx4 geometry_msgs sensor_msgs riptide_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(thrust_cal src/thrust_cal.cpp)
add_dependencies(thrust_cal riptide_msgs_gencpp)
target_link_libraries(thrust_cal ${catkin_LIBRARIES})

add_executable(imu_transform src/imu_transform.cpp)
add_dependencies(imu_transform imu_3dm_gx4_gencpp)
target_link_libraries(imu_transform ${catkin_LIBRARIES})

add_executable(imu_orientation src/imu_orientation.cpp)
target_link_libraries(imu_orientation ${catkin_LIBRARIES})
