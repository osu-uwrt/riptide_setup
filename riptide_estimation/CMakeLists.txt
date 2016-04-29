cmake_minimum_required(VERSION 2.8.3)
project(riptide_estimation)

find_package(catkin REQUIRED COMPONENTS roscpp tf geometry_msgs sensor_msgs imu_3dm_gx4)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(imu_transform src/imu_transform.cpp)
add_dependencies(imu_transform imu_3dm_gx4_gencpp)
target_link_libraries(imu_transform ${catkin_LIBRARIES})

add_executable(imu_orientation src/imu_orientation.cpp)
add_dependencies(imu_orientation imu_3dm_gx4_gencpp)
target_link_libraries(imu_orientation ${catkin_LIBRARIES})
