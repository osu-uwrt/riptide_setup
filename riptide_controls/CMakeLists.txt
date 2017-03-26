cmake_minimum_required(VERSION 2.8.3)
project(riptide_controls)

find_package(catkin REQUIRED COMPONENTS
  roslint tf control_toolbox geometry_msgs nav_msgs sensor_msgs riptide_msgs imu_3dm_gx4
)
find_package(Ceres REQUIRED)

catkin_package(INCLUDE_DIRS include)

roslint_cpp()

roslint_python()

include_directories(include ${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

add_executable(attitude_controller src/attitude_controller.cpp)
target_link_libraries(attitude_controller ${catkin_LIBRARIES})
add_dependencies(attitude_controller riptide_msgs_gencpp imu_3dm_gx4_gencpp)

add_executable(position_controller src/position_controller.cpp)
target_link_libraries(position_controller ${catkin_LIBRARIES})
add_dependencies(position_controller riptide_msgs_gencpp)

add_executable(thruster_controller src/thruster_controller.cpp)
target_link_libraries(thruster_controller ${catkin_LIBRARIES} ${CERES_LIBRARIES})
add_dependencies(thruster_controller riptide_msgs_gencpp imu_3dm_gx4_gencpp)
