cmake_minimum_required(VERSION 2.8.3)
project(riptide_controllers)

find_package(catkin REQUIRED
    COMPONENTS
    geometry_msgs
    imu_3dm_gx4
    riptide_msgs
    roscpp
    roslint
    sensor_msgs
    tf
    control_toolbox
)

find_package(Ceres REQUIRED)

catkin_package(INCLUDE_DIRS include)

roslint_cpp()

include_directories(include ${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

add_executable(thruster_controller src/thruster_controller.cpp)
target_link_libraries(thruster_controller ${catkin_LIBRARIES} ${CERES_LIBRARIES})
add_dependencies(thruster_controller riptide_msgs_gencpp imu_3dm_gx4_gencpp)

add_executable(depth_controller src/depth_controller.cpp)
target_link_libraries(depth_controller ${catkin_LIBRARIES})
add_dependencies(depth_controller riptide_msgs_gencpp ${PROJECT_NAME}_gencfg)

add_executable(pwm_controller src/pwm_controller.cpp)
target_link_libraries(pwm_controller ${catkin_LIBRARIES})
add_dependencies(pwm_controller riptide_msgs_gencpp ${PROJECT_NAME}_gencfg)
