cmake_minimum_required(VERSION 2.8.3)
project(riptide_navigation)

find_package(catkin REQUIRED COMPONENTS
  tf control_toolbox message_filters
  riptide_msgs sensor_msgs geometry_msgs
)
find_package(Ceres REQUIRED)

generate_dynamic_reconfigure_options(cfg/pid_orient.cfg cfg/pid.cfg)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

add_executable(thruster_controller src/thruster_controller.cpp)
target_link_libraries(thruster_controller ${catkin_LIBRARIES} ${CERES_LIBRARIES})
add_dependencies(thruster_controller riptide_msgs_gencpp)

add_executable(fake_data src/fake_data.cpp)
add_executable(fake_imu src/fake_imu.cpp)
target_link_libraries(fake_data ${catkin_LIBRARIES})
target_link_libraries(fake_imu  ${catkin_LIBRARIES})

add_executable(pid_linear src/pid_linear.cpp)
add_executable(pid_roll src/pid_roll.cpp)
add_executable(pid_pitch src/pid_pitch.cpp)
add_executable(pid_yaw src/pid_yaw.cpp)

add_dependencies(pid_linear riptide_navigation_gencfg)
add_dependencies(pid_roll riptide_navigation_gencfg)
add_dependencies(pid_pitch riptide_navigation_gencfg)
add_dependencies(pid_yaw riptide_navigation_gencfg)

target_link_libraries(pid_linear ${catkin_LIBRARIES})
target_link_libraries(pid_roll ${catkin_LIBRARIES})
target_link_libraries(pid_pitch ${catkin_LIBRARIES})
target_link_libraries(pid_yaw ${catkin_LIBRARIES})
