cmake_minimum_required(VERSION 2.8.3)
project(riptide_pid)

find_package(catkin REQUIRED COMPONENTS roscpp rospy control_toolbox dynamic_reconfigure message_filters)

generate_dynamic_reconfigure_options(cfg/pid_orient.cfg cfg/pid.cfg)

add_executable(fake_data src/fake_data.cpp)
add_executable(fake_imu src/fake_imu.cpp)

add_executable(pid_linear src/pid_linear.cpp)
#add_executable(pid_roll src/pid_roll.cpp)

add_dependencies(pid_linear riptide_pid_gencfg)
#add_dependencies(pid_roll riptide_pid_gencfg)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

target_link_libraries(fake_data ${catkin_LIBRARIES})
target_link_libraries(fake_imu  ${catkin_LIBRARIES})

target_link_libraries(pid_linear ${catkin_LIBRARIES})
#target_link_libraries(pid_roll ${catkin_LIBRARIES})
