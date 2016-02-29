cmake_minimum_required(VERSION 2.8.3)
project(riptide_hardware)

find_package(catkin REQUIRED COMPONENTS roscpp message_filters riptide_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(thrust_cal src/thrust_cal.cpp)
add_executable(serial_port src/serial_port.cpp)
#add_executable(serial_port2 src/serial_port2.cpp)

add_dependencies(thrust_cal riptide_msgs_gencpp)
add_dependencies(serial_port riptide_msgs_gencpp)
#add_dependencies(serial_port2 riptide_msgs_gencpp)

target_link_libraries(thrust_cal ${catkin_LIBRARIES})
target_link_libraries(serial_port ${catkin_LIBRARIES})
#target_link_libraries(serial_port2 ${catkin_LIBRARIES})
