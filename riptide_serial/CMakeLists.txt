cmake_minimum_required(VERSION 2.8.3)
project(riptide_serial)

find_package(catkin REQUIRED COMPONENTS message_filters riptide_msgs)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(serial_port src/serial_port.cpp)

add_dependencies(serial_port riptide_msgs_gencpp)

target_link_libraries(serial_port ${catkin_LIBRARIES})
