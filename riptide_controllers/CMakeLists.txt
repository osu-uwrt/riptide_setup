cmake_minimum_required(VERSION 2.8.3)
project(riptide_controllers)

# NOTE: Most dependencies can be taken care of with $(catkin_EXPORTED_TARGETS).
# There is no need to add mention the specific dependency, such as
# <package_name>_gencpp, or <package_name>_gencfg

# NOTE: OpenCV by default is included by ROS

find_package(catkin REQUIRED
    COMPONENTS
    dynamic_reconfigure
    geometry_msgs
    imu_3dm_gx4
    riptide_msgs
    roscpp
    roslint
    sensor_msgs
    riptide_msgs
    tf
    control_toolbox
)

find_package(Ceres REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/VehicleProperties.cfg
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    dynamic_reconfigure
)

roslint_cpp()

include_directories(include ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

add_executable(thruster_controller src/thruster_controller.cpp)
target_link_libraries(thruster_controller ${catkin_LIBRARIES} ${CERES_LIBRARIES})
add_dependencies(thruster_controller ${catkin_EXPORTED_TARGETS})
#add_dependencies(thruster_controller riptide_controllers_gencfg)

add_executable(depth_controller src/depth_controller.cpp)
target_link_libraries(depth_controller ${catkin_LIBRARIES})
add_dependencies(depth_controller ${catkin_EXPORTED_TARGETS})

add_executable(alignment_controller src/alignment_controller.cpp)
target_link_libraries(alignment_controller ${catkin_LIBRARIES})
add_dependencies(alignment_controller ${catkin_EXPORTED_TARGETS})

add_executable(pwm_controller src/pwm_controller.cpp)
target_link_libraries(pwm_controller ${catkin_LIBRARIES})
add_dependencies(pwm_controller ${catkin_EXPORTED_TARGETS})

add_executable(attitude_controller src/attitude_controller.cpp)
target_link_libraries(attitude_controller ${catkin_LIBRARIES})
add_dependencies(attitude_controller ${catkin_EXPORTED_TARGETS})

add_executable(command_combinator src/command_combinator.cpp)
target_link_libraries(command_combinator ${catkin_LIBRARIES})
add_dependencies(command_combinator ${catkin_EXPORTED_TARGETS})
