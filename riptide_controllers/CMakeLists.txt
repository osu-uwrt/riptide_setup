cmake_minimum_required(VERSION 2.8.3)
project(riptide_controllers)
add_compile_options(-std=c++11)

# NOTE: Most dependencies can be taken care of with $(catkin_EXPORTED_TARGETS).
# There is no need to add mention the specific dependency, such as
# <package_name>_gencpp, or <package_name>_gencfg

# NOTE: To include OpenCV, must do find_package(OpenCV REQUIRED), followed by
# include_directories(...)

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

find_package(Eigen3 3.3 REQUIRED)

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
  {Eigen3_INCLUDE_DIRS}
)

cmake_host_system_information(RESULT _host_name QUERY HOSTNAME)

#if(NOT _host_name MATCHES "riptide")
add_executable(thruster_controller src/thruster_controller.cpp)
target_link_libraries(thruster_controller ${catkin_LIBRARIES} ${CERES_LIBRARIES} yaml-cpp)
add_dependencies(thruster_controller ${catkin_EXPORTED_TARGETS})
#endif()
#add_dependencies(thruster_controller riptide_controllers_gencfg)

add_executable(depth_controller src/depth_controller.cpp)
target_link_libraries(depth_controller ${catkin_LIBRARIES})
add_dependencies(depth_controller ${catkin_EXPORTED_TARGETS})

add_executable(alignment_controller src/alignment_controller.cpp)
target_link_libraries(alignment_controller ${catkin_LIBRARIES})
add_dependencies(alignment_controller ${catkin_EXPORTED_TARGETS})

add_executable(pwm_controller src/pwm_controller.cpp)
target_link_libraries(pwm_controller ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(pwm_controller ${catkin_EXPORTED_TARGETS})

add_executable(attitude_controller src/attitude_controller.cpp)
target_link_libraries(attitude_controller ${catkin_LIBRARIES})
add_dependencies(attitude_controller ${catkin_EXPORTED_TARGETS})

add_executable(command_combinator src/command_combinator.cpp)
target_link_libraries(command_combinator ${catkin_LIBRARIES})
add_dependencies(command_combinator ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/controller_tests.test)
endif()
