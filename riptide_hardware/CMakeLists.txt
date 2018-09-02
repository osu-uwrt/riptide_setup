cmake_minimum_required(VERSION 2.8.3)
project(riptide_hardware)
add_compile_options(-std=c++11)

# NOTE: Most dependencies can be taken care of with $(catkin_EXPORTED_TARGETS).
# There is no need to add mention the specific dependency, such as
# <package_name>_gencpp, or <package_name>_gencfg

# NOTE: To include OpenCV, must do find_package(OpenCV REQUIRED), followed by
# include_directories(...)

find_package(catkin REQUIRED
    COMPONENTS
    roslint
    roscpp
    riptide_msgs
    geometry_msgs
    imu_3dm_gx4
    pointgrey_camera_driver
    message_filters
    tf
    message_generation
    sensor_msgs
    cv_bridge
)

add_service_files(FILES Pneumatics.srv)

generate_messages()

catkin_package(INCLUDE_DIRS include)
find_package(OpenCV REQUIRED) # To include, OpenCV, this MUST go BEFORE include_directories()
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} resources/acoustics/include)


roslint_cpp()

# include boost
find_package(Boost REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

cmake_host_system_information(RESULT _host_name QUERY HOSTNAME)
message("-- Computer name: " ${_host_name})

if(NOT _host_name MATCHES "jetson")

    add_executable(imu_logger src/imu_logger.cpp)
    target_link_libraries(imu_logger ${catkin_LIBRARIES})
    add_dependencies(imu_logger ${catkin_EXPORTED_TARGETS})

    add_executable(depth_processor src/depth_processor.cpp)
    target_link_libraries(depth_processor ${catkin_LIBRARIES})
    add_dependencies(depth_processor ${catkin_EXPORTED_TARGETS})
else()
    message("-- Skipping riptide code on jetson")
endif()

if(NOT _host_name MATCHES "riptide")

    add_executable(calibrate_camera src/calibrate_camera.cpp)
    target_link_libraries(calibrate_camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
    add_dependencies(calibrate_camera ${catkin_EXPORTED_TARGETS})

    add_executable(undistort_camera src/undistort_camera.cpp)
    target_link_libraries(undistort_camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
    add_dependencies(undistort_camera ${catkin_EXPORTED_TARGETS})
else()
    message("-- Skipping jetson code on riptide")
endif()

# ONLY compile this on riptide
if(_host_name MATCHES "riptide")

    file(GLOB ACOUSTICS_REFERENCES
        resources/acoustics/src/*.cpp
    )

    add_executable(acoustics src/acoustics.cpp ${ACOUSTICS_REFERENCES})
    target_link_libraries(acoustics ${catkin_LIBRARIES} okFrontPanel)
    add_dependencies(acoustics ${catkin_EXPORTED_TARGETS})

endif()

# Compile on all computers
add_executable(bag_to_csv src/bag_to_csv.cpp)
target_link_libraries(bag_to_csv ${catkin_LIBRARIES})
add_dependencies(bag_to_csv ${catkin_EXPORTED_TARGETS})

add_executable(imu_processor src/imu_processor.cpp)
target_link_libraries(imu_processor ${catkin_LIBRARIES})
add_dependencies(imu_processor ${catkin_EXPORTED_TARGETS})
