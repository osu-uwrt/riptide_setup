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
    nortek_dvl
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

if(NOT _host_name MATCHES "jetson")

    # add_executable(acoustics src/acoustics.cpp)
    # target_link_libraries(acoustics ${catkin_LIBRARIES} okFrontPanel fftw3)
    # add_dependencies(acoustics ${catkin_EXPORTED_TARGETS})

    add_executable(depth_processor src/depth_processor.cpp)
    target_link_libraries(depth_processor ${catkin_LIBRARIES})
    add_dependencies(depth_processor ${catkin_EXPORTED_TARGETS})
endif()

add_executable(imu_processor src/imu_processor.cpp)
target_link_libraries(imu_processor ${catkin_LIBRARIES})
add_dependencies(imu_processor ${catkin_EXPORTED_TARGETS})

add_executable(dvl_processor src/dvl_processor.cpp)
target_link_libraries(dvl_processor ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(dvl_processor ${catkin_EXPORTED_TARGETS})
