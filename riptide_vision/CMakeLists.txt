cmake_minimum_required(VERSION 2.8.3)
project(riptide_vision)

# NOTE: Most dependencies can be taken care of with $(catkin_EXPORTED_TARGETS).
# There is no need to add mention the specific dependency, such as
# <package_name>_gencpp, or <package_name>_gencfg

# NOTE: OpenCV by default is included by ROS

find_package(catkin REQUIRED
    COMPONENTS
    rospy
    roscpp
    roslint
    riptide_msgs
    sensor_msgs
    cv_bridge
)

catkin_package(INCLUDE_DIRS include)
include_directories(include ${catkin_INCLUDE_DIRS} $(OpenCV_INCLUDE_DIRS))

roslint_cpp()

add_executable(extract_video src/extract_video.cpp)
target_link_libraries(extract_video ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(extract_video ${catkin_EXPORTED_TARGETS})
