cmake_minimum_required(VERSION 2.8.3)
project(riptide_vision)

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
find_package(OpenCV REQUIRED)

add_executable(extract_video src/extract_video.cpp)
target_link_libraries(extract_video ${catkin_LIBRARIES})
add_dependencies(extract_video ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
