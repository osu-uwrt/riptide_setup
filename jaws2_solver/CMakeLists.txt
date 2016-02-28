cmake_minimum_required(VERSION 2.8.3)
project(jaws2_solver)

find_package(catkin REQUIRED COMPONENTS
  jaws2_msgs
  roscpp
)
find_package(Ceres REQUIRED
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jaws2_solver
#  CATKIN_DEPENDS jaws2_msgs roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS} 
)

add_executable(thrust_mapper
  src/thrust_mapper.cpp
)

target_link_libraries(thrust_mapper
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)

