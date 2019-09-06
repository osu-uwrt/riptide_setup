# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "nortek_dvl: 2 messages, 0 services")

set(MSG_I_FLAGS "-Inortek_dvl:/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl/msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(nortek_dvl_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl/msg/Dvl.msg" NAME_WE)
add_custom_target(_nortek_dvl_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "nortek_dvl" "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl/msg/Dvl.msg" "std_msgs/Header:geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl/msg/DvlStatus.msg" NAME_WE)
add_custom_target(_nortek_dvl_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "nortek_dvl" "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl/msg/DvlStatus.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(nortek_dvl
  "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl/msg/DvlStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nortek_dvl
)
_generate_msg_cpp(nortek_dvl
  "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl/msg/Dvl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nortek_dvl
)

### Generating Services

### Generating Module File
_generate_module_cpp(nortek_dvl
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nortek_dvl
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(nortek_dvl_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(nortek_dvl_generate_messages nortek_dvl_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl/msg/Dvl.msg" NAME_WE)
add_dependencies(nortek_dvl_generate_messages_cpp _nortek_dvl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl/msg/DvlStatus.msg" NAME_WE)
add_dependencies(nortek_dvl_generate_messages_cpp _nortek_dvl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nortek_dvl_gencpp)
add_dependencies(nortek_dvl_gencpp nortek_dvl_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nortek_dvl_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(nortek_dvl
  "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl/msg/DvlStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/nortek_dvl
)
_generate_msg_eus(nortek_dvl
  "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl/msg/Dvl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/nortek_dvl
)

### Generating Services

### Generating Module File
_generate_module_eus(nortek_dvl
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/nortek_dvl
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(nortek_dvl_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(nortek_dvl_generate_messages nortek_dvl_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl/msg/Dvl.msg" NAME_WE)
add_dependencies(nortek_dvl_generate_messages_eus _nortek_dvl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl/msg/DvlStatus.msg" NAME_WE)
add_dependencies(nortek_dvl_generate_messages_eus _nortek_dvl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nortek_dvl_geneus)
add_dependencies(nortek_dvl_geneus nortek_dvl_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nortek_dvl_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(nortek_dvl
  "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl/msg/DvlStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nortek_dvl
)
_generate_msg_lisp(nortek_dvl
  "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl/msg/Dvl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nortek_dvl
)

### Generating Services

### Generating Module File
_generate_module_lisp(nortek_dvl
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nortek_dvl
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(nortek_dvl_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(nortek_dvl_generate_messages nortek_dvl_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl/msg/Dvl.msg" NAME_WE)
add_dependencies(nortek_dvl_generate_messages_lisp _nortek_dvl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl/msg/DvlStatus.msg" NAME_WE)
add_dependencies(nortek_dvl_generate_messages_lisp _nortek_dvl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nortek_dvl_genlisp)
add_dependencies(nortek_dvl_genlisp nortek_dvl_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nortek_dvl_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(nortek_dvl
  "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl/msg/DvlStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/nortek_dvl
)
_generate_msg_nodejs(nortek_dvl
  "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl/msg/Dvl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/nortek_dvl
)

### Generating Services

### Generating Module File
_generate_module_nodejs(nortek_dvl
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/nortek_dvl
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(nortek_dvl_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(nortek_dvl_generate_messages nortek_dvl_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl/msg/Dvl.msg" NAME_WE)
add_dependencies(nortek_dvl_generate_messages_nodejs _nortek_dvl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl/msg/DvlStatus.msg" NAME_WE)
add_dependencies(nortek_dvl_generate_messages_nodejs _nortek_dvl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nortek_dvl_gennodejs)
add_dependencies(nortek_dvl_gennodejs nortek_dvl_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nortek_dvl_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(nortek_dvl
  "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl/msg/DvlStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nortek_dvl
)
_generate_msg_py(nortek_dvl
  "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl/msg/Dvl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nortek_dvl
)

### Generating Services

### Generating Module File
_generate_module_py(nortek_dvl
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nortek_dvl
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(nortek_dvl_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(nortek_dvl_generate_messages nortek_dvl_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl/msg/Dvl.msg" NAME_WE)
add_dependencies(nortek_dvl_generate_messages_py _nortek_dvl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl/msg/DvlStatus.msg" NAME_WE)
add_dependencies(nortek_dvl_generate_messages_py _nortek_dvl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nortek_dvl_genpy)
add_dependencies(nortek_dvl_genpy nortek_dvl_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nortek_dvl_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nortek_dvl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nortek_dvl
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(nortek_dvl_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(nortek_dvl_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/nortek_dvl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/nortek_dvl
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(nortek_dvl_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(nortek_dvl_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nortek_dvl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nortek_dvl
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(nortek_dvl_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(nortek_dvl_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/nortek_dvl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/nortek_dvl
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(nortek_dvl_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(nortek_dvl_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nortek_dvl)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nortek_dvl\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nortek_dvl
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(nortek_dvl_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(nortek_dvl_generate_messages_py std_msgs_generate_messages_py)
endif()
