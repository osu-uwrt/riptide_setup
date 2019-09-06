# Install script for directory: /home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/imu_3dm_gx4

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/opt/ros/kinetic")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/imu_3dm_gx4/msg" TYPE FILE FILES
    "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/imu_3dm_gx4/msg/FilterOutput.msg"
    "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/imu_3dm_gx4/msg/MagFieldCF.msg"
    "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/imu_3dm_gx4/msg/MagFieldKF.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/imu_3dm_gx4/cmake" TYPE FILE FILES "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/build/imu_3dm_gx4/catkin_generated/installspace/imu_3dm_gx4-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/devel/include/imu_3dm_gx4")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/devel/share/roseus/ros/imu_3dm_gx4")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/devel/share/common-lisp/ros/imu_3dm_gx4")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/devel/share/gennodejs/ros/imu_3dm_gx4")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/devel/lib/python2.7/dist-packages/imu_3dm_gx4")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/devel/lib/python2.7/dist-packages/imu_3dm_gx4")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/build/imu_3dm_gx4/catkin_generated/installspace/imu_3dm_gx4.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/imu_3dm_gx4/cmake" TYPE FILE FILES "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/build/imu_3dm_gx4/catkin_generated/installspace/imu_3dm_gx4-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/imu_3dm_gx4/cmake" TYPE FILE FILES
    "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/build/imu_3dm_gx4/catkin_generated/installspace/imu_3dm_gx4Config.cmake"
    "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/build/imu_3dm_gx4/catkin_generated/installspace/imu_3dm_gx4Config-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/imu_3dm_gx4" TYPE FILE FILES "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/imu_3dm_gx4/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/imu_3dm_gx4" TYPE PROGRAM FILES "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/build/imu_3dm_gx4/catkin_generated/installspace/add_rule")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/imu_3dm_gx4" TYPE FILE FILES "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/imu_3dm_gx4/script/imu_3dm_gx4.rules")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/imu_3dm_gx4/imu_3dm_gx4" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/imu_3dm_gx4/imu_3dm_gx4")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/imu_3dm_gx4/imu_3dm_gx4"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/imu_3dm_gx4" TYPE EXECUTABLE FILES "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/devel/lib/imu_3dm_gx4/imu_3dm_gx4")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/imu_3dm_gx4/imu_3dm_gx4" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/imu_3dm_gx4/imu_3dm_gx4")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/imu_3dm_gx4/imu_3dm_gx4"
         OLD_RPATH "/opt/ros/kinetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/imu_3dm_gx4/imu_3dm_gx4")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/imu_3dm_gx4/launch" TYPE DIRECTORY FILES "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/imu_3dm_gx4/launch/")
endif()

