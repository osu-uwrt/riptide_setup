# Install script for directory: /home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nortek_dvl/msg" TYPE FILE FILES
    "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl/msg/Dvl.msg"
    "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl/msg/DvlStatus.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nortek_dvl/cmake" TYPE FILE FILES "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/build/nortek_dvl/catkin_generated/installspace/nortek_dvl-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/devel/include/nortek_dvl")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/devel/share/roseus/ros/nortek_dvl")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/devel/share/common-lisp/ros/nortek_dvl")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/devel/share/gennodejs/ros/nortek_dvl")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/devel/lib/python2.7/dist-packages/nortek_dvl")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/devel/lib/python2.7/dist-packages/nortek_dvl")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/build/nortek_dvl/catkin_generated/installspace/nortek_dvl.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nortek_dvl/cmake" TYPE FILE FILES "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/build/nortek_dvl/catkin_generated/installspace/nortek_dvl-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nortek_dvl/cmake" TYPE FILE FILES
    "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/build/nortek_dvl/catkin_generated/installspace/nortek_dvlConfig.cmake"
    "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/build/nortek_dvl/catkin_generated/installspace/nortek_dvlConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nortek_dvl" TYPE FILE FILES "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nortek_dvl/nortek_dvl" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nortek_dvl/nortek_dvl")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nortek_dvl/nortek_dvl"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/nortek_dvl" TYPE EXECUTABLE FILES "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/devel/lib/nortek_dvl/nortek_dvl")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nortek_dvl/nortek_dvl" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nortek_dvl/nortek_dvl")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nortek_dvl/nortek_dvl"
         OLD_RPATH "/opt/ros/kinetic/lib:/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/build/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nortek_dvl/nortek_dvl")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/nortek_dvl/nortek_dvl" TYPE DIRECTORY FILES "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl/include/")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nortek_dvl/launch" TYPE DIRECTORY FILES "/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/src/nortek_dvl/launch/")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/pparekh/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/dependencies/build/nortek_dvl/tacopie/cmake_install.cmake")

endif()

