# Install script for directory: /home/aaeon/PycharmProjects/Image-Capture-System-ICS-/ICS/src/ICS

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/aaeon/PycharmProjects/Image-Capture-System-ICS-/ICS/install")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/aaeon/PycharmProjects/Image-Capture-System-ICS-/ICS/build/ICS/catkin_generated/installspace/ICS.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ICS/cmake" TYPE FILE FILES
    "/home/aaeon/PycharmProjects/Image-Capture-System-ICS-/ICS/build/ICS/catkin_generated/installspace/ICSConfig.cmake"
    "/home/aaeon/PycharmProjects/Image-Capture-System-ICS-/ICS/build/ICS/catkin_generated/installspace/ICSConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ICS" TYPE FILE FILES "/home/aaeon/PycharmProjects/Image-Capture-System-ICS-/ICS/src/ICS/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ICS" TYPE PROGRAM FILES "/home/aaeon/PycharmProjects/Image-Capture-System-ICS-/ICS/build/ICS/catkin_generated/installspace/publisher_gps.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ICS" TYPE PROGRAM FILES "/home/aaeon/PycharmProjects/Image-Capture-System-ICS-/ICS/build/ICS/catkin_generated/installspace/subscriber_main.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ICS" TYPE PROGRAM FILES "/home/aaeon/PycharmProjects/Image-Capture-System-ICS-/ICS/build/ICS/catkin_generated/installspace/publisher_basler_camera.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ICS" TYPE PROGRAM FILES "/home/aaeon/PycharmProjects/Image-Capture-System-ICS-/ICS/build/ICS/catkin_generated/installspace/publisher_basler_camera2.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ICS" TYPE PROGRAM FILES "/home/aaeon/PycharmProjects/Image-Capture-System-ICS-/ICS/build/ICS/catkin_generated/installspace/gui.py")
endif()

