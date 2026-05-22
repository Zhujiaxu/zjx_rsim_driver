# Install script for directory: /home/ubzjx/Proj/Intern_QCen/work/rsim-driver/third_party/PROJ/nad

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/proj" TYPE FILE FILES
    "/home/ubzjx/Proj/Intern_QCen/work/rsim-driver/third_party/PROJ/nad/epsg"
    "/home/ubzjx/Proj/Intern_QCen/work/rsim-driver/third_party/PROJ/nad/esri"
    "/home/ubzjx/Proj/Intern_QCen/work/rsim-driver/third_party/PROJ/nad/world"
    "/home/ubzjx/Proj/Intern_QCen/work/rsim-driver/third_party/PROJ/nad/esri.extra"
    "/home/ubzjx/Proj/Intern_QCen/work/rsim-driver/third_party/PROJ/nad/other.extra"
    "/home/ubzjx/Proj/Intern_QCen/work/rsim-driver/third_party/PROJ/nad/IGNF"
    "/home/ubzjx/Proj/Intern_QCen/work/rsim-driver/third_party/PROJ/nad/nad27"
    "/home/ubzjx/Proj/Intern_QCen/work/rsim-driver/third_party/PROJ/nad/GL27"
    "/home/ubzjx/Proj/Intern_QCen/work/rsim-driver/third_party/PROJ/nad/nad83"
    "/home/ubzjx/Proj/Intern_QCen/work/rsim-driver/third_party/PROJ/nad/nad.lst"
    "/home/ubzjx/Proj/Intern_QCen/work/rsim-driver/third_party/PROJ/nad/proj_def.dat"
    "/home/ubzjx/Proj/Intern_QCen/work/rsim-driver/third_party/PROJ/nad/CH"
    "/home/ubzjx/Proj/Intern_QCen/work/rsim-driver/third_party/PROJ/nad/ITRF2000"
    "/home/ubzjx/Proj/Intern_QCen/work/rsim-driver/third_party/PROJ/nad/ITRF2008"
    "/home/ubzjx/Proj/Intern_QCen/work/rsim-driver/third_party/PROJ/nad/ITRF2014"
    )
endif()

