cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

# rosbuild setup
rosbuild_init()
set(ROS_BUILD_TYPE RelWithDebInfo)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Add our custom find macros to cmake
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake;${CMAKE_MODULE_PATH};")

# Pkgconfig
find_package(PkgConfig REQUIRED)

# Ogre
pkg_check_modules(OGRE OGRE)
include_directories(SYSTEM ${OGRE_INCLUDE_DIRS} )
link_directories( ${OGRE_LIBRARY_DIRS} )

# Find Ogre Plugin path
execute_process(COMMAND 
pkg-config --variable=plugindir OGRE 
OUTPUT_VARIABLE OGRE_PLUGIN_PATH
OUTPUT_STRIP_TRAILING_WHITESPACE
)
message(STATUS OGRE_PLUGIN_PATH=${OGRE_PLUGIN_PATH})
set(ENV_CONFIG_FILE ${PROJECT_SOURCE_DIR}/build/env_config.cpp)
configure_file(src/env_config.cpp.in ${ENV_CONFIG_FILE} @ONLY)

# QT
find_package(Qt4 REQUIRED)

set(QT_USE_QTXML 1)
include(${QT_USE_FILE})
include_directories( SYSTEM ${QT_INCLUDE_DIR})

# Festival
find_package(Festival REQUIRED)
include_directories( SYSTEM ${Festival_INCLUDE_DIR} )

# EST
find_package(EST REQUIRED)
include_directories( SYSTEM ${EST_INCLUDE_DIR} )


# Source files
set(RobotFace_SRC
  src/main.cpp
  src/MainWindow.cpp
  src/TalkingHead.cpp
  src/TextOutDisplay.cpp
  src/QtRosNode.cpp
  src/FestivalGenerator.cpp
  ${ENV_CONFIG_FILE}
)

# add headers of widgets that declare Q_OBJECT here
qt4_wrap_cpp(MOC_FILES
  include/MainWindow.h
  include/TalkingHead.h
  include/TextOutDisplay.h
  include/QtRosNode.h
  include/FestivalGenerator.h
)


rosbuild_add_executable(RobotFace 
${RobotFace_SRC} 
${MOC_FILES} )

target_link_libraries(RobotFace 
${OGRE_LIBRARIES}
${QT_LIBRARIES}
${Festival_LIBRARIES}
${EST_LIBRARIES} )


rosbuild_add_executable(FestivalSynthesizer 
src/FestivalSynthesizer.cpp )

target_link_libraries(FestivalSynthesizer 
${USEPKG_LIBRARIES} 
${Festival_LIBRARIES}
${EST_LIBRARIES}
pulse-simple )

#
# check code stlye
#

if(EXISTS "/usr/bin/python2")
    set(PYTHON_VER "python2")
else()
    set(PYTHON_VER "python")
endif()

set(STYLE_IGNORE "-whitespace/parens, -whitespace/braces, -build, -legal, -readability, -runtime, -build/header_guard")
set(STYLE_SOURCES
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/includes
)

foreach(STYLE_SOURCE ${STYLE_SOURCES})
    file(GLOB_RECURSE GLOB_CPP ${STYLE_SOURCE}/*.cpp)
    file(GLOB_RECURSE GLOB_H ${STYLE_SOURCE}/*.h)
    set(GLOB_STYLE ${GLOB_STYLE} ${GLOB_CPP} ${GLOB_H})
endforeach()

add_custom_target(style
    COMMAND ${PYTHON_VER} cpplint.py --filter=${STYLE_IGNORE} --counting=detailed ${GLOB_STYLE}
)

