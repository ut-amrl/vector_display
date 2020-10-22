PROJECT(vector_display)
CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0)

SET(arch ${})

MESSAGE(STATUS "Compiling Vector Display")

SET(CMAKE_CXX_FLAGS "-std=c++11 -fPIC -march=native -Wall -Werror -g -rdynamic")
ADD_DEFINITIONS(-Wfatal-errors)

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
SET(ROS_BUILD_STATIC_LIBS true)
SET(ROS_BUILD_SHARED_LIBS false)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

FIND_PACKAGE(Qt5 COMPONENTS Core Widgets Gui OpenGL REQUIRED)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)
FIND_PACKAGE(Lua51 REQUIRED)

SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTORCC ON)
SET(CMAKE_AUTOUIC ON)

rosbuild_genmsg()
rosbuild_gensrv()

INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/gui)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/vector_display)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/third_party)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/config_reader/include)
INCLUDE_DIRECTORIES(${Qt5Widgets_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${Qt5Gui_INCLUDE_DIRS})

MESSAGE(STATUS "Widgets: ${Qt5Widgets_INCLUDE_DIRS}")
MESSAGE(STATUS "Gui: ${Qt5Gui_INCLUDE_DIRS}")

SET(libs roslib roscpp pthread glut gflags
    Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL
     ${LUA_LIBRARIES} rt GL GLU
    glog X11 roscpp roslib amrl_shared_lib)

INCLUDE_DIRECTORIES(src/shared)
ADD_SUBDIRECTORY(src/shared)

SET(target vector_display)
ROSBUILD_ADD_EXECUTABLE(${target}
    src/gui/vector_display_main.cpp
    src/gui/vector_display_thread.cpp
    src/gui/vector_display.cpp
    src/gui/gltext.cpp
    src/vector_map/vector_map.cc
    ${VECTOR_DISPLAY_MOC_SRCS})
TARGET_LINK_LIBRARIES(${target} ${libs} ${QT_LIBRARIES})

ADD_EXECUTABLE(map_upgrade
    src/navigation_map/map_upgrade.cc)
TARGET_LINK_LIBRARIES(map_upgrade gflags glog)
