cmake_minimum_required(VERSION 2.8 FATAL_ERROR)


include (FindPkgConfig)

find_package(gazebo REQUIRED)
add_definitions(-std=c++11)

if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${OGRE_INCLUDE_DIR} ${OGRE_Paging_INCLUDE_DIR})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(motion_blur SHARED motion_blur.cc)
target_link_libraries(motion_blur ${GAZEBO_libraries} CameraPlugin)

add_library(camera_move SHARED camera_move.cc)
target_link_libraries(camera_move ${GAZEBO_libraries})
