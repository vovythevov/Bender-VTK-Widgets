#
# Create a project with vtkWIdget for animation
#

cmake_minimum_required (VERSION 2.6) 
set (Project_Name BoneWidget)
project (${Project_Name})

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

option(BUILD_SHARED_LIBS "Build Shared libraries")

set (LIB_TYPE STATIC)
if (BUILD_SHARED_LIBS)
  # User wants to build Dynamic Libraries, so change the LIB_TYPE variable to CMake keyword 'SHARED'
  set (LIB_TYPE SHARED)
  set (VTK_BONE_WIDGET_BUILD_SHARED_LIBS 1)
endif (BUILD_SHARED_LIBS)

configure_file(${CMAKE_SOURCE_DIR}/CMake/vtkBoneWidgetConfigure.h.in
               ${CMAKE_BINARY_DIR}/CMake/vtkBoneWidgetConfigure.h)

enable_testing()

include_directories(
  ${CMAKE_SOURCE_DIR}/Code
  ${CMAKE_SOURCE_DIR}/Testing
  ${CMAKE_BINARY_DIR}/CMake)

add_subdirectory(${CMAKE_SOURCE_DIR}/Code)
add_subdirectory(${CMAKE_SOURCE_DIR}/Testing)
