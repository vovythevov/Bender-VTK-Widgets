#
# Create a project with vtkWIdget for animation
#

cmake_minimum_required (VERSION 2.6) 
set (Project_Name BoneWidget)
project (${Project_Name})

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

enable_testing()

include_directories(
  ${CMAKE_SOURCE_DIR}/Code
  ${CMAKE_SOURCE_DIR}/Testing)

add_subdirectory(${CMAKE_SOURCE_DIR}/Code)
add_subdirectory(${CMAKE_SOURCE_DIR}/Testing)
