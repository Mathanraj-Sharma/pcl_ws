cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(cloud_viewer)

find_package(PCL 1.8 REQUIRED common io visualization)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cloud_viewer cloud_viewer.cpp)
target_link_libraries (cloud_viewer ${PCL_LIBRARIES})
