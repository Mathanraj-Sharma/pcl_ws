cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(basic_io)

find_package(PCL 1.8 REQUIRED common io visualization)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcd_write write.cpp)
target_link_libraries (pcd_write ${PCL_LIBRARIES})

add_executable (pcd_read read.cpp)
target_link_libraries (pcd_read ${PCL_LIBRARIES})

add_executable (pcd_concat concat.cpp)
target_link_libraries (pcd_concat ${PCL_LIBRARIES})