cmake_minimum_required(VERSION 3.19 FATAL_ERROR)
project(pcl_samples)

find_package(PCL 1.11 REQUIRED COMPONENTS common io)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message ([main] "PCL_DIR = ${PCL_DIR}")
message ([main] "PCL_FOUND = ${PCL_FOUND}")
message ([main] "PCL_INCLUDE_DIRS = ${PCL_INCLUDE_DIRS}")
#message ([main] "PCL_LIBRARIES = ${PCL_LIBRARIES}")
message ([main] "PCL_COMMON_LIBRARIES = ${PCL_COMMON_LIBRARIES}")
message ([main] "PCL_IO_LIBRARIES = ${PCL_IO_LIBRARIES}")

add_executable (pcl_sample pcl_sample.cpp)
target_link_libraries (pcl_sample ${PCL_LIBRARIES})