cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(basic_structure)

find_package(PCL 1.8 REQUIRED common io visualization)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (basic_structure basic_structure.cpp)
target_link_libraries (basic_structure ${PCL_LIBRARIES})
