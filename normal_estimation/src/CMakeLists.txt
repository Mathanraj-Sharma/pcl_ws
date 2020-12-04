cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(normal_estimation)

find_package(PCL 1.8 REQUIRED common io visualization)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (all_points_normal all_points.cpp)
target_link_libraries (all_points_normal ${PCL_LIBRARIES})

add_executable (subset_points_normal subset_points.cpp)
target_link_libraries (subset_points_normal ${PCL_LIBRARIES})
