cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(visualizer_test)

find_package(PCL 1.4 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (visual visualizer.cpp)
target_link_libraries (visual ${PCL_LIBRARIES})
