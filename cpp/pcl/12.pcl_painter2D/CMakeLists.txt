cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcl_painter2D)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl_painter2D pcl_painter2D.cpp)
target_link_libraries(pcl_painter2D ${PCL_LIBRARIES})
