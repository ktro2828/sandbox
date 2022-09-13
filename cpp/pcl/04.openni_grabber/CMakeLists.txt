
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(openni_grabber)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (simple_openni_processor.h)
target_link_libraries (concatenate_clouds ${PCL_LIBRARIES})

add_executable (concatenate_clouds concatenate_clouds.cpp)
target_link_libraries (concatenate_clouds ${PCL_LIBRARIES})