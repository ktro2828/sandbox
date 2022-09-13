cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(convert_pcd)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(obj2pcd src/obj2pcd.cpp)
target_link_libraries(obj2pcd ${PCL_LIBRARIES})

add_executable(pcd_read src/pcd_read.cpp)
target_link_libraries(pcd_read ${PCL_LIBRARIES})
