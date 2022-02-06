cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcdio)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(read read.cpp)
target_link_libraries(read ${PCL_LIBRARIES})

add_executable(write write.cpp)
target_link_libraries(write ${PCL_LIBRARIES})
