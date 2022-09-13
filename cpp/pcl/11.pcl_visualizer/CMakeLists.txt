
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl_visualizer)

find_package(PCL 1.8 REQUIRED)

SET(CMAKE_C_COMPILER mpicc)

SET(CMAKE_CXX_COMPILER mpicxx)

include_directories(MPI_INCLUDE_PATH)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_visualizer pcl_visualizer.cpp)
target_link_libraries (pcl_visualizer ${PCL_LIBRARIES} ${MPI_LIBRARIES})