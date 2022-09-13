cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(lidar_segmentation)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(CVC src/CVC.cpp)
target_link_libraries(CVC ${PCL_LIBRARIES})

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable (
        environment
        src/environment.cpp
        src/render/render.cpp
        src/processPointClouds.cpp)

target_link_libraries (environment ${PCL_LIBRARIES})
