cmake_minimum_required(VERSION 2.8)
set(CMAKE_VERBOSE_MAKEFILE 1) # to see all make process

project(3d_registration)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIOS})

add_executable(icp_after_feature_registration src/icp_after_feature_registration.cpp)
target_link_libraries(icp_after_feature_registration src/icp_after_feature_registration.cpp)
