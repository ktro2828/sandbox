cmake_minimum_required(VERSION 2.8)

project(kdtree)

set(CXX_FLAGS "W-all")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

add_executable(${PROJECT_NAME} src/test_kdtree.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)
