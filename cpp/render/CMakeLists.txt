cmake_minimum_required(VERSION 2.8)

project(render CXX)

set(CXX_FLAGS "w-all")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

add_executable(test src/test.cpp)
# target_link_libraries(${PROJECT_NAME} SOURCE)
target_include_directories(test PUBLIC ${PROJECT_SOURCE_DIR}/include)
