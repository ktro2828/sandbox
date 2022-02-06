cmake_minimum_required(VERSION 2.8)

project(render CXX)

set(CXX_FLAGS "w-all")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

# add_library(SOURCE src/vector.cpp)
# target_include_directories(SOURCE PUBLIC ${PROJECT_SOURCE_DIR}/include)

add_executable(${PROJECT_NAME} src/main.cpp)
# target_link_libraries(${PROJECT_NAME} SOURCE)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)
