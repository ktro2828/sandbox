cmake_minimum_required(VERSION 3.14)
project(opencv_test)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} include)

add_library(agc src/agc.cpp)

add_executable(main src/main.cpp)

target_link_libraries(main ${OpenCV_LIBS} agc)