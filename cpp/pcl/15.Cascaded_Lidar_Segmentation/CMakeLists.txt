cmake_minimum_required(VERSION 2.8.12)
project(point)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fopenmp -lefence -g")
set(CMAKE_BUILD_TYPE "Release")
find_package(PCL 1.8 REQUIRED)
#find_package(CSF REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system iostreams)

find_package(OpenCV REQUIRED)

add_definitions( ${PCL_DEFINITIONS} )
include_directories(${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} )

add_executable(test1 slope_seg.cpp)
target_link_libraries(test1 ${EIGEN3_INCLUDE_DIR} ${PCL_LIBRARIES} ${Boost_INCLUDE_DIRS} ${OpenCV_LIBS})
