cmake_minimum_required(VERSION 3.14)
project(boost-python-test)

find_package(Python3 COMPONENTS Development)
find_package(Boost COMPONENTS python3 REQUIRED)

# DEBUG
message("## Boost_LIBRARIES:  ${Boost_LIBRARIES}")  
message("## Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
message("## Python3_INCLUDE_DIRS: ${Python3_INCLUDE_DIRS}") 
message("## Python3_LIBRARIES :${Python3_LIBRARIES}")

add_library(sample SHARED basic.cpp)
target_include_directories(sample PRIVATE ${Python3_INCLUDE_DIRS})
target_link_libraries(sample PRIVATE ${Boost_LIBRARIES} ${Python3_LIBRARIES})
set_target_properties(sample PROPERTIES PREFIX "") # build .so without "lib"