cmake_minimum_required(VERSION 3.25.1)
project(viewer)
add_executable(viewer viewer.cpp)
find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(viewer ${PCL_LIBRARIES})

message(${CMAKE_CXX_COMPILER})
