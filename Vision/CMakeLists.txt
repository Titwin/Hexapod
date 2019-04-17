cmake_minimum_required(VERSION 3.7)

project(HexapodVision_v1)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

find_package(OpenCV REQUIRED)
find_package(aruco REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${aruco_INCLUDE_DIRS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin)

file( GLOB_RECURSE SRC src/*)

add_executable(HexapodVision_v1 ${SRC})

target_link_libraries(HexapodVision_v1 ${OpenCV_LIBS})
target_link_libraries(HexapodVision_v1 ${aruco_LIBS})