cmake_minimum_required(VERSION 3.0.2)

project(mapf_test)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(LaChaCom REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

include_directories("include")
include_directories(${OPENCV_INCLUDE_DIRS})

link_directories(${OpenCV_LIBRARY_DIRS})


file(GLOB SOURCES "src/*.cpp")

add_executable(mapf_test ${SOURCES})
target_link_libraries(mapf_test 
    LaChaCom::LaChaCom ${YAML_CPP_LIBRARIES} ${OpenCV_LIBRARIES}
)
