cmake_minimum_required(VERSION 2.8)
project(Thesis_DJI_PI)

set(CMAKE_VERBOSE_MAKEFILE OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g -O0")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/libs)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/TestCode/common/UserConfig.txt
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin/)
add_subdirectory(acel-base-master/src)
add_subdirectory(Onboard-SDK-3.5)

# Project program directory
add_subdirectory(TestCode)

set(SOURCE_FILES main.cpp)
add_executable(Thesis_DJI_PI ${SOURCE_FILES})
