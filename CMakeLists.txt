cmake_minimum_required(VERSION 2.8)
project(Thesis_DJI_PI)

set(CMAKE_VERBOSE_MAKEFILE OFF)

#set(DJIOSDK 0)
#set(DJIOSDK_MINOR_VERSION 1)
#set(DJIOSDK_PATCH_VERSION 0)
#set(DJIOSDK_VERSION
#        ${DJIOSDK_MAJOR_VERSION}.${DJIOSDK_MINOR_VERSION}.${DJIOSDK_PATCH_VERSION})
#
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/libs)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/libs)

#file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/TestCode/common/UserConfig.txt
#        DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin/)
file(WRITE UserConfig.txt "app_id : 1051117
app_key : 8de7ffac1a632b8b6e21df0026532e12e621b2ab4193c7dc65265a8f061655e5
device : /dev/serial0
baudrate : 230400")
add_subdirectory(Onboard-SDK-3.5)

# Project program directory
add_subdirectory(TestCode)

set(SOURCE_FILES main.cpp)
add_executable(Thesis_DJI_PI ${SOURCE_FILES})