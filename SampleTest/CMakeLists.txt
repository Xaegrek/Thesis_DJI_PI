cmake_minimum_required(VERSION 2.8)
project(Thesis_DJI_PI)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g -O0")

## osdk-core location
set(ONBOARDSDK_SOURCE osdk-core)


## osdk-core headers
include_directories(${ONBOARDSDK_SOURCE}/api/inc)
include_directories(${ONBOARDSDK_SOURCE}/utility/inc)
include_directories(${ONBOARDSDK_SOURCE}/hal/inc)
include_directories(${ONBOARDSDK_SOURCE}/protocol/inc)
include_directories(${ONBOARDSDK_SOURCE}/platform/linux/inc)

## Project code files
include_directories(common)
FILE(GLOB SOURCE_FILES *.hpp *.cpp
        #${CMAKE_CURRENT_SOURCE_DIR}/common/dji_linux_environment.cpp
        #${CMAKE_CURRENT_SOURCE_DIR}/common/dji_linux_helpers.cpp
        )

if (${CMAKE_SYSTEM_NAME} MATCHES Linux)
    add_subdirectory(samples)
endif()

## target and linking
#set(SOURCE_FILES common/main.cpp)
add_executable(${PROJECT_NAME} ${SOURCE_FILES} Contrl_Fusion/main.cpp Contrl_Fusion/CTRLFusion.cpp Contrl_Fusion/CTRLFusion.h)
target_link_libraries(${PROJECT_NAME})
