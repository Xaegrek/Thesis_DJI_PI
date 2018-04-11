cmake_minimum_required(VERSION 2.6.2)
project(spdlog)

# Include path
include_directories(.)

# Add libraries
add_library(spdlog INTERFACE)
target_include_directories(spdlog INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	$<INSTALL_INTERFACE:include>)