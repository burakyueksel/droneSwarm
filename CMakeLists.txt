cmake_minimum_required(VERSION 3.10)
project(DroneSwarmSimulation)

set(CMAKE_CXX_STANDARD 11)

# Set the include directories
include_directories(include)

# Specify the build directory
set(BUILD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/build")

# Add all source files in the src directory
file(GLOB SOURCES src/*.cpp)

# Create the executable
add_executable(DroneSwarmSimulation ${SOURCES})

# Set the output directory
set_target_properties(DroneSwarmSimulation PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${BUILD_DIR}
)

# Clean rule
add_custom_target(clean_files
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${BUILD_DIR}
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/CMakeCache.txt
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/CMakeFiles
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/Makefile
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/cmake_install.cmake
)