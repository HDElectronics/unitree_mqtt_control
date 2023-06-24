# Specify the minimum CMake version
cmake_minimum_required(VERSION 3.0)

# Set the project name
project(unitree_mqtt_control)

# Enable verbose output
set(CMAKE_VERBOSE_MAKEFILE ON)

# Enable compiler warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# Set the C++ standard to use (optional)
set(CMAKE_CXX_STANDARD 14)

# Set all the libraries needed
set(EXTRA_LIBS -pthread libunitree_legged_sdk.a paho-mqttpp3 paho-mqtt3as)

# Add the Unitree Legged SDK library and include paths
set(UNITREE_LEGGED_SDK_PATH "/home/ibrahim/unitree_legged_sdk")
include_directories(${UNITREE_LEGGED_SDK_PATH}/include)
link_directories(${UNITREE_LEGGED_SDK_PATH}/lib/cpp/amd64)

# Set the output directory path
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "output")

# Add your source code files
set(SOURCES
    src/main.cpp
)

# Add an executable target
add_executable(unitree_mqtt ${SOURCES})

# Link with the Unitree Legged SDK library
target_link_libraries(unitree_mqtt ${EXTRA_LIBS})