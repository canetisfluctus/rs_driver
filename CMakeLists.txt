cmake_minimum_required(VERSION 3.10)

# Project name
project(LidarClient)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories for rs_driver
include_directories(
    ${CMAKE_SOURCE_DIR}/../rs_driver
    "C:/Program Files (x86)/WpdPack_4_1_2/WpdPack/Include" # Include WinPcap headers
    )

# Add the executable
add_executable(LidarClient main.cpp)

# Link directories for rs_driver
link_directories(
    "C:/Program Files (x86)/WpdPack_4_1_2/WpdPack/Lib" # Link WinPcap libraries
    )

# Link required libraries
target_link_libraries(LidarClient 
    rs_driver 
    ws2_32
    wpcap
    Packet)