cmake_minimum_required(VERSION 3.10)
project(pcap_capture)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define the source files
set(SOURCES
    main.cpp
)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}
    "C:/Program Files (x86)/WpdPack_4_1_2/WpdPack/Include" # Include WinPcap headers
)

# Link directories
link_directories(
    "C:/Program Files (x86)/WpdPack_4_1_2/WpdPack/Lib" # Link WinPcap libraries
)

# Add the executable
add_executable(pcap_capture ${SOURCES})

# Link libraries
target_link_libraries(pcap_capture 
    ws2_32
    wpcap 
    Packet) # Link WinPcap libraries

# Link libraries (if any external libraries are required, add them here)
if (UNIX)
    target_link_libraries(pcap_capture pthread) # Link pthread on Linux
endif()

# Add definitions for platform-specific code
if (WIN32)
    add_definitions(-D_WIN32)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS) # Suppress warnings for unsafe functions
    add_compile_options(/wd4267) # Suppress warning C4267 (size_t to uint32_t conversion)
elseif (UNIX)
    add_definitions(-D__linux__)
endif()

# Print a message to indicate the build type
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")