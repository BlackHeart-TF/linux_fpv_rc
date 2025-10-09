cmake_minimum_required(VERSION 3.16)
project(linux_viewer VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Configuration options
option(ENABLE_HARDWARE_DECODING "Enable hardware H.264 decoding" ON)
option(ENABLE_RT_OPTIMIZATION "Enable real-time kernel optimizations" ON)
set(MAX_SERVO_COUNT 8 CACHE STRING "Maximum number of servos to support")

# Compiler flags for RT optimization
if(ENABLE_RT_OPTIMIZATION)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -mtune=native")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math -funroll-loops")
    add_definitions(-DENABLE_RT_OPTIMIZATION)
endif()

# Add servo count definition
add_definitions(-DMAX_SERVO_COUNT=${MAX_SERVO_COUNT})

# Find required packages
find_package(PkgConfig REQUIRED)

# FFmpeg for H.264 decoding
pkg_check_modules(FFMPEG REQUIRED
    libavcodec
    libavformat
    libavutil
    libswscale
)

# SDL2 for gamepad input and video display
pkg_check_modules(SDL2 REQUIRED sdl2)
pkg_check_modules(SDL2_IMAGE REQUIRED SDL2_image)

# OpenGL for video rendering (optional)
find_package(OpenGL)
if(OpenGL_FOUND)
    add_definitions(-DHAVE_OPENGL)
endif()

# Thread support
find_package(Threads REQUIRED)

# Include directories
include_directories(include)
include_directories(${FFMPEG_INCLUDE_DIRS})
include_directories(${SDL2_INCLUDE_DIRS})

# Source files
set(SOURCES
    src/main.cpp
    src/decoder/h264_decoder.cpp
    src/gamepad/gamepad_manager.cpp
    src/network/udp_receiver.cpp
    src/network/command_sender.cpp
    src/config/runtime_config.cpp
    src/utils/logger.cpp
    src/utils/performance_monitor.cpp
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME}
    ${FFMPEG_LIBRARIES}
    ${SDL2_LIBRARIES}
    Threads::Threads
)

if(OpenGL_FOUND)
    target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES})
endif()

# Compiler definitions
target_compile_definitions(${PROJECT_NAME} PRIVATE
    $<$<BOOL:${ENABLE_HARDWARE_DECODING}>:ENABLE_HARDWARE_DECODING>
)

# Installation
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
install(FILES config/viewer_config.json DESTINATION /etc/linux_viewer/)

# Create systemd service file
configure_file(
    ${CMAKE_SOURCE_DIR}/systemd/linux_viewer.service.in
    ${CMAKE_BINARY_DIR}/linux_viewer.service
    @ONLY
)

install(FILES ${CMAKE_BINARY_DIR}/linux_viewer.service
    DESTINATION /etc/systemd/system/
)
