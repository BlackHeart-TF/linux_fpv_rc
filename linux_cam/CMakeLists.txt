cmake_minimum_required(VERSION 3.16)
project(linux_cam VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Configuration options
option(ENABLE_HARDWARE_ENCODING "Enable hardware H.264 encoding" ON)
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

# V4L2 for camera interface
pkg_check_modules(V4L2 REQUIRED libv4l2)

# FFmpeg for H.264 encoding
pkg_check_modules(FFMPEG REQUIRED
    libavcodec
    libavformat
    libavutil
    libswscale
)

# Thread support
find_package(Threads REQUIRED)

# Include directories
include_directories(include)
include_directories(${V4L2_INCLUDE_DIRS})
include_directories(${FFMPEG_INCLUDE_DIRS})

# Source files
set(SOURCES
    src/main.cpp
    src/camera/csi_camera.cpp
    src/encoder/h264_encoder.cpp
    src/network/udp_streamer.cpp
    src/network/command_receiver.cpp
    src/servo/i2c_servo_controller.cpp
    src/config/runtime_config.cpp
    src/utils/logger.cpp
    src/utils/performance_monitor.cpp
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME}
    ${V4L2_LIBRARIES}
    ${FFMPEG_LIBRARIES}
    Threads::Threads
    i2c
)

# Compiler definitions
target_compile_definitions(${PROJECT_NAME} PRIVATE
    $<$<BOOL:${ENABLE_HARDWARE_ENCODING}>:ENABLE_HARDWARE_ENCODING>
)

# Installation
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
install(FILES config/camera_config.json DESTINATION /etc/linux_cam/)

# Create systemd service file
configure_file(
    ${CMAKE_SOURCE_DIR}/systemd/linux_cam.service.in
    ${CMAKE_BINARY_DIR}/linux_cam.service
    @ONLY
)

install(FILES ${CMAKE_BINARY_DIR}/linux_cam.service
    DESTINATION /etc/systemd/system/
)
