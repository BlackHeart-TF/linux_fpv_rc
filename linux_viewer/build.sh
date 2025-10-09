#!/bin/bash

# Build script for linux_viewer
set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}Building Linux Viewer...${NC}"

# Create build directory
mkdir -p build
cd build

# Configure with CMake
echo -e "${YELLOW}Configuring with CMake...${NC}"
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DENABLE_HARDWARE_DECODING=ON \
    -DENABLE_RT_OPTIMIZATION=ON \
    -DMAX_SERVO_COUNT=8

# Build
echo -e "${YELLOW}Building...${NC}"
make -j$(nproc)

echo -e "${GREEN}Build completed successfully!${NC}"
echo -e "${YELLOW}Executable: $(pwd)/linux_viewer${NC}"

# Check if we should install
if [ "$1" = "install" ]; then
    echo -e "${YELLOW}Installing...${NC}"
    sudo make install
    echo -e "${GREEN}Installation completed!${NC}"
fi
