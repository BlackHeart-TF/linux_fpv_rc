#!/bin/bash

# Linux Camera RT - Build Script
# Builds the embedded camera application with RT kernel support

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
BUILD_TYPE="Release"
SERVO_COUNT=8
ENABLE_HW_ENCODING=ON
ENABLE_RT_OPT=ON
INSTALL_PREFIX="/usr/local"
BUILD_DIR="build"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --debug)
            BUILD_TYPE="Debug"
            shift
            ;;
        --servo-count)
            SERVO_COUNT="$2"
            shift 2
            ;;
        --no-hw-encoding)
            ENABLE_HW_ENCODING=OFF
            shift
            ;;
        --no-rt-opt)
            ENABLE_RT_OPT=OFF
            shift
            ;;
        --install-prefix)
            INSTALL_PREFIX="$2"
            shift 2
            ;;
        --clean)
            echo -e "${YELLOW}Cleaning build directory...${NC}"
            rm -rf "$BUILD_DIR"
            shift
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo "Options:"
            echo "  --debug              Build in debug mode"
            echo "  --servo-count N      Set maximum servo count (default: 8)"
            echo "  --no-hw-encoding     Disable hardware encoding support"
            echo "  --no-rt-opt          Disable RT kernel optimizations"
            echo "  --install-prefix P   Set installation prefix (default: /usr/local)"
            echo "  --clean              Clean build directory"
            echo "  --help               Show this help"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

echo -e "${BLUE}=== Linux Camera RT Build Script ===${NC}"
echo -e "${BLUE}Build Type:${NC} $BUILD_TYPE"
echo -e "${BLUE}Servo Count:${NC} $SERVO_COUNT"
echo -e "${BLUE}Hardware Encoding:${NC} $ENABLE_HW_ENCODING"
echo -e "${BLUE}RT Optimizations:${NC} $ENABLE_RT_OPT"
echo -e "${BLUE}Install Prefix:${NC} $INSTALL_PREFIX"
echo

# Check dependencies
echo -e "${YELLOW}Checking dependencies...${NC}"

check_dependency() {
    if command -v "$1" &> /dev/null; then
        echo -e "${GREEN}✓${NC} $1 found"
    else
        echo -e "${RED}✗${NC} $1 not found"
        return 1
    fi
}

DEPS_OK=true

check_dependency "cmake" || DEPS_OK=false
check_dependency "make" || DEPS_OK=false
check_dependency "g++" || DEPS_OK=false
check_dependency "pkg-config" || DEPS_OK=false

# Check for required libraries
echo -e "${YELLOW}Checking libraries...${NC}"

check_library() {
    if pkg-config --exists "$1" 2>/dev/null; then
        echo -e "${GREEN}✓${NC} $1 found"
    else
        echo -e "${RED}✗${NC} $1 not found"
        return 1
    fi
}

check_library "libv4l2" || DEPS_OK=false
check_library "libavcodec" || DEPS_OK=false
check_library "libavformat" || DEPS_OK=false
check_library "libavutil" || DEPS_OK=false
check_library "libswscale" || DEPS_OK=false

# Check for I2C headers
if [ -f "/usr/include/linux/i2c-dev.h" ] || [ -f "/usr/include/i2c/smbus.h" ]; then
    echo -e "${GREEN}✓${NC} I2C development headers found"
else
    echo -e "${RED}✗${NC} I2C development headers not found"
    DEPS_OK=false
fi

if [ "$DEPS_OK" = false ]; then
    echo -e "${RED}Missing dependencies. Please install required packages:${NC}"
    echo "Debian/Ubuntu:"
    echo "  sudo apt install build-essential cmake libv4l-dev libavcodec-dev libavformat-dev libavutil-dev libswscale-dev libi2c-dev pkg-config"
    echo "Fedora/RHEL:"
    echo "  sudo dnf install gcc-c++ cmake v4l-utils-devel ffmpeg-devel i2c-tools-devel pkgconfig"
    echo "Arch Linux:"
    echo "  sudo pacman -S base-devel cmake v4l-utils ffmpeg i2c-tools pkgconf"
    exit 1
fi

echo

# Create build directory
echo -e "${YELLOW}Creating build directory...${NC}"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Configure with CMake
echo -e "${YELLOW}Configuring with CMake...${NC}"
cmake .. \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DMAX_SERVO_COUNT="$SERVO_COUNT" \
    -DENABLE_HARDWARE_ENCODING="$ENABLE_HW_ENCODING" \
    -DENABLE_RT_OPTIMIZATION="$ENABLE_RT_OPT" \
    -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX"

# Build
echo -e "${YELLOW}Building application...${NC}"
NPROC=$(nproc)
make -j"$NPROC"

echo -e "${GREEN}Build completed successfully!${NC}"
echo

# Show build information
echo -e "${BLUE}=== Build Information ===${NC}"
echo -e "${BLUE}Executable:${NC} $(pwd)/linux_cam"
echo -e "${BLUE}Size:${NC} $(du -h linux_cam | cut -f1)"
echo -e "${BLUE}Type:${NC} $(file linux_cam)"
echo

# Check if we can create systemd service
if [ -f "linux_cam.service" ]; then
    echo -e "${BLUE}Systemd Service:${NC} $(pwd)/linux_cam.service"
fi

echo -e "${GREEN}=== Next Steps ===${NC}"
echo "1. Test the application:"
echo "   ./linux_cam --help"
echo "   ./linux_cam --config ../config/camera_config.json"
echo
echo "2. Install system-wide (optional):"
echo "   sudo make install"
echo
echo "3. Install as systemd service:"
echo "   sudo cp linux_cam.service /etc/systemd/system/"
echo "   sudo systemctl daemon-reload"
echo "   sudo systemctl enable linux_cam"
echo
echo "4. Check camera device:"
echo "   ls -l /dev/video*"
echo "   v4l2-ctl --list-devices"
echo
echo "5. Check I2C device:"
echo "   ls -l /dev/i2c*"
echo "   sudo i2cdetect -y 1"

cd ..
echo -e "${GREEN}Build script completed!${NC}"
