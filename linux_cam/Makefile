# Simple Makefile for testing compilation
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -DMAX_SERVO_COUNT=8
INCLUDES = -Iinclude
LDFLAGS = -lpthread

# Check for dependencies
HAS_V4L2 := $(shell pkg-config --exists libv4l2 && echo 1)
HAS_FFMPEG := $(shell pkg-config --exists libavcodec libavformat libavutil libswscale && echo 1)

ifeq ($(HAS_V4L2),1)
    CXXFLAGS += $(shell pkg-config --cflags libv4l2)
    LDFLAGS += $(shell pkg-config --libs libv4l2)
else
    CXXFLAGS += -DMOCK_V4L2
endif

ifeq ($(HAS_FFMPEG),1)
    CXXFLAGS += $(shell pkg-config --cflags libavcodec libavformat libavutil libswscale)
    LDFLAGS += $(shell pkg-config --libs libavcodec libavformat libavutil libswscale)
else
    CXXFLAGS += -DMOCK_FFMPEG
endif

SRCDIR = src
SOURCES = $(wildcard $(SRCDIR)/*/*.cpp) $(wildcard $(SRCDIR)/*.cpp)
OBJECTS = $(SOURCES:.cpp=.o)
TARGET = linux_cam

.PHONY: all clean test-compile

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(OBJECTS) -o $@ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

test-compile: 
	@echo "Testing compilation of individual files..."
	@echo "Sources found: $(SOURCES)"
	@echo "V4L2 available: $(HAS_V4L2)"
	@echo "FFmpeg available: $(HAS_FFMPEG)"
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c src/main.cpp -o /tmp/test_main.o
	@echo "Main compilation successful!"

clean:
	rm -f $(OBJECTS) $(TARGET)

# Individual file compilation for debugging
src/main.o: src/main.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

src/camera/csi_camera.o: src/camera/csi_camera.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

src/encoder/h264_encoder.o: src/encoder/h264_encoder.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@
