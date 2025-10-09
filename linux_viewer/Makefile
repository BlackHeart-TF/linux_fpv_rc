# Simple Makefile for linux_viewer
# For full build system, use CMake with build.sh

CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -g
INCLUDES = -Iinclude
LIBS = -lavcodec -lavformat -lavutil -lavdevice -lswscale -lSDL2 -lSDL2_image -lpthread

SRCDIR = src
OBJDIR = build/obj
SOURCES = $(shell find $(SRCDIR) -name "*.cpp")
OBJECTS = $(SOURCES:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
TARGET = build/linux_viewer

.PHONY: all clean install

all: $(TARGET)

$(TARGET): $(OBJECTS) | build
	$(CXX) $(OBJECTS) -o $@ $(LIBS)

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp | $(OBJDIR)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

build:
	@mkdir -p build

$(OBJDIR):
	@mkdir -p $(OBJDIR)

clean:
	rm -rf build

install: $(TARGET)
	sudo cp $(TARGET) /usr/local/bin/
	sudo mkdir -p /etc/linux_viewer
	sudo cp config/viewer_config.json /etc/linux_viewer/

.PHONY: deps
deps:
	@echo "Installing dependencies..."
	sudo apt-get update
	sudo apt-get install -y \
		build-essential \
		cmake \
		pkg-config \
		libavcodec-dev \
		libavformat-dev \
		libavutil-dev \
		libavdevice-dev \
		libswscale-dev \
		libsdl2-dev \
		libsdl2-image-dev

help:
	@echo "Available targets:"
	@echo "  all     - Build the application"
	@echo "  clean   - Remove build files"
	@echo "  install - Install to system"
	@echo "  deps    - Install dependencies"
	@echo "  help    - Show this help"
