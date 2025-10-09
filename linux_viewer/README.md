# Linux FPV Viewer

A real-time H.264 video receiver and gamepad-to-servo controller for FPV (First Person View) applications. This application receives H.264 video streams over UDP and sends gamepad input as servo commands to control remote devices.

## Features

- **H.264 Video Decoding**: Hardware-accelerated H.264 decoding using FFmpeg/libav
- **Real-time Display**: SDL2-based video display with low latency
- **Gamepad Input**: SDL2 gamepad/joystick support with configurable mappings
- **Network Communication**: UDP video reception and command transmission
- **Performance Monitoring**: Built-in performance metrics and statistics
- **Configurable**: JSON-based configuration system
- **Cross-platform**: Linux-focused but portable design

## Requirements

### System Dependencies

```bash
# Ubuntu/Debian
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

# Arch Linux
sudo pacman -S \
    base-devel \
    cmake \
    pkgconf \
    ffmpeg \
    sdl2 \
    sdl2_image
```

### Hardware

- **Video**: Any system capable of H.264 decoding (hardware acceleration recommended)
- **Gamepad**: Any SDL2-compatible gamepad/joystick
- **Network**: UDP networking capability

## Building

### Using CMake (Recommended)

```bash
./build.sh
# or for installation:
./build.sh install
```

### Using Make (Simple)

```bash
make deps  # Install dependencies
make       # Build application
make install  # Install to system
```

## Configuration

The application uses a JSON configuration file. Default locations:
1. `./config/viewer_config.json`
2. `/etc/linux_viewer/viewer_config.json`

### Example Configuration

```json
{
  "network": {
    "video_receive_port": 5000,
    "command_send_ip": "192.168.1.100",
    "command_send_port": 5001,
    "receive_buffer_size": 1048576,
    "heartbeat_interval_ms": 1000
  },
  "decoder": {
    "enable_hardware_acceleration": true,
    "max_frame_queue_size": 10,
    "decoder_threads": 2
  },
  "display": {
    "window_width": 640,
    "window_height": 480,
    "fullscreen": false,
    "vsync": true,
    "show_stats": true
  },
  "gamepad": {
    "device_index": 0,
    "deadzone": 0.1,
    "servo_mappings": [
      {
        "servo_id": 0,
        "axis": 0,
        "invert": false,
        "min_value": 1000,
        "max_value": 2000
      }
    ],
    "button_mappings": [
      {
        "button": 0,
        "action": "center_servos"
      }
    ]
  }
}
```

## Usage

### Basic Usage

```bash
# Run with default configuration
./linux_viewer

# Run with custom configuration
./linux_viewer --config my_config.json

# Show help
./linux_viewer --help
```

### Controls

- **ESC/Q**: Quit application
- **F**: Toggle fullscreen
- **C**: Center all servos
- **I**: Force I-frame request
- **S**: Show performance statistics

### Gamepad Controls

Gamepad controls are fully configurable through the configuration file:

- **Analog Sticks**: Map to servo positions
- **Buttons**: Trigger various actions (center servos, force I-frame, etc.)
- **D-Pad**: Can be mapped to additional servo controls

## Network Protocol

### Video Reception (Port 5000)

The application receives H.264 video frames over UDP with a custom framing protocol:

```c
struct FrameHeader {
    uint32_t timestamp;
    uint32_t sequence;
    uint16_t fragment_id;
    uint16_t fragment_offset;
    uint16_t fragment_size;
    uint8_t flags; // bit 0: keyframe, bit 1: last fragment
    uint8_t reserved;
} __attribute__((packed));
```

### Command Transmission (Port 5001)

Servo commands are sent using a simple protocol:

```c
enum CommandOpcode {
    SERVO_MOVE = 0x01,      // servo_id(1) + position(2) + speed(1)
    SERVO_MULTI = 0x02,     // count(1) + [servo_id(1) + position(2)]...
    SYSTEM_SHUTDOWN = 0x10,
    FORCE_IFRAME = 0x11,
    SET_BITRATE = 0x12,
    HEARTBEAT = 0xFF
};
```

## Integration with linux_cam

This viewer is designed to work with the `linux_cam` application:

1. **linux_cam** captures video, encodes to H.264, and streams over UDP
2. **linux_viewer** receives video, decodes, displays, and sends gamepad commands
3. **linux_cam** receives commands and controls servos via I2C

### Network Setup

```
[Camera Device] ---> linux_cam ---> [Network] ---> linux_viewer ---> [Gamepad]
                         ^                              |
                         |                              v
                    [Servo Control] <------------- [Commands]
```

## Performance Optimization

### Hardware Acceleration

Enable hardware decoding for better performance:

```json
{
  "decoder": {
    "enable_hardware_acceleration": true
  }
}
```

Supported acceleration:
- **VAAPI** (Intel/AMD)
- **VDPAU** (NVIDIA)
- **CUDA** (NVIDIA)
- **QSV** (Intel Quick Sync)

### Network Optimization

- Adjust buffer sizes based on network conditions
- Use appropriate MTU settings
- Consider QoS/traffic shaping for video traffic

### Display Optimization

- Enable VSync for smooth display
- Use appropriate window resolution
- Consider fullscreen mode for dedicated displays

## Troubleshooting

### Common Issues

1. **No Video**: Check network connectivity and port configuration
2. **Gamepad Not Detected**: Verify SDL2 gamepad support and device permissions
3. **High Latency**: Disable VSync, reduce buffer sizes, enable hardware acceleration
4. **Audio Issues**: This application only handles video; audio requires separate handling

### Debug Information

Enable debug logging:

```json
{
  "logging": {
    "log_level": "DEBUG",
    "log_file": "/tmp/linux_viewer.log",
    "enable_console_output": true
  }
}
```

### Performance Monitoring

The application includes built-in performance monitoring:

- Frame rates and timing
- Network statistics
- Decode performance
- System resource usage

## Development

### Project Structure

```
linux_viewer/
├── src/
│   ├── main.cpp              # Main application
│   ├── decoder/              # H.264 decoding
│   ├── gamepad/              # Gamepad input handling
│   ├── network/              # UDP communication
│   ├── utils/                # Logging, performance monitoring
│   └── config/               # Configuration management
├── include/                  # Header files
├── config/                   # Configuration files
└── build/                    # Build output
```

### Adding Features

1. **New Command Types**: Extend `CommandOpcode` enum and handlers
2. **Additional Codecs**: Extend decoder interface
3. **UI Improvements**: Enhance SDL2 rendering
4. **Platform Support**: Add platform-specific optimizations

## License

This project is licensed under the same terms as the parent linux_fpv_rc project.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make changes with appropriate tests
4. Submit a pull request

## See Also

- [linux_cam](../linux_cam/README.md) - Camera capture and streaming
- [SDL2 Documentation](https://wiki.libsdl.org/)
- [FFmpeg Documentation](https://ffmpeg.org/documentation.html)
