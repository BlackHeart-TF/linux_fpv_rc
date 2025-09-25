# Linux Camera RT - Embedded Camera Streaming Application

A high-performance C++ application designed for embedded Linux systems with RT kernel support. Captures video from CSI camera interfaces, encodes to H.264, streams via UDP, and provides servo control capabilities.

DISPLAIMER: This is largely AI assisted and not cleaned up, fully tested, or optimized. 

CURRENT STATUS: not fully working, significant data loss, high latency, but technically does transmit the video


## Features

- **CSI Camera Interface**: V4L2-based camera capture with configurable resolution and frame rate
- **H.264 Encoding**: Hardware-accelerated encoding with fallback to software encoding
- **UDP Streaming**: Low-latency video streaming with frame fragmentation support
- **Servo Control**: I2C-based servo controller supporting up to 8 servos (compile-time configurable)
- **Command Interface**: UDP command receiver for servo control and system commands
- **RT Optimization**: Real-time kernel optimizations for low-latency performance
- **Performance Monitoring**: Comprehensive performance metrics and logging

## System Requirements

### Hardware
- Embedded Linux system (ARM/x86_64)
- CSI camera interface (/dev/video0)
- I2C interface for servo control (/dev/i2c-1)
- Network interface for UDP streaming

### Software
- Linux kernel 4.19+ (RT kernel recommended)
- V4L2 (Video4Linux2) support
- FFmpeg libraries (libavcodec, libavformat, libavutil, libswscale)
- I2C development libraries

### Recommended Packages (Debian/Ubuntu)
```bash
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    libv4l-dev \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libswscale-dev \
    libi2c-dev \
    pkg-config
```

## Testing
To receive the video stream
```bash
ffplay -i udp://127.0.0.1:5000 -fflags nobuffer -flags low_delay -framedrop
```

## Build Instructions

1. **Clone and navigate to the project**:
```bash
cd linux_cam
```

2. **Create build directory**:
```bash
mkdir build && cd build
```

3. **Configure with CMake**:
```bash
# Basic build
cmake ..

# With custom servo count (default is 8)
cmake -DMAX_SERVO_COUNT=4 ..

# Disable hardware encoding
cmake -DENABLE_HARDWARE_ENCODING=OFF ..

# Disable RT optimizations
cmake -DENABLE_RT_OPTIMIZATION=OFF ..
```

4. **Build the application**:
```bash
make -j$(nproc)
```

5. **Install (optional)**:
```bash
sudo make install
```

## Configuration

The application uses a JSON configuration file. Example configuration:

```json
{
  "camera": {
    "device_path": "/dev/video0",
    "width": 1920,
    "height": 1080,
    "fps": 30
  },
  "encoder": {
    "bitrate": 2000000,
    "preset": "ultrafast",
    "use_hardware": true
  },
  "streamer": {
    "target_ip": "192.168.1.100",
    "target_port": 5000
  },
  "command_receiver": {
    "listen_port": 5001
  },
  "servo_controller": {
    "i2c_device": "/dev/i2c-1",
    "controller_address": 64
  }
}
```

## Usage

### Basic Usage
```bash
# Run with default configuration
./linux_cam

# Run with custom configuration
./linux_cam --config /path/to/config.json
```

### Systemd Service
```bash
# Install service
sudo cp linux_cam.service /etc/systemd/system/
sudo systemctl daemon-reload

# Enable and start
sudo systemctl enable linux_cam
sudo systemctl start linux_cam

# Check status
sudo systemctl status linux_cam

# View logs
sudo journalctl -u linux_cam -f
```

## Command Protocol

The application listens for UDP command packets on the configured port (default 5001).

### Command Packet Structure
```c
struct CommandPacket {
    CommandHeader header;
    ServoCommand servos[MAX_SERVO_COUNT];
    uint8_t padding[32];
} __attribute__((packed));
```

### Servo Commands
```c
struct ServoCommand {
    uint8_t servo_id;      // 0-7 for up to 8 servos
    uint16_t position;     // 0-65535, maps to servo range
    uint8_t speed;         // 0-255, servo movement speed
} __attribute__((packed));
```

### Command Types
- `SERVO_CONTROL` (0x01): Control servo positions
- `SYSTEM_SHUTDOWN` (0x02): Shutdown the application
- `HEARTBEAT` (0x04): Keep-alive packet

## Hardware Setup

### Camera Connection
- Connect CSI camera to the camera interface
- Ensure camera is detected: `ls /dev/video*`
- Test camera: `v4l2-ctl --list-devices`

### Servo Controller (PCA9685)
- Connect PCA9685 to I2C bus (default: /dev/i2c-1)
- Default I2C address: 0x40
- Verify connection: `i2cdetect -y 1`

### Wiring Example (Raspberry Pi)
```
PCA9685   RPi
VCC   --> 5V
GND   --> GND
SDA   --> GPIO 2 (SDA)
SCL   --> GPIO 3 (SCL)
```

## Performance Tuning

### RT Kernel Optimizations
```bash
# Enable RT scheduling
echo 'kernel.sched_rt_runtime_us = -1' >> /etc/sysctl.conf

# Set CPU isolation (optional)
# Add to kernel command line: isolcpus=1,2,3
```

### Application-level Tuning
- Use hardware encoding when available
- Adjust buffer count based on memory constraints
- Configure CPU affinity for dedicated cores
- Enable memory locking for RT performance

### Network Optimization
```bash
# Increase UDP buffer sizes
echo 'net.core.rmem_max = 16777216' >> /etc/sysctl.conf
echo 'net.core.wmem_max = 16777216' >> /etc/sysctl.conf
echo 'net.core.rmem_default = 262144' >> /etc/sysctl.conf
echo 'net.core.wmem_default = 262144' >> /etc/sysctl.conf
```

## Troubleshooting

### Camera Issues
```bash
# Check camera detection
ls /dev/video*

# Test camera formats
v4l2-ctl -d /dev/video0 --list-formats-ext

# Check permissions
sudo usermod -a -G video $USER
```

### I2C Issues
```bash
# Check I2C devices
sudo i2cdetect -y 1

# Check permissions
sudo usermod -a -G i2c $USER
```

### Network Issues
```bash
# Check UDP ports
sudo netstat -ulnp | grep 500

# Test network connectivity
ping <target_ip>
```

### Performance Issues
- Monitor with: `sudo journalctl -u linux_cam -f`
- Check CPU usage: `top -p $(pgrep linux_cam)`
- Verify RT scheduling: `chrt -p $(pgrep linux_cam)`

## Development

### Adding New Features
1. Add interface to appropriate header file
2. Implement in corresponding source file
3. Update configuration system if needed
4. Add proper error handling and logging
5. Update documentation

### Testing
```bash
# Build tests (if available)
cmake -DBUILD_TESTS=ON ..
make tests
./tests/unit_tests
```

## License

This project is provided as-is for embedded camera applications. Modify as needed for your specific use case.

## Support

For issues specific to your hardware platform, consult your board manufacturer's documentation. For RT kernel setup, refer to the RT kernel documentation for your distribution.
