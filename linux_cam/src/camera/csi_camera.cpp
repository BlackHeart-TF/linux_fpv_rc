#include "camera/csi_camera.h"
#include "utils/logger.h"
#include "utils/performance_monitor.h"

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <cstring>
#include <cerrno>

namespace camera {

CSICamera::CSICamera(const Config& config)
    : config_(config), fd_(-1), capturing_(false), should_stop_(false) {
}

CSICamera::~CSICamera() {
    stop_capture();
    if (fd_ >= 0) {
        close(fd_);
    }
}

bool CSICamera::initialize() {
    LOG_INFO("Initializing CSI camera with device: " + config_.device_path);
    LOG_INFO("Config: " + std::to_string(config_.width) + "x" + std::to_string(config_.height) + 
             "@" + std::to_string(config_.fps) + "fps, format=0x" + std::to_string(config_.pixel_format));
    
    if (!open_device()) {
        LOG_ERROR("Failed to open camera device");
        return false;
    }
    LOG_INFO("Camera device opened successfully");
    
    if (!setup_format()) {
        LOG_ERROR("Failed to setup camera format");
        return false;
    }
    LOG_INFO("Camera format setup successfully");
    
    if (!setup_buffers()) {
        LOG_ERROR("Failed to setup camera buffers");
        return false;
    }
    LOG_INFO("Camera buffers setup successfully");
    
    LOG_INFO("CSI camera initialized successfully");
    return true;
}

bool CSICamera::start_capture(FrameCallback callback) {
    if (capturing_) {
        LOG_WARNING("Camera is already capturing");
        return true;
    }
    
    if (!callback) {
        LOG_ERROR("Invalid frame callback provided");
        return false;
    }
    
    frame_callback_ = callback;
    
    if (!start_streaming()) {
        LOG_ERROR("Failed to start V4L2 streaming");
        return false;
    }
    
    should_stop_ = false;
    capture_thread_ = std::thread(&CSICamera::capture_loop, this);
    capturing_ = true;
    
    LOG_INFO("Camera capture started");
    return true;
}

void CSICamera::stop_capture() {
    if (!capturing_) {
        return;
    }
    
    LOG_INFO("Stopping camera capture");
    should_stop_ = true;
    
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
    
    stop_streaming();
    capturing_ = false;
    
    LOG_INFO("Camera capture stopped");
}

bool CSICamera::is_capturing() const {
    return capturing_;
}

bool CSICamera::set_exposure(int32_t value) {
    v4l2_control ctrl = {};
    ctrl.id = V4L2_CID_EXPOSURE;
    ctrl.value = value;
    
    if (ioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0) {
        LOG_ERROR("Failed to set exposure: " + std::string(strerror(errno)));
        return false;
    }
    
    return true;
}

bool CSICamera::set_gain(int32_t value) {
    v4l2_control ctrl = {};
    ctrl.id = V4L2_CID_GAIN;
    ctrl.value = value;
    
    if (ioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0) {
        LOG_ERROR("Failed to set gain: " + std::string(strerror(errno)));
        return false;
    }
    
    return true;
}

bool CSICamera::set_white_balance(int32_t value) {
    v4l2_control ctrl = {};
    ctrl.id = V4L2_CID_WHITE_BALANCE_TEMPERATURE;
    ctrl.value = value;
    
    if (ioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0) {
        LOG_ERROR("Failed to set white balance: " + std::string(strerror(errno)));
        return false;
    }
    
    return true;
}

bool CSICamera::open_device() {
    fd_ = open(config_.device_path.c_str(), O_RDWR | O_NONBLOCK);
    if (fd_ < 0) {
        LOG_ERROR("Failed to open device " + config_.device_path + ": " + strerror(errno));
        return false;
    }
    
    // Query capabilities
    v4l2_capability cap = {};
    if (ioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
        LOG_ERROR("Failed to query device capabilities: " + std::string(strerror(errno)));
        return false;
    }
    
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        LOG_ERROR("Device does not support video capture");
        return false;
    }
    
    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        LOG_ERROR("Device does not support streaming");
        return false;
    }
    
    LOG_INFO("Camera device opened: " + std::string(reinterpret_cast<char*>(cap.card)));
    return true;
}

bool CSICamera::setup_format() {
    v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = config_.width;
    fmt.fmt.pix.height = config_.height;
    fmt.fmt.pix.pixelformat = config_.pixel_format;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;  // Use FIELD_NONE instead of INTERLACED for USB cameras
    
    LOG_INFO("Setting camera format: " + std::to_string(config_.width) + "x" + std::to_string(config_.height) + 
             " format=0x" + std::to_string(config_.pixel_format));
    
    if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
        LOG_ERROR("Failed to set format: " + std::string(strerror(errno)));
        return false;
    }
    
    // Verify the format was set correctly
    LOG_INFO("Camera accepted format: " + std::to_string(fmt.fmt.pix.width) + "x" + std::to_string(fmt.fmt.pix.height) + 
             " format=0x" + std::to_string(fmt.fmt.pix.pixelformat));
             
    if (fmt.fmt.pix.width != config_.width || fmt.fmt.pix.height != config_.height) {
        LOG_WARNING("Requested format not fully supported, using " +
                   std::to_string(fmt.fmt.pix.width) + "x" + std::to_string(fmt.fmt.pix.height));
        config_.width = fmt.fmt.pix.width;
        config_.height = fmt.fmt.pix.height;
    }
    
    if (fmt.fmt.pix.pixelformat != config_.pixel_format) {
        LOG_ERROR("Camera rejected pixel format! Requested: 0x" + std::to_string(config_.pixel_format) + 
                  " Got: 0x" + std::to_string(fmt.fmt.pix.pixelformat));
        return false;
    }
    
    // Set frame rate
    v4l2_streamparm parm = {};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = config_.fps;
    
    if (ioctl(fd_, VIDIOC_S_PARM, &parm) < 0) {
        LOG_WARNING("Failed to set frame rate, using default: " + std::string(strerror(errno)));
    }
    
    LOG_INFO("Camera format set: " + std::to_string(config_.width) + "x" + 
             std::to_string(config_.height) + " @ " + std::to_string(config_.fps) + " fps");
    
    return true;
}

bool CSICamera::setup_buffers() {
    v4l2_requestbuffers req = {};
    req.count = config_.buffer_count;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    
    if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
        LOG_ERROR("Failed to request buffers: " + std::string(strerror(errno)));
        return false;
    }
    
    if (req.count < 2) {
        LOG_ERROR("Insufficient buffer memory");
        return false;
    }
    
    buffers_.resize(req.count);
    
    for (uint32_t i = 0; i < req.count; i++) {
        v4l2_buffer buf = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        
        if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
            LOG_ERROR("Failed to query buffer " + std::to_string(i) + ": " + strerror(errno));
            return false;
        }
        
        buffers_[i].length = buf.length;
        buffers_[i].start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, 
                                MAP_SHARED, fd_, buf.m.offset);
        
        if (buffers_[i].start == MAP_FAILED) {
            LOG_ERROR("Failed to mmap buffer " + std::to_string(i) + ": " + strerror(errno));
            return false;
        }
        
        // Queue the buffer
        if (!queue_buffer(i)) {
            LOG_ERROR("Failed to queue buffer " + std::to_string(i));
            return false;
        }
    }
    
    LOG_INFO("Set up " + std::to_string(req.count) + " camera buffers");
    return true;
}

bool CSICamera::start_streaming() {
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
        LOG_ERROR("Failed to start streaming: " + std::string(strerror(errno)));
        return false;
    }
    
    return true;
}

void CSICamera::stop_streaming() {
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_STREAMOFF, &type) < 0) {
        LOG_ERROR("Failed to stop streaming: " + std::string(strerror(errno)));
    }
    
    // Unmap buffers
    for (auto& buffer : buffers_) {
        if (buffer.start != MAP_FAILED) {
            munmap(buffer.start, buffer.length);
            buffer.start = MAP_FAILED;
        }
    }
    buffers_.clear();
}

void CSICamera::capture_loop() {
    LOG_INFO("Camera capture loop started");
    
    while (!should_stop_) {
        // Use poll to wait for frame with timeout
        pollfd pfd = {};
        pfd.fd = fd_;
        pfd.events = POLLIN;
        
        int ret = poll(&pfd, 1, 100); // 100ms timeout
        if (ret < 0) {
            if (errno != EINTR) {
                LOG_ERROR("Poll error: " + std::string(strerror(errno)));
                break;
            }
            continue;
        }
        
        if (ret == 0) {
            // Timeout, continue
            continue;
        }
        
        if (!(pfd.revents & POLLIN)) {
            continue;
        }
        
        // Dequeue buffer
        v4l2_buffer buf = {};
        if (!dequeue_buffer(buf)) {
            continue;
        }
        
        // Performance measurement
        MEASURE_CAPTURE_TIME();
        
        // Create frame data
        FrameData frame;
        frame.data = static_cast<uint8_t*>(buffers_[buf.index].start);
        frame.size = buf.bytesused;
        frame.timestamp_us = buf.timestamp.tv_sec * 1000000LL + buf.timestamp.tv_usec;
        frame.sequence = buf.sequence;
        
        // Call frame callback
        if (frame_callback_) {
            frame_callback_(frame);
        }
        
        // Re-queue the buffer
        queue_buffer(buf.index);
    }
    
    LOG_INFO("Camera capture loop ended");
}

bool CSICamera::queue_buffer(uint32_t index) {
    v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = index;
    
    if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
        LOG_ERROR("Failed to queue buffer " + std::to_string(index) + ": " + strerror(errno));
        return false;
    }
    
    return true;
}

bool CSICamera::dequeue_buffer(v4l2_buffer& buf) {
    buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    
    if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
        if (errno != EAGAIN) {
            LOG_ERROR("Failed to dequeue buffer: " + std::string(strerror(errno)));
        }
        return false;
    }
    
    return true;
}

} // namespace camera
