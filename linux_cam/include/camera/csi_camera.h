#pragma once

#include <memory>
#include <functional>
#include <atomic>
#include <thread>
#include <vector>
#include <string>
#include <linux/videodev2.h>

namespace camera {

struct FrameData {
    uint8_t* data;
    size_t size;
    uint64_t timestamp_us;
    uint32_t sequence;
};

using FrameCallback = std::function<void(const FrameData&)>;

class CSICamera {
public:
    struct Config {
        std::string device_path = "/dev/video0";
        uint32_t width = 640;           // Low resolution for 4G efficiency  
        uint32_t height = 480;
        uint32_t fps = 15;              // Lower FPS for 4G bandwidth
        uint32_t pixel_format = V4L2_PIX_FMT_NV12;  // Optimal for H.264 hardware encoders
        uint32_t buffer_count = 2;      // Minimal buffering for low latency
    };

    explicit CSICamera(const Config& config);
    ~CSICamera();

    // Non-copyable, movable
    CSICamera(const CSICamera&) = delete;
    CSICamera& operator=(const CSICamera&) = delete;
    CSICamera(CSICamera&&) = default;
    CSICamera& operator=(CSICamera&&) = default;

    bool initialize();
    bool start_capture(FrameCallback callback);
    void stop_capture();
    bool is_capturing() const;

    // Camera controls
    bool set_exposure(int32_t value);
    bool set_gain(int32_t value);
    bool set_white_balance(int32_t value);

    const Config& get_config() const { return config_; }

private:
    Config config_;
    int fd_;
    std::atomic<bool> capturing_;
    std::atomic<bool> should_stop_;
    
    struct Buffer {
        void* start;
        size_t length;
    };
    
    std::vector<Buffer> buffers_;
    std::thread capture_thread_;
    FrameCallback frame_callback_;

    bool open_device();
    bool setup_format();
    bool setup_buffers();
    bool start_streaming();
    void stop_streaming();
    void capture_loop();
    bool query_buffer(uint32_t index);
    bool queue_buffer(uint32_t index);
    bool dequeue_buffer(v4l2_buffer& buf);
};

} // namespace camera
