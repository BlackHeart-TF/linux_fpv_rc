#pragma once

#include <memory>
#include <functional>
#include <atomic>
#include <mutex>
#include "camera/csi_camera.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libswscale/swscale.h>
}

namespace encoder {

struct EncodedFrame {
    uint8_t* data;
    size_t size;
    uint64_t timestamp_us;
    bool is_keyframe;
};

using EncodedFrameCallback = std::function<void(const EncodedFrame&)>;

class H264Encoder {
public:
    struct Config {
        uint32_t width = 640;
        uint32_t height = 480;
        uint32_t fps = 15;
        uint32_t bitrate = 200000;      // 200 kbps for 4G efficiency
        uint32_t gop_size = 15;       // Very large GOP - only I-frame at start
        uint32_t max_b_frames = 0;      // No B-frames, only I and P
        bool use_hardware = true;
        std::string preset = "ultrafast";
        std::string profile = "baseline";
        bool force_idr_on_first = true; // Force IDR frame as first frame
        bool no_scenecut = true;        // Disable scene cut detection
        bool constant_quality = false;  // Use CBR for predictable bandwidth
    };

    explicit H264Encoder(const Config& config);
    ~H264Encoder();

    // Non-copyable, movable
    H264Encoder(const H264Encoder&) = delete;
    H264Encoder& operator=(const H264Encoder&) = delete;
    H264Encoder(H264Encoder&&) = default;
    H264Encoder& operator=(H264Encoder&&) = default;

    bool initialize();
    void set_encoded_frame_callback(EncodedFrameCallback callback);
    bool encode_frame(const camera::FrameData& frame);
    void flush();
    
    // Force next frame to be I-frame (e.g., on startup or command)
    void force_next_iframe();
    
    // Get encoding statistics
    struct Stats {
        uint64_t iframes_encoded;
        uint64_t pframes_encoded;
        uint64_t total_bytes_encoded;
        double avg_iframe_size;
        double avg_pframe_size;
    };
    Stats get_stats() const;

private:
    Config config_;
    EncodedFrameCallback encoded_callback_;
    
    const AVCodec* codec_;
    AVCodecContext* codec_ctx_;
    AVFrame* frame_;
    AVPacket* packet_;
    SwsContext* sws_ctx_;
    
    uint64_t frame_count_;
    std::atomic<bool> initialized_;
    std::atomic<bool> force_iframe_;
    
    // Statistics
    mutable std::mutex stats_mutex_;
    Stats stats_;

    bool setup_codec();
    bool setup_hardware_encoder();
    bool setup_software_encoder();
    bool convert_frame(const camera::FrameData& input, AVFrame* output);
    void cleanup();
};

} // namespace encoder
