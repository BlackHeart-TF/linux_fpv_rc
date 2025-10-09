#pragma once

#include <string>
#include <atomic>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <memory>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

namespace decoder {

struct DecodedFrame {
    uint8_t* data[4];           // Planar data pointers (Y, U, V, A)
    int linesize[4];            // Line sizes for each plane
    int width;
    int height;
    int64_t timestamp_us;
    AVPixelFormat pixel_format;
    
    DecodedFrame() {
        memset(data, 0, sizeof(data));
        memset(linesize, 0, sizeof(linesize));
        width = height = 0;
        timestamp_us = 0;
        pixel_format = AV_PIX_FMT_NONE;
    }
};

struct EncodedPacket {
    std::vector<uint8_t> data;
    int64_t timestamp_us;
    bool is_keyframe;
    
    EncodedPacket() : timestamp_us(0), is_keyframe(false) {}
    EncodedPacket(const uint8_t* packet_data, size_t size, int64_t ts, bool keyframe)
        : data(packet_data, packet_data + size), timestamp_us(ts), is_keyframe(keyframe) {}
};

using DecodedFrameCallback = std::function<void(const DecodedFrame& frame)>;

class H264Decoder {
public:
    struct Config {
        bool enable_hardware_acceleration = true;
        int max_frame_queue_size = 10;
        int decoder_threads = 2;
        AVPixelFormat output_format = AV_PIX_FMT_YUV420P;
        bool enable_multithreading = true;
    };

    explicit H264Decoder(const Config& config);
    ~H264Decoder();

    // Non-copyable, movable
    H264Decoder(const H264Decoder&) = delete;
    H264Decoder& operator=(const H264Decoder&) = delete;
    H264Decoder(H264Decoder&&) = default;
    H264Decoder& operator=(H264Decoder&&) = default;

    bool initialize();
    void shutdown();
    bool is_initialized() const;

    // Decoding interface
    bool decode_packet(const EncodedPacket& packet);
    void set_decoded_frame_callback(DecodedFrameCallback callback);

    // Statistics
    struct Stats {
        uint64_t packets_received;
        uint64_t frames_decoded;
        uint64_t frames_dropped;
        uint64_t decode_errors;
        double average_decode_time_ms;
        double current_fps;
    };
    
    Stats get_stats() const;

    // Utility functions
    static std::string get_decoder_info();
    static bool is_hardware_acceleration_available();

private:
    Config config_;
    std::atomic<bool> initialized_;
    
    // FFmpeg components
    const AVCodec* codec_;
    AVCodecContext* codec_context_;
    AVFrame* frame_;
    AVPacket* packet_;
    SwsContext* sws_context_;
    
    // Frame conversion
    AVFrame* converted_frame_;
    uint8_t* conversion_buffer_;
    size_t conversion_buffer_size_;
    
    // Threading
    std::atomic<bool> should_stop_;
    std::thread decoder_thread_;
    
    // Packet queue
    std::queue<EncodedPacket> packet_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    
    // Callback
    DecodedFrameCallback frame_callback_;
    std::mutex callback_mutex_;
    
    // Statistics
    mutable std::mutex stats_mutex_;
    Stats stats_;
    std::chrono::steady_clock::time_point last_stats_update_;
    
    bool setup_decoder();
    bool setup_hardware_acceleration();
    bool setup_software_decoder();
    void decoder_loop();
    bool process_packet(const EncodedPacket& encoded_packet);
    bool convert_frame(AVFrame* src_frame, DecodedFrame& dst_frame);
    void update_stats(bool decode_success, double decode_time_ms);
    void cleanup();
    
    // Hardware acceleration helpers
    static AVPixelFormat get_hw_format(AVCodecContext* ctx, const AVPixelFormat* pix_fmts);
    bool init_hw_decoder(AVHWDeviceType type);
};

} // namespace decoder
