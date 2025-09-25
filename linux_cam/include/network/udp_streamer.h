#pragma once

#include <string>
#include <atomic>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <sys/socket.h>
#include <netinet/in.h>
#include "encoder/h264_encoder.h"

namespace network {

class UDPStreamer {
public:
    struct Config {
        std::string target_ip = "192.168.1.100";
        uint16_t target_port = 5000;
        uint16_t mtu = 1400; // Leave room for UDP/IP headers
        uint32_t send_buffer_size = 1024 * 1024; // 1MB
        bool enable_fragmentation = true;
    };

    explicit UDPStreamer(const Config& config);
    ~UDPStreamer();

    // Non-copyable, movable
    UDPStreamer(const UDPStreamer&) = delete;
    UDPStreamer& operator=(const UDPStreamer&) = delete;
    UDPStreamer(UDPStreamer&&) = default;
    UDPStreamer& operator=(UDPStreamer&&) = default;

    bool initialize();
    void start_streaming();
    void stop_streaming();
    bool is_streaming() const;

    void queue_frame(const encoder::EncodedFrame& frame);

    // Statistics
    struct Stats {
        uint64_t frames_sent;
        uint64_t bytes_sent;
        uint64_t packets_sent;
        uint64_t packets_dropped;
        double average_bitrate;
    };
    
    Stats get_stats() const;

private:
    Config config_;
    int socket_fd_;
    sockaddr_in target_addr_;
    
    std::atomic<bool> streaming_;
    std::atomic<bool> should_stop_;
    std::thread sender_thread_;
    
    // Frame queue for thread-safe operation
    std::queue<encoder::EncodedFrame> frame_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    static constexpr size_t MAX_QUEUE_SIZE = 10;
    
    // Statistics
    mutable std::mutex stats_mutex_;
    Stats stats_;
    
    bool setup_socket();
    void sender_loop();
    bool send_frame(const encoder::EncodedFrame& frame);
    bool send_packet(const uint8_t* data, size_t size, bool is_fragment = false, 
                    uint16_t fragment_id = 0, uint16_t fragment_offset = 0);
    void update_stats(size_t bytes_sent, bool packet_sent);
    void cleanup();
};

// RTP-like header for frame fragmentation
struct FrameHeader {
    uint32_t timestamp;
    uint32_t sequence;
    uint16_t fragment_id;
    uint16_t fragment_offset;
    uint16_t fragment_size;
    uint8_t flags; // bit 0: keyframe, bit 1: last fragment
    uint8_t reserved;
} __attribute__((packed));

} // namespace network
