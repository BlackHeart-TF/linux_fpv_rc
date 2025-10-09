#pragma once

#include <string>
#include <atomic>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <map>
#include <sys/socket.h>
#include <netinet/in.h>

#include "decoder/h264_decoder.h"

namespace network {

// Frame header structure (matches linux_cam)
struct FrameHeader {
    uint32_t timestamp;
    uint32_t sequence;
    uint16_t fragment_id;
    uint16_t fragment_offset;
    uint16_t fragment_size;
    uint8_t flags; // bit 0: keyframe, bit 1: last fragment
    uint8_t reserved;
} __attribute__((packed));

struct ReceivedFrame {
    std::vector<uint8_t> data;
    uint32_t timestamp;
    uint32_t sequence;
    bool is_keyframe;
    bool is_complete;
};

using FrameReceivedCallback = std::function<void(const decoder::EncodedPacket& packet)>;

class UDPReceiver {
public:
    struct Config {
        uint16_t listen_port = 5000;
        std::string bind_address = "0.0.0.0";
        uint32_t receive_buffer_size = 1024 * 1024; // 1MB
        uint32_t timeout_ms = 100;
        uint32_t max_frame_size = 1024 * 1024; // 1MB max frame
        uint32_t fragment_timeout_ms = 1000;
    };

    explicit UDPReceiver(const Config& config);
    ~UDPReceiver();

    // Non-copyable, movable
    UDPReceiver(const UDPReceiver&) = delete;
    UDPReceiver& operator=(const UDPReceiver&) = delete;
    UDPReceiver(UDPReceiver&&) = default;
    UDPReceiver& operator=(UDPReceiver&&) = default;

    bool initialize();
    void shutdown();
    bool is_initialized() const;

    void start_receiving();
    void stop_receiving();
    bool is_receiving() const;

    void set_frame_received_callback(FrameReceivedCallback callback);

    // Statistics
    struct Stats {
        uint64_t packets_received;
        uint64_t frames_received;
        uint64_t frames_dropped;
        uint64_t fragments_received;
        uint64_t fragment_timeouts;
        uint64_t bytes_received;
        double current_bitrate;
        double packet_loss_rate;
    };
    
    Stats get_stats() const;

private:
    Config config_;
    std::atomic<bool> initialized_;
    std::atomic<bool> receiving_;
    std::atomic<bool> should_stop_;
    
    // Socket
    int socket_fd_;
    sockaddr_in bind_addr_;
    
    // Threading
    std::thread receiver_thread_;
    
    // Frame reassembly
    struct FrameAssembly {
        std::vector<uint8_t> data;
        std::vector<bool> fragment_received;
        uint32_t timestamp;
        uint32_t sequence;
        uint16_t total_fragments;
        bool is_keyframe;
        std::chrono::steady_clock::time_point last_update;
        
        FrameAssembly() : total_fragments(0), is_keyframe(false) {}
    };
    
    std::map<uint32_t, FrameAssembly> frame_assemblies_; // sequence -> assembly
    std::mutex assembly_mutex_;
    
    // Callback
    FrameReceivedCallback frame_callback_;
    std::mutex callback_mutex_;
    
    // Statistics
    mutable std::mutex stats_mutex_;
    Stats stats_;
    std::chrono::steady_clock::time_point last_stats_update_;
    uint32_t last_sequence_;
    uint32_t expected_sequence_;
    
    bool setup_socket();
    void receiver_loop();
    bool process_packet(const uint8_t* data, size_t size);
    bool handle_frame_fragment(const FrameHeader& header, const uint8_t* payload, size_t payload_size);
    void check_frame_timeouts();
    void complete_frame(uint32_t sequence);
    void cleanup_old_assemblies();
    void update_stats(size_t bytes_received);
    void cleanup();
};

} // namespace network
