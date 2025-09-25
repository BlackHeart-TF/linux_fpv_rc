#include "network/udp_streamer.h"
#include "utils/logger.h"

#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <chrono>

namespace network {

UDPStreamer::UDPStreamer(const Config& config)
    : config_(config), socket_fd_(-1), streaming_(false), should_stop_(false) {
    memset(&stats_, 0, sizeof(stats_));
}

UDPStreamer::~UDPStreamer() {
    stop_streaming();
    cleanup();
}

bool UDPStreamer::initialize() {
    LOG_INFO("Initializing UDP streamer, target: " + config_.target_ip + ":" + std::to_string(config_.target_port));
    
    if (!setup_socket()) {
        LOG_ERROR("Failed to setup UDP socket");
        return false;
    }
    
    LOG_INFO("UDP streamer initialized successfully");
    return true;
}

void UDPStreamer::start_streaming() {
    if (streaming_) {
        LOG_WARNING("UDP streamer is already running");
        return;
    }
    
    should_stop_ = false;
    sender_thread_ = std::thread(&UDPStreamer::sender_loop, this);
    streaming_ = true;
    
    LOG_INFO("UDP streaming started");
}

void UDPStreamer::stop_streaming() {
    if (!streaming_) {
        return;
    }
    
    LOG_INFO("Stopping UDP streaming");
    should_stop_ = true;
    queue_cv_.notify_all();
    
    if (sender_thread_.joinable()) {
        sender_thread_.join();
    }
    
    streaming_ = false;
    LOG_INFO("UDP streaming stopped");
}

bool UDPStreamer::is_streaming() const {
    return streaming_;
}

void UDPStreamer::queue_frame(const encoder::EncodedFrame& frame) {
    if (!streaming_) {
        return;
    }
    
    std::unique_lock<std::mutex> lock(queue_mutex_);
    
    // Drop frames if queue is full
    if (frame_queue_.size() >= MAX_QUEUE_SIZE) {
        frame_queue_.pop();
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        stats_.packets_dropped++;
    }
    
    frame_queue_.push(frame);
    queue_cv_.notify_one();
}

UDPStreamer::Stats UDPStreamer::get_stats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

bool UDPStreamer::setup_socket() {
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        LOG_ERROR("Failed to create socket: " + std::string(strerror(errno)));
        return false;
    }
    
    // Set send buffer size
    int send_buffer_size = config_.send_buffer_size;
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_SNDBUF, &send_buffer_size, sizeof(send_buffer_size)) < 0) {
        LOG_WARNING("Failed to set send buffer size: " + std::string(strerror(errno)));
    }
    
    // Setup target address
    memset(&target_addr_, 0, sizeof(target_addr_));
    target_addr_.sin_family = AF_INET;
    target_addr_.sin_port = htons(config_.target_port);
    
    if (inet_pton(AF_INET, config_.target_ip.c_str(), &target_addr_.sin_addr) <= 0) {
        LOG_ERROR("Invalid target IP address: " + config_.target_ip);
        return false;
    }
    
    return true;
}

void UDPStreamer::sender_loop() {
    LOG_INFO("UDP sender loop started");
    
    while (!should_stop_) {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        
        // Wait for frames or stop signal
        queue_cv_.wait(lock, [this] { return !frame_queue_.empty() || should_stop_; });
        
        if (should_stop_) {
            break;
        }
        
        if (frame_queue_.empty()) {
            continue;
        }
        
        // Get frame from queue
        encoder::EncodedFrame frame = frame_queue_.front();
        frame_queue_.pop();
        lock.unlock();
        
        // Send frame
        if (!send_frame(frame)) {
            LOG_ERROR("Failed to send frame");
            std::lock_guard<std::mutex> stats_lock(stats_mutex_);
            stats_.packets_dropped++;
        }
    }
    
    LOG_INFO("UDP sender loop ended");
}

bool UDPStreamer::send_frame(const encoder::EncodedFrame& frame) {
    static uint32_t sequence_number = 0;
    sequence_number++;
    
    if (!config_.enable_fragmentation || frame.size <= config_.mtu) {
        // Send as single packet
        FrameHeader header;
        header.timestamp = static_cast<uint32_t>(frame.timestamp_us);
        header.sequence = sequence_number;
        header.fragment_id = 0;
        header.fragment_offset = 0;
        header.fragment_size = static_cast<uint16_t>(frame.size);
        header.flags = frame.is_keyframe ? 0x01 : 0x00;
        header.flags |= 0x02; // Last fragment
        header.reserved = 0;
        
        // Create packet with header + data
        std::vector<uint8_t> packet(sizeof(FrameHeader) + frame.size);
        memcpy(packet.data(), &header, sizeof(FrameHeader));
        memcpy(packet.data() + sizeof(FrameHeader), frame.data, frame.size);
        
        bool success = send_packet(packet.data(), packet.size());
        if (success) {
            update_stats(frame.size, true);
            std::lock_guard<std::mutex> stats_lock(stats_mutex_);
            stats_.frames_sent++;
        }
        return success;
        
    } else {
        // Fragment large frames
        size_t max_payload = config_.mtu - sizeof(FrameHeader);
        size_t offset = 0;
        uint16_t fragment_id = 0;
        bool all_sent = true;
        
        while (offset < frame.size) {
            size_t fragment_size = std::min(max_payload, frame.size - offset);
            bool is_last = (offset + fragment_size >= frame.size);
            
            FrameHeader header;
            header.timestamp = static_cast<uint32_t>(frame.timestamp_us);
            header.sequence = sequence_number;
            header.fragment_id = fragment_id++;
            header.fragment_offset = static_cast<uint16_t>(offset);
            header.fragment_size = static_cast<uint16_t>(fragment_size);
            header.flags = frame.is_keyframe ? 0x01 : 0x00;
            if (is_last) header.flags |= 0x02; // Last fragment
            header.reserved = 0;
            
            // Create fragment packet
            std::vector<uint8_t> packet(sizeof(FrameHeader) + fragment_size);
            memcpy(packet.data(), &header, sizeof(FrameHeader));
            memcpy(packet.data() + sizeof(FrameHeader), frame.data + offset, fragment_size);
            
            if (!send_packet(packet.data(), packet.size(), true, fragment_id, offset)) {
                all_sent = false;
            } else {
                update_stats(fragment_size, true);
            }
            
            offset += fragment_size;
        }
        
        if (all_sent) {
            std::lock_guard<std::mutex> stats_lock(stats_mutex_);
            stats_.frames_sent++;
        }
        
        return all_sent;
    }
}

bool UDPStreamer::send_packet(const uint8_t* data, size_t size, bool is_fragment, 
                              uint16_t fragment_id, uint16_t fragment_offset) {
    ssize_t sent = sendto(socket_fd_, data, size, 0, 
                         reinterpret_cast<const sockaddr*>(&target_addr_), 
                         sizeof(target_addr_));
    
    if (sent < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            LOG_ERROR("Send error: " + std::string(strerror(errno)));
        }
        return false;
    }
    
    if (static_cast<size_t>(sent) != size) {
        LOG_WARNING("Partial send: " + std::to_string(sent) + "/" + std::to_string(size));
        return false;
    }
    
    return true;
}

void UDPStreamer::update_stats(size_t bytes_sent, bool packet_sent) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    
    if (packet_sent) {
        stats_.packets_sent++;
    }
    
    stats_.bytes_sent += bytes_sent;
    
    // Calculate average bitrate over last second
    static auto last_time = std::chrono::steady_clock::now();
    static uint64_t last_bytes = 0;
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
    
    if (elapsed >= 1000) { // Update every second
        uint64_t bytes_diff = stats_.bytes_sent - last_bytes;
        stats_.average_bitrate = (bytes_diff * 8.0) / (elapsed / 1000.0); // bits per second
        
        last_time = now;
        last_bytes = stats_.bytes_sent;
    }
}

void UDPStreamer::cleanup() {
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
    
    // Clear any remaining frames in queue
    std::lock_guard<std::mutex> lock(queue_mutex_);
    while (!frame_queue_.empty()) {
        frame_queue_.pop();
    }
}

} // namespace network
