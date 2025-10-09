#include "network/udp_receiver.h"
#include "utils/logger.h"

#include <arpa/inet.h>
#include <unistd.h>
#include <poll.h>
#include <cstring>
#include <cerrno>
#include <chrono>

namespace network {

UDPReceiver::UDPReceiver(const Config& config)
    : config_(config), initialized_(false), receiving_(false), should_stop_(false),
      socket_fd_(-1), last_sequence_(0), expected_sequence_(0) {
    memset(&stats_, 0, sizeof(stats_));
    memset(&bind_addr_, 0, sizeof(bind_addr_));
    last_stats_update_ = std::chrono::steady_clock::now();
}

UDPReceiver::~UDPReceiver() {
    shutdown();
}

bool UDPReceiver::initialize() {
    LOG_INFO("Initializing UDP receiver on port " + std::to_string(config_.listen_port));
    
    if (initialized_) {
        LOG_WARNING("UDP receiver already initialized");
        return true;
    }
    
    if (!setup_socket()) {
        LOG_ERROR("Failed to setup UDP socket");
        return false;
    }
    
    initialized_ = true;
    LOG_INFO("UDP receiver initialized successfully");
    return true;
}

void UDPReceiver::shutdown() {
    if (!initialized_) {
        return;
    }
    
    LOG_INFO("Shutting down UDP receiver");
    
    stop_receiving();
    cleanup();
    
    initialized_ = false;
    LOG_INFO("UDP receiver shutdown complete");
}

bool UDPReceiver::is_initialized() const {
    return initialized_;
}

void UDPReceiver::start_receiving() {
    if (!initialized_ || receiving_) {
        return;
    }
    
    should_stop_ = false;
    receiver_thread_ = std::thread(&UDPReceiver::receiver_loop, this);
    receiving_ = true;
    
    LOG_INFO("UDP receiver started");
}

void UDPReceiver::stop_receiving() {
    if (!receiving_) {
        return;
    }
    
    LOG_INFO("Stopping UDP receiver");
    should_stop_ = true;
    
    if (receiver_thread_.joinable()) {
        receiver_thread_.join();
    }
    
    receiving_ = false;
    LOG_INFO("UDP receiver stopped");
}

bool UDPReceiver::is_receiving() const {
    return receiving_;
}

void UDPReceiver::set_frame_received_callback(FrameReceivedCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    frame_callback_ = callback;
}

UDPReceiver::Stats UDPReceiver::get_stats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

bool UDPReceiver::setup_socket() {
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        LOG_ERROR("Failed to create socket: " + std::string(strerror(errno)));
        return false;
    }
    
    // Set receive buffer size
    int recv_buffer_size = config_.receive_buffer_size;
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF, &recv_buffer_size, sizeof(recv_buffer_size)) < 0) {
        LOG_WARNING("Failed to set receive buffer size: " + std::string(strerror(errno)));
    }
    
    // Enable address reuse
    int reuse = 1;
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        LOG_WARNING("Failed to set SO_REUSEADDR: " + std::string(strerror(errno)));
    }
    
    // Setup bind address
    bind_addr_.sin_family = AF_INET;
    bind_addr_.sin_port = htons(config_.listen_port);
    
    if (config_.bind_address == "0.0.0.0") {
        bind_addr_.sin_addr.s_addr = INADDR_ANY;
    } else {
        if (inet_pton(AF_INET, config_.bind_address.c_str(), &bind_addr_.sin_addr) <= 0) {
            LOG_ERROR("Invalid bind address: " + config_.bind_address);
            return false;
        }
    }
    
    // Bind socket
    if (bind(socket_fd_, reinterpret_cast<const sockaddr*>(&bind_addr_), sizeof(bind_addr_)) < 0) {
        LOG_ERROR("Failed to bind socket: " + std::string(strerror(errno)));
        return false;
    }
    
    return true;
}

void UDPReceiver::receiver_loop() {
    LOG_INFO("UDP receiver loop started");
    
    std::vector<uint8_t> buffer(config_.max_frame_size);
    auto last_timeout_check = std::chrono::steady_clock::now();
    
    int packet_count = 0;
    
    while (!should_stop_) {
        // Use poll for timeout
        pollfd pfd = {};
        pfd.fd = socket_fd_;
        pfd.events = POLLIN;
        
        int ret = poll(&pfd, 1, config_.timeout_ms);
        if (ret < 0) {
            if (errno != EINTR) {
                LOG_ERROR("Poll error: " + std::string(strerror(errno)));
                break;
            }
            continue;
        }
        
        if (ret == 0) {
            // Timeout - check for fragment timeouts
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_timeout_check).count() >= 100) {
                check_frame_timeouts();
                cleanup_old_assemblies();
                last_timeout_check = now;
            }
            continue;
        }
        
        if (!(pfd.revents & POLLIN)) {
            continue;
        }
        
        // Receive packet
        sockaddr_in sender_addr;
        socklen_t sender_len = sizeof(sender_addr);
        ssize_t received = recvfrom(socket_fd_, buffer.data(), buffer.size(), 0,
                                   reinterpret_cast<sockaddr*>(&sender_addr), &sender_len);
        
        if (received > 0) {
            packet_count++;
            if (packet_count <= 10 || packet_count % 100 == 1) {  // Print first 10 packets, then every 100th
                LOG_INFO("Received packet " + std::to_string(packet_count) + ", size: " + std::to_string(received));
            }
        }
        
        if (received < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                LOG_ERROR("Receive error: " + std::string(strerror(errno)));
            }
            continue;
        }
        
        if (received < static_cast<ssize_t>(sizeof(FrameHeader))) {
            LOG_WARNING("Received packet too small: " + std::to_string(received));
            continue;
        }
        
        // Process packet
        if (process_packet(buffer.data(), received)) {
            update_stats(received);
        }
    }
    
    LOG_INFO("UDP receiver loop ended");
}

bool UDPReceiver::process_packet(const uint8_t* data, size_t size) {
    if (size < sizeof(FrameHeader)) {
        return false;
    }
    
    const FrameHeader* header = reinterpret_cast<const FrameHeader*>(data);
    const uint8_t* payload = data + sizeof(FrameHeader);
    size_t payload_size = size - sizeof(FrameHeader);
    
    // Validate header
    if (header->fragment_size != payload_size) {
        LOG_WARNING("Fragment size mismatch: header=" + std::to_string(header->fragment_size) + 
                   " actual=" + std::to_string(payload_size));
        return false;
    }
    
    std::lock_guard<std::mutex> stats_lock(stats_mutex_);
    stats_.packets_received++;
    stats_.fragments_received++;
    
    return handle_frame_fragment(*header, payload, payload_size);
}

bool UDPReceiver::handle_frame_fragment(const FrameHeader& header, const uint8_t* payload, size_t payload_size) {
    std::lock_guard<std::mutex> lock(assembly_mutex_);
    
    uint32_t sequence = header.sequence;
    auto& assembly = frame_assemblies_[sequence];
    
    // Initialize assembly if this is the first fragment
    if (assembly.data.empty()) {
        assembly.timestamp = header.timestamp;
        assembly.sequence = sequence;
        assembly.is_keyframe = (header.flags & 0x01) != 0;
        assembly.last_update = std::chrono::steady_clock::now();
        
        // Estimate total size (this is a rough estimate)
        size_t estimated_size = payload_size * 10; // Assume max 10 fragments
        assembly.data.reserve(estimated_size);
    }
    
    // Update last activity time
    assembly.last_update = std::chrono::steady_clock::now();
    
    // Handle fragment offset and data
    uint16_t offset = header.fragment_offset;
    
    // Ensure data buffer is large enough
    size_t required_size = offset + payload_size;
    if (assembly.data.size() < required_size) {
        assembly.data.resize(required_size);
    }
    
    // Copy fragment data
    memcpy(assembly.data.data() + offset, payload, payload_size);
    
    // Mark fragment as received
    uint16_t fragment_id = header.fragment_id;
    if (assembly.fragment_received.size() <= fragment_id) {
        assembly.fragment_received.resize(fragment_id + 1, false);
    }
    assembly.fragment_received[fragment_id] = true;
    
    // Check if this is the last fragment
    if (header.flags & 0x02) { // Last fragment flag
        assembly.total_fragments = fragment_id + 1;
        
        // Check if frame is complete
        bool complete = true;
        for (uint16_t i = 0; i < assembly.total_fragments; i++) {
            if (i >= assembly.fragment_received.size() || !assembly.fragment_received[i]) {
                complete = false;
                break;
            }
        }
        
        if (complete) {
            complete_frame(sequence);
        }
    }
    
    return true;
}

void UDPReceiver::complete_frame(uint32_t sequence) {
    auto it = frame_assemblies_.find(sequence);
    if (it == frame_assemblies_.end()) {
        return;
    }
    
    FrameAssembly& assembly = it->second;
    
    // Create encoded packet
    decoder::EncodedPacket packet(
        assembly.data.data(),
        assembly.data.size(),
        assembly.timestamp,
        assembly.is_keyframe
    );
    
    LOG_INFO("Complete frame assembled: seq=" + std::to_string(sequence) + 
             ", size=" + std::to_string(assembly.data.size()) + 
             ", keyframe=" + (assembly.is_keyframe ? "true" : "false"));
    
    // Only deliver keyframes or frames after we've seen a keyframe
    static bool seen_keyframe = false;
    if (assembly.is_keyframe) {
        seen_keyframe = true;
        LOG_INFO("Keyframe received! Starting video decode.");
    }
    
    if (seen_keyframe) {
        // Deliver frame
        std::lock_guard<std::mutex> callback_lock(callback_mutex_);
        if (frame_callback_) {
            frame_callback_(packet);
        }
    } else {
        LOG_INFO("Skipping frame (waiting for keyframe)");
    }
    
    // Update statistics
    {
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        stats_.frames_received++;
        
        // Check for packet loss
        if (expected_sequence_ != 0 && sequence != expected_sequence_) {
            uint32_t lost_packets = sequence - expected_sequence_;
            stats_.frames_dropped += lost_packets;
        }
        expected_sequence_ = sequence + 1;
    }
    
    // Remove completed assembly
    frame_assemblies_.erase(it);
    
    LOG_DEBUG("Completed frame: sequence=" + std::to_string(sequence) + 
             " size=" + std::to_string(assembly.data.size()) + 
             " keyframe=" + (assembly.is_keyframe ? "true" : "false"));
}

void UDPReceiver::check_frame_timeouts() {
    std::lock_guard<std::mutex> lock(assembly_mutex_);
    
    auto now = std::chrono::steady_clock::now();
    auto timeout_threshold = std::chrono::milliseconds(config_.fragment_timeout_ms);
    
    auto it = frame_assemblies_.begin();
    while (it != frame_assemblies_.end()) {
        if (now - it->second.last_update > timeout_threshold) {
            LOG_WARNING("Frame assembly timeout: sequence=" + std::to_string(it->first));
            
            std::lock_guard<std::mutex> stats_lock(stats_mutex_);
            stats_.fragment_timeouts++;
            stats_.frames_dropped++;
            
            it = frame_assemblies_.erase(it);
        } else {
            ++it;
        }
    }
}

void UDPReceiver::cleanup_old_assemblies() {
    std::lock_guard<std::mutex> lock(assembly_mutex_);
    
    // Keep only the most recent assemblies to prevent memory growth
    const size_t max_assemblies = 100;
    
    if (frame_assemblies_.size() > max_assemblies) {
        // Remove oldest assemblies (lowest sequence numbers)
        auto it = frame_assemblies_.begin();
        size_t to_remove = frame_assemblies_.size() - max_assemblies;
        
        for (size_t i = 0; i < to_remove && it != frame_assemblies_.end(); i++) {
            LOG_DEBUG("Cleaning up old assembly: sequence=" + std::to_string(it->first));
            it = frame_assemblies_.erase(it);
        }
    }
}

void UDPReceiver::update_stats(size_t bytes_received) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    
    stats_.bytes_received += bytes_received;
    
    // Calculate bitrate and packet loss rate
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_stats_update_).count();
    
    if (elapsed >= 1000) { // Update every second
        static uint64_t last_bytes = 0;
        static uint64_t last_packets = 0;
        static uint64_t last_dropped = 0;
        
        uint64_t bytes_diff = stats_.bytes_received - last_bytes;
        uint64_t packets_diff = stats_.packets_received - last_packets;
        uint64_t dropped_diff = stats_.frames_dropped - last_dropped;
        
        stats_.current_bitrate = (bytes_diff * 8.0) / (elapsed / 1000.0); // bits per second
        
        if (packets_diff > 0) {
            stats_.packet_loss_rate = (dropped_diff * 100.0) / (packets_diff + dropped_diff);
        }
        
        last_stats_update_ = now;
        last_bytes = stats_.bytes_received;
        last_packets = stats_.packets_received;
        last_dropped = stats_.frames_dropped;
    }
}

void UDPReceiver::cleanup() {
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
    
    std::lock_guard<std::mutex> lock(assembly_mutex_);
    frame_assemblies_.clear();
}

} // namespace network
