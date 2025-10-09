#include "network/command_sender.h"
#include "utils/logger.h"

#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <chrono>

namespace network {

CommandSender::CommandSender(const Config& config)
    : config_(config), initialized_(false), sending_(false), should_stop_(false), socket_fd_(-1) {
    memset(&stats_, 0, sizeof(stats_));
    memset(&target_addr_, 0, sizeof(target_addr_));
    last_stats_update_ = std::chrono::steady_clock::now();
    last_heartbeat_ = std::chrono::steady_clock::now();
}

CommandSender::~CommandSender() {
    shutdown();
}

bool CommandSender::initialize() {
    LOG_INFO("Initializing command sender, target: " + config_.target_ip + ":" + std::to_string(config_.target_port));
    
    if (initialized_) {
        LOG_WARNING("Command sender already initialized");
        return true;
    }
    
    if (!setup_socket()) {
        LOG_ERROR("Failed to setup UDP socket");
        return false;
    }
    
    initialized_ = true;
    LOG_INFO("Command sender initialized successfully");
    return true;
}

void CommandSender::shutdown() {
    if (!initialized_) {
        return;
    }
    
    LOG_INFO("Shutting down command sender");
    
    stop_sending();
    cleanup();
    
    initialized_ = false;
    LOG_INFO("Command sender shutdown complete");
}

bool CommandSender::is_initialized() const {
    return initialized_;
}

void CommandSender::start_sending() {
    if (!initialized_ || sending_) {
        return;
    }
    
    should_stop_ = false;
    sender_thread_ = std::thread(&CommandSender::sender_loop, this);
    sending_ = true;
    
    LOG_INFO("Command sender started");
}

void CommandSender::stop_sending() {
    if (!sending_) {
        return;
    }
    
    LOG_INFO("Stopping command sender");
    should_stop_ = true;
    queue_cv_.notify_all();
    
    if (sender_thread_.joinable()) {
        sender_thread_.join();
    }
    
    sending_ = false;
    LOG_INFO("Command sender stopped");
}

bool CommandSender::is_sending() const {
    return sending_;
}

bool CommandSender::send_servo_command(uint8_t servo_id, uint16_t position, uint8_t speed) {
    if (!initialized_) {
        return false;
    }
    
    uint8_t params[4];
    params[0] = servo_id;
    *reinterpret_cast<uint16_t*>(&params[1]) = position;
    params[3] = speed;
    
    return send_raw_command(CommandOpcode::SERVO_MOVE, params, 4);
}

bool CommandSender::send_multi_servo_command(const std::vector<ServoPosition>& servos) {
    if (!initialized_ || servos.empty() || servos.size() > 10) { // Limit to 10 servos per command
        return false;
    }
    
    uint8_t params[32];
    params[0] = static_cast<uint8_t>(servos.size());
    
    size_t offset = 1;
    for (const auto& servo : servos) {
        if (offset + 3 > sizeof(params)) {
            break; // Not enough space
        }
        
        params[offset] = servo.servo_id;
        *reinterpret_cast<uint16_t*>(&params[offset + 1]) = servo.position;
        offset += 3;
    }
    
    return send_raw_command(CommandOpcode::SERVO_MULTI, params, offset);
}

bool CommandSender::send_system_shutdown() {
    if (!initialized_) {
        return false;
    }
    
    return send_raw_command(CommandOpcode::SYSTEM_SHUTDOWN);
}

bool CommandSender::send_force_iframe() {
    if (!initialized_) {
        return false;
    }
    
    return send_raw_command(CommandOpcode::FORCE_IFRAME);
}

bool CommandSender::send_set_bitrate(uint32_t bitrate) {
    if (!initialized_) {
        return false;
    }
    
    uint8_t params[4];
    *reinterpret_cast<uint32_t*>(params) = bitrate;
    
    return send_raw_command(CommandOpcode::SET_BITRATE, params, 4);
}

bool CommandSender::send_heartbeat() {
    if (!initialized_) {
        return false;
    }
    
    bool success = send_raw_command(CommandOpcode::HEARTBEAT);
    if (success) {
        last_heartbeat_ = std::chrono::steady_clock::now();
        
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        stats_.heartbeats_sent++;
    }
    
    return success;
}

void CommandSender::queue_servo_command(uint8_t servo_id, uint16_t position, uint8_t speed) {
    if (!initialized_) {
        return;
    }
    
    SimpleCommand command;
    command.opcode = CommandOpcode::SERVO_MOVE;
    command.length = 4;
    command.params[0] = servo_id;
    *reinterpret_cast<uint16_t*>(&command.params[1]) = position;
    command.params[3] = speed;
    
    std::unique_lock<std::mutex> lock(queue_mutex_);
    
    // Drop commands if queue is full
    if (command_queue_.size() >= MAX_QUEUE_SIZE) {
        command_queue_.pop();
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        stats_.commands_failed++;
    }
    
    command_queue_.push(command);
    queue_cv_.notify_one();
}

void CommandSender::flush_queued_commands() {
    queue_cv_.notify_all();
}

CommandSender::Stats CommandSender::get_stats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    Stats stats = stats_;
    stats.connection_active = initialized_ && sending_;
    return stats;
}

bool CommandSender::setup_socket() {
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
    target_addr_.sin_family = AF_INET;
    target_addr_.sin_port = htons(config_.target_port);
    
    if (inet_pton(AF_INET, config_.target_ip.c_str(), &target_addr_.sin_addr) <= 0) {
        LOG_ERROR("Invalid target IP address: " + config_.target_ip);
        return false;
    }
    
    return true;
}

void CommandSender::sender_loop() {
    LOG_INFO("Command sender loop started");
    
    while (!should_stop_) {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        
        // Wait for commands or timeout for heartbeat
        auto timeout = std::chrono::milliseconds(config_.command_timeout_ms);
        queue_cv_.wait_for(lock, timeout, [this] { return !command_queue_.empty() || should_stop_; });
        
        if (should_stop_) {
            break;
        }
        
        // Send queued commands
        while (!command_queue_.empty() && !should_stop_) {
            SimpleCommand command = command_queue_.front();
            command_queue_.pop();
            lock.unlock();
            
            if (!send_command(command)) {
                std::lock_guard<std::mutex> stats_lock(stats_mutex_);
                stats_.commands_failed++;
            }
            
            lock.lock();
        }
        
        lock.unlock();
        
        // Send periodic heartbeat
        if (config_.enable_heartbeat) {
            send_periodic_heartbeat();
        }
    }
    
    LOG_INFO("Command sender loop ended");
}

bool CommandSender::send_command(const SimpleCommand& command) {
    size_t command_size = sizeof(SimpleCommand) - sizeof(SimpleCommand::params) + command.length;
    
    ssize_t sent = sendto(socket_fd_, &command, command_size, 0,
                         reinterpret_cast<const sockaddr*>(&target_addr_),
                         sizeof(target_addr_));
    
    if (sent < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            LOG_ERROR("Send error: " + std::string(strerror(errno)));
        }
        return false;
    }
    
    if (static_cast<size_t>(sent) != command_size) {
        LOG_WARNING("Partial send: " + std::to_string(sent) + "/" + std::to_string(command_size));
        return false;
    }
    
    update_stats(true, sent);
    return true;
}

bool CommandSender::send_raw_command(CommandOpcode opcode, const uint8_t* params, uint8_t length) {
    if (!initialized_) {
        return false;
    }
    
    SimpleCommand command;
    command.opcode = opcode;
    command.length = length;
    
    if (params && length > 0) {
        memcpy(command.params, params, std::min(length, static_cast<uint8_t>(sizeof(command.params))));
    }
    
    return send_command(command);
}

void CommandSender::send_periodic_heartbeat() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_heartbeat_).count();
    
    if (elapsed >= config_.heartbeat_interval_ms) {
        send_heartbeat();
    }
}

void CommandSender::update_stats(bool command_sent, size_t bytes_sent) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    
    if (command_sent) {
        stats_.commands_sent++;
        stats_.bytes_sent += bytes_sent;
    }
    
    // Calculate send rate
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_stats_update_).count();
    
    if (elapsed >= 1000) { // Update every second
        static uint64_t last_commands = 0;
        uint64_t commands_diff = stats_.commands_sent - last_commands;
        stats_.send_rate = commands_diff * 1000.0 / elapsed;
        
        last_stats_update_ = now;
        last_commands = stats_.commands_sent;
    }
}

void CommandSender::cleanup() {
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
    
    // Clear command queue
    std::lock_guard<std::mutex> lock(queue_mutex_);
    while (!command_queue_.empty()) {
        command_queue_.pop();
    }
}

} // namespace network
