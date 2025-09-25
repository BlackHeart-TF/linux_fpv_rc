#include "network/command_receiver.h"
#include "utils/logger.h"

#include <arpa/inet.h>
#include <unistd.h>
#include <poll.h>
#include <cstring>
#include <cerrno>
#include <chrono>

namespace network {

CommandReceiver::CommandReceiver(const Config& config)
    : config_(config), socket_fd_(-1), listening_(false), should_stop_(false), 
      connected_(false), last_heartbeat_time_(0) {
    memset(&stats_, 0, sizeof(stats_));
    memset(&bind_addr_, 0, sizeof(bind_addr_));
    memset(&sender_addr_, 0, sizeof(sender_addr_));
}

CommandReceiver::~CommandReceiver() {
    stop_listening();
    cleanup();
}

bool CommandReceiver::initialize() {
    LOG_INFO("Initializing command receiver on port " + std::to_string(config_.listen_port));
    
    if (!setup_socket()) {
        LOG_ERROR("Failed to setup UDP socket for command receiver");
        return false;
    }
    
    LOG_INFO("Command receiver initialized successfully");
    return true;
}

void CommandReceiver::start_listening() {
    if (listening_) {
        LOG_WARNING("Command receiver is already listening");
        return;
    }
    
    should_stop_ = false;
    receiver_thread_ = std::thread(&CommandReceiver::receiver_loop, this);
    listening_ = true;
    
    LOG_INFO("Command receiver started listening");
}

void CommandReceiver::stop_listening() {
    if (!listening_) {
        return;
    }
    
    LOG_INFO("Stopping command receiver");
    should_stop_ = true;
    
    if (receiver_thread_.joinable()) {
        receiver_thread_.join();
    }
    
    listening_ = false;
    connected_ = false;
    LOG_INFO("Command receiver stopped");
}

bool CommandReceiver::is_listening() const {
    return listening_;
}

void CommandReceiver::set_servo_command_callback(ServoCommandCallback callback) {
    servo_callback_ = callback;
}

void CommandReceiver::set_multi_servo_command_callback(MultiServoCommandCallback callback) {
    multi_servo_callback_ = callback;
}

void CommandReceiver::set_system_command_callback(SystemCommandCallback callback) {
    system_callback_ = callback;
}

bool CommandReceiver::is_connected() const {
    return connected_;
}

uint64_t CommandReceiver::get_last_heartbeat_time() const {
    return last_heartbeat_time_;
}

CommandReceiver::Stats CommandReceiver::get_stats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

bool CommandReceiver::setup_socket() {
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        LOG_ERROR("Failed to create socket: " + std::string(strerror(errno)));
        return false;
    }
    
    // Set socket to non-blocking for timeout support
    int flags = fcntl(socket_fd_, F_GETFL, 0);
    if (flags < 0 || fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
        LOG_WARNING("Failed to set socket non-blocking: " + std::string(strerror(errno)));
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

void CommandReceiver::receiver_loop() {
    LOG_INFO("Command receiver loop started");
    
    uint8_t buffer[sizeof(SimpleCommand)];
    
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
            // Timeout - check heartbeat
            if (config_.enable_heartbeat) {
                update_connection_status();
            }
            continue;
        }
        
        if (!(pfd.revents & POLLIN)) {
            continue;
        }
        
        // Receive packet
        socklen_t sender_len = sizeof(sender_addr_);
        ssize_t received = recvfrom(socket_fd_, buffer, sizeof(buffer), 0,
                                   reinterpret_cast<sockaddr*>(&sender_addr_), &sender_len);
        
        if (received < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                LOG_ERROR("Receive error: " + std::string(strerror(errno)));
            }
            continue;
        }
        
        if (received < static_cast<ssize_t>(sizeof(SimpleCommand) - sizeof(SimpleCommand::params))) {
            LOG_WARNING("Received packet too small: " + std::to_string(received));
            std::lock_guard<std::mutex> stats_lock(stats_mutex_);
            stats_.invalid_packets++;
            continue;
        }
        
        // Process packet
        if (!process_packet(buffer, received, sender_addr_)) {
            std::lock_guard<std::mutex> stats_lock(stats_mutex_);
            stats_.invalid_packets++;
        } else {
            std::lock_guard<std::mutex> stats_lock(stats_mutex_);
            stats_.packets_received++;
        }
        
        // Update connection status
        if (config_.enable_heartbeat) {
            update_connection_status();
        }
    }
    
    LOG_INFO("Command receiver loop ended");
}

bool CommandReceiver::process_packet(const uint8_t* data, size_t size, const sockaddr_in& sender) {
    if (size < sizeof(SimpleCommand) - sizeof(SimpleCommand::params)) {
        return false;
    }
    
    const SimpleCommand* command = reinterpret_cast<const SimpleCommand*>(data);
    
    // Validate command
    if (!validate_command(*command, size)) {
        return false;
    }
    
    // Update heartbeat time
    auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    last_heartbeat_time_ = now;
    connected_ = true;
    
    // Process command
    return process_simple_command(*command);
}

bool CommandReceiver::validate_command(const SimpleCommand& command, size_t total_size) {
    // Check if we have enough data for the command length
    size_t expected_size = sizeof(SimpleCommand) - sizeof(SimpleCommand::params) + command.length;
    if (total_size < expected_size) {
        LOG_WARNING("Command packet truncated");
        return false;
    }
    
    // Check parameter length is reasonable
    if (command.length > sizeof(SimpleCommand::params)) {
        LOG_WARNING("Command parameter length too large: " + std::to_string(command.length));
        return false;
    }
    
    return true;
}

bool CommandReceiver::process_simple_command(const SimpleCommand& command) {
    switch (command.opcode) {
        case CommandOpcode::SERVO_MOVE:
            if (command.length >= 4 && servo_callback_) {  // servo_id(1) + position(2) + speed(1)
                uint8_t servo_id = command.params[0];
                uint16_t position = *reinterpret_cast<const uint16_t*>(&command.params[1]);
                uint8_t speed = command.params[3];
                
                if (servo_id < MAX_SERVO_COUNT) {
                    servo_callback_(servo_id, position, speed);
                    std::lock_guard<std::mutex> stats_lock(stats_mutex_);
                    stats_.servo_commands_processed++;
                } else {
                    LOG_WARNING("Invalid servo ID: " + std::to_string(servo_id));
                    return false;
                }
            }
            break;
            
        case CommandOpcode::SERVO_MULTI:
            if (command.length >= 1 && multi_servo_callback_) {
                uint8_t count = command.params[0];
                std::vector<ServoPosition> servos;
                
                size_t offset = 1;
                for (uint8_t i = 0; i < count && offset + 3 <= command.length; i++) {
                    ServoPosition servo;
                    servo.servo_id = command.params[offset];
                    servo.position = *reinterpret_cast<const uint16_t*>(&command.params[offset + 1]);
                    
                    if (servo.servo_id < MAX_SERVO_COUNT) {
                        servos.push_back(servo);
                    }
                    offset += 3;
                }
                
                if (!servos.empty()) {
                    multi_servo_callback_(servos);
                    std::lock_guard<std::mutex> stats_lock(stats_mutex_);
                    stats_.servo_commands_processed++;
                }
            }
            break;
            
        case CommandOpcode::SYSTEM_SHUTDOWN:
        case CommandOpcode::FORCE_IFRAME:
        case CommandOpcode::SET_BITRATE:
        case CommandOpcode::HEARTBEAT:
            if (system_callback_) {
                system_callback_(command.opcode, command.params, command.length);
                std::lock_guard<std::mutex> stats_lock(stats_mutex_);
                stats_.system_commands_processed++;
            }
            break;
            
        default:
            LOG_WARNING("Unknown command opcode: " + std::to_string(static_cast<uint8_t>(command.opcode)));
            return false;
    }
    
    return true;
}

void CommandReceiver::update_connection_status() {
    auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    
    if (connected_ && (now - last_heartbeat_time_) > config_.heartbeat_timeout_ms) {
        connected_ = false;
        LOG_WARNING("Connection lost - no heartbeat received");
    }
}

void CommandReceiver::cleanup() {
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
}

} // namespace network
