#pragma once

#include <string>
#include <atomic>
#include <thread>
#include <functional>
#include <array>
#include <mutex>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>

namespace network {

// Simple command protocol: opcode + variable length parameters
enum class CommandOpcode : uint8_t {
    SERVO_MOVE = 0x01,      // servo_id(1) + position(2) + speed(1)
    SERVO_MULTI = 0x02,     // count(1) + [servo_id(1) + position(2)]...
    SYSTEM_SHUTDOWN = 0x10,  // no params
    FORCE_IFRAME = 0x11,    // no params  
    SET_BITRATE = 0x12,     // bitrate(4)
    HEARTBEAT = 0xFF        // no params
};

struct SimpleCommand {
    CommandOpcode opcode;
    uint8_t length;         // Length of parameters that follow
    uint8_t params[32];     // Variable length parameters
} __attribute__((packed));

// Servo position for multi-servo commands
struct ServoPosition {
    uint8_t servo_id;       // 0 to MAX_SERVO_COUNT-1
    uint16_t position;      // 0-65535, maps to servo range
} __attribute__((packed));

using ServoCommandCallback = std::function<void(uint8_t servo_id, uint16_t position, uint8_t speed)>;
using MultiServoCommandCallback = std::function<void(const std::vector<ServoPosition>& servos)>;
using SystemCommandCallback = std::function<void(CommandOpcode opcode, const uint8_t* params, uint8_t length)>;

class CommandReceiver {
public:
    struct Config {
        uint16_t listen_port = 5001;
        std::string bind_address = "0.0.0.0";
        uint32_t timeout_ms = 100;
        bool enable_heartbeat = true;
        uint32_t heartbeat_timeout_ms = 5000;
    };

    explicit CommandReceiver(const Config& config);
    ~CommandReceiver();

    // Non-copyable, movable
    CommandReceiver(const CommandReceiver&) = delete;
    CommandReceiver& operator=(const CommandReceiver&) = delete;
    CommandReceiver(CommandReceiver&&) = default;
    CommandReceiver& operator=(CommandReceiver&&) = default;

    bool initialize();
    void start_listening();
    void stop_listening();
    bool is_listening() const;

    void set_servo_command_callback(ServoCommandCallback callback);
    void set_multi_servo_command_callback(MultiServoCommandCallback callback);
    void set_system_command_callback(SystemCommandCallback callback);

    // Connection status
    bool is_connected() const;
    uint64_t get_last_heartbeat_time() const;

    struct Stats {
        uint64_t packets_received;
        uint64_t invalid_packets;
        uint64_t checksum_errors;
        uint64_t servo_commands_processed;
        uint64_t system_commands_processed;
    };
    
    Stats get_stats() const;

private:
    Config config_;
    int socket_fd_;
    sockaddr_in bind_addr_;
    sockaddr_in sender_addr_;
    
    std::atomic<bool> listening_;
    std::atomic<bool> should_stop_;
    std::atomic<bool> connected_;
    std::atomic<uint64_t> last_heartbeat_time_;
    std::thread receiver_thread_;
    
    ServoCommandCallback servo_callback_;
    MultiServoCommandCallback multi_servo_callback_;
    SystemCommandCallback system_callback_;
    
    // Statistics
    mutable std::mutex stats_mutex_;
    Stats stats_;
    
    bool setup_socket();
    void receiver_loop();
    bool process_packet(const uint8_t* data, size_t size, const sockaddr_in& sender);
    bool process_simple_command(const SimpleCommand& command);
    bool validate_command(const SimpleCommand& command, size_t total_size);
    void update_connection_status();
    void cleanup();
};

} // namespace network
