#pragma once

#include <string>
#include <atomic>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <sys/socket.h>
#include <netinet/in.h>

namespace network {

// Command protocol (matches linux_cam)
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

struct ServoPosition {
    uint8_t servo_id;       // 0 to MAX_SERVO_COUNT-1
    uint16_t position;      // 0-65535, maps to servo range
} __attribute__((packed));

class CommandSender {
public:
    struct Config {
        std::string target_ip = "127.0.0.1";
        uint16_t target_port = 5001;
        uint32_t send_buffer_size = 32768;
        uint32_t heartbeat_interval_ms = 1000;
        bool enable_heartbeat = true;
        uint32_t command_timeout_ms = 100;
    };

    explicit CommandSender(const Config& config);
    ~CommandSender();

    // Non-copyable, movable
    CommandSender(const CommandSender&) = delete;
    CommandSender& operator=(const CommandSender&) = delete;
    CommandSender(CommandSender&&) = default;
    CommandSender& operator=(CommandSender&&) = default;

    bool initialize();
    void shutdown();
    bool is_initialized() const;

    void start_sending();
    void stop_sending();
    bool is_sending() const;

    // Command interface
    bool send_servo_command(uint8_t servo_id, uint16_t position, uint8_t speed = 255);
    bool send_multi_servo_command(const std::vector<ServoPosition>& servos);
    bool send_system_shutdown();
    bool send_force_iframe();
    bool send_set_bitrate(uint32_t bitrate);
    bool send_heartbeat();

    // Batch commands (more efficient)
    void queue_servo_command(uint8_t servo_id, uint16_t position, uint8_t speed = 255);
    void flush_queued_commands();

    // Statistics
    struct Stats {
        uint64_t commands_sent;
        uint64_t commands_failed;
        uint64_t heartbeats_sent;
        uint64_t bytes_sent;
        double send_rate;
        bool connection_active;
    };
    
    Stats get_stats() const;

private:
    Config config_;
    std::atomic<bool> initialized_;
    std::atomic<bool> sending_;
    std::atomic<bool> should_stop_;
    
    // Socket
    int socket_fd_;
    sockaddr_in target_addr_;
    
    // Threading
    std::thread sender_thread_;
    
    // Command queue
    std::queue<SimpleCommand> command_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    static constexpr size_t MAX_QUEUE_SIZE = 100;
    
    // Heartbeat
    std::chrono::steady_clock::time_point last_heartbeat_;
    
    // Statistics
    mutable std::mutex stats_mutex_;
    Stats stats_;
    std::chrono::steady_clock::time_point last_stats_update_;
    
    bool setup_socket();
    void sender_loop();
    bool send_command(const SimpleCommand& command);
    bool send_raw_command(CommandOpcode opcode, const uint8_t* params = nullptr, uint8_t length = 0);
    void send_periodic_heartbeat();
    void update_stats(bool command_sent, size_t bytes_sent);
    void cleanup();
};

} // namespace network
