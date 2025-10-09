#pragma once

#include <string>
#include <vector>
#include "decoder/h264_decoder.h"
#include "gamepad/gamepad_manager.h"
#include "network/udp_receiver.h"
#include "network/command_sender.h"

namespace config {

struct ViewerConfig {
    // Network configuration
    network::UDPReceiver::Config network_receiver;
    network::CommandSender::Config network_sender;
    
    // Decoder configuration
    decoder::H264Decoder::Config decoder;
    
    // Display configuration
    struct {
        int window_width = 640;
        int window_height = 480;
        bool fullscreen = false;
        bool vsync = true;
        bool show_stats = true;
    } display;
    
    // Gamepad configuration
    gamepad::GamepadManager::Config gamepad;
    
    // Logging configuration
    struct {
        std::string log_level = "INFO";
        std::string log_file = "";
        bool enable_console_output = true;
    } logging;
    
    // Performance monitoring
    struct {
        bool enable_monitoring = true;
        uint32_t stats_update_interval_ms = 1000;
        uint32_t max_latency_ms = 100;
    } performance;
};

class RuntimeConfig {
public:
    static RuntimeConfig& instance();
    
    bool load_from_file(const std::string& config_file);
    bool save_to_file(const std::string& config_file) const;
    
    const ViewerConfig& get_config() const;
    ViewerConfig& get_config();
    
    void set_config(const ViewerConfig& config);
    
    // Utility functions
    static ViewerConfig get_default_config();
    std::string to_json_string() const;
    bool from_json_string(const std::string& json);

private:
    RuntimeConfig() = default;
    ~RuntimeConfig() = default;
    
    RuntimeConfig(const RuntimeConfig&) = delete;
    RuntimeConfig& operator=(const RuntimeConfig&) = delete;
    
    ViewerConfig config_;
    
    bool parse_json_config(const std::string& json_content);
    std::string generate_json_config() const;
};

} // namespace config
