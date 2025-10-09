#include "config/runtime_config.h"
#include "utils/logger.h"

#include <fstream>
#include <sstream>
#include <iostream>

// Simple JSON parsing - in a real implementation, you'd use a proper JSON library
#include <regex>

namespace config {

RuntimeConfig& RuntimeConfig::instance() {
    static RuntimeConfig instance;
    return instance;
}

bool RuntimeConfig::load_from_file(const std::string& config_file) {
    LOG_INFO("Loading configuration from: " + config_file);
    
    std::ifstream file(config_file);
    if (!file.is_open()) {
        LOG_ERROR("Failed to open config file: " + config_file);
        return false;
    }
    
    std::string json_content((std::istreambuf_iterator<char>(file)),
                             std::istreambuf_iterator<char>());
    
    if (!parse_json_config(json_content)) {
        LOG_ERROR("Failed to parse config file: " + config_file);
        return false;
    }
    
    LOG_INFO("Configuration loaded successfully");
    return true;
}

bool RuntimeConfig::save_to_file(const std::string& config_file) const {
    LOG_INFO("Saving configuration to: " + config_file);
    
    std::ofstream file(config_file);
    if (!file.is_open()) {
        LOG_ERROR("Failed to create config file: " + config_file);
        return false;
    }
    
    file << generate_json_config();
    
    LOG_INFO("Configuration saved successfully");
    return true;
}

const ViewerConfig& RuntimeConfig::get_config() const {
    return config_;
}

ViewerConfig& RuntimeConfig::get_config() {
    return config_;
}

void RuntimeConfig::set_config(const ViewerConfig& config) {
    config_ = config;
}

ViewerConfig RuntimeConfig::get_default_config() {
    ViewerConfig config;
    
    // Network receiver defaults
    config.network_receiver.listen_port = 5000;
    config.network_receiver.bind_address = "0.0.0.0";
    config.network_receiver.receive_buffer_size = 1024 * 1024;
    config.network_receiver.timeout_ms = 100;
    config.network_receiver.max_frame_size = 1024 * 1024;
    config.network_receiver.fragment_timeout_ms = 1000;
    
    // Network sender defaults
    config.network_sender.target_ip = "127.0.0.1";
    config.network_sender.target_port = 5001;
    config.network_sender.send_buffer_size = 32768;
    config.network_sender.heartbeat_interval_ms = 1000;
    config.network_sender.enable_heartbeat = true;
    config.network_sender.command_timeout_ms = 100;
    
    // Decoder defaults
    config.decoder.enable_hardware_acceleration = true;
    config.decoder.max_frame_queue_size = 10;
    config.decoder.decoder_threads = 2;
    config.decoder.output_format = AV_PIX_FMT_YUV420P;
    config.decoder.enable_multithreading = true;
    
    // Display defaults
    config.display.window_width = 640;
    config.display.window_height = 480;
    config.display.fullscreen = false;
    config.display.vsync = true;
    config.display.show_stats = true;
    
    // Gamepad defaults
    config.gamepad.device_index = 0;
    config.gamepad.global_deadzone = 0.1f;
    config.gamepad.update_rate_hz = 50;
    config.gamepad.enable_rumble = false;
    
    // Add default servo mappings
    gamepad::ServoMapping servo0;
    servo0.servo_id = 0;
    servo0.axis = 0; // Left stick X
    servo0.invert = false;
    servo0.min_value = 1000;
    servo0.max_value = 2000;
    servo0.deadzone = 0.0f; // Use global deadzone
    config.gamepad.servo_mappings.push_back(servo0);
    
    gamepad::ServoMapping servo1;
    servo1.servo_id = 1;
    servo1.axis = 1; // Left stick Y
    servo1.invert = true;
    servo1.min_value = 1000;
    servo1.max_value = 2000;
    servo1.deadzone = 0.0f;
    config.gamepad.servo_mappings.push_back(servo1);
    
    // Add default button mappings
    gamepad::ButtonMapping button0;
    button0.button = 0; // A button
    button0.action = "center_servos";
    config.gamepad.button_mappings.push_back(button0);
    
    gamepad::ButtonMapping button1;
    button1.button = 1; // B button
    button1.action = "force_iframe";
    config.gamepad.button_mappings.push_back(button1);
    
    // Logging defaults
    config.logging.log_level = "INFO";
    config.logging.log_file = "/tmp/linux_viewer.log";
    config.logging.enable_console_output = true;
    
    // Performance defaults
    config.performance.enable_monitoring = true;
    config.performance.stats_update_interval_ms = 1000;
    config.performance.max_latency_ms = 100;
    
    return config;
}

std::string RuntimeConfig::to_json_string() const {
    return generate_json_config();
}

bool RuntimeConfig::from_json_string(const std::string& json) {
    return parse_json_config(json);
}

bool RuntimeConfig::parse_json_config(const std::string& json_content) {
    // This is a simplified JSON parser for demonstration
    // In a real implementation, use a proper JSON library like nlohmann/json
    
    config_ = get_default_config();
    
    // Parse network configuration
    std::regex port_regex(R"("video_receive_port"\s*:\s*(\d+))");
    std::smatch match;
    if (std::regex_search(json_content, match, port_regex)) {
        config_.network_receiver.listen_port = std::stoi(match[1]);
    }
    
    std::regex ip_regex("\"command_send_ip\"\\s*:\\s*\"([^\"]+)\"");
    if (std::regex_search(json_content, match, ip_regex)) {
        config_.network_sender.target_ip = match[1];
    }
    
    std::regex cmd_port_regex(R"("command_send_port"\s*:\s*(\d+))");
    if (std::regex_search(json_content, match, cmd_port_regex)) {
        config_.network_sender.target_port = std::stoi(match[1]);
    }
    
    // Parse display configuration
    std::regex width_regex(R"("window_width"\s*:\s*(\d+))");
    if (std::regex_search(json_content, match, width_regex)) {
        config_.display.window_width = std::stoi(match[1]);
    }
    
    std::regex height_regex(R"("window_height"\s*:\s*(\d+))");
    if (std::regex_search(json_content, match, height_regex)) {
        config_.display.window_height = std::stoi(match[1]);
    }
    
    std::regex fullscreen_regex(R"("fullscreen"\s*:\s*(true|false))");
    if (std::regex_search(json_content, match, fullscreen_regex)) {
        config_.display.fullscreen = (match[1] == "true");
    }
    
    // Parse gamepad configuration
    std::regex deadzone_regex(R"("deadzone"\s*:\s*([0-9.]+))");
    if (std::regex_search(json_content, match, deadzone_regex)) {
        config_.gamepad.global_deadzone = std::stof(match[1]);
    }
    
    // Parse logging configuration
    std::regex log_level_regex("\"log_level\"\\s*:\\s*\"([^\"]+)\"");
    if (std::regex_search(json_content, match, log_level_regex)) {
        config_.logging.log_level = match[1];
    }
    
    std::regex log_file_regex("\"log_file\"\\s*:\\s*\"([^\"]+)\"");
    if (std::regex_search(json_content, match, log_file_regex)) {
        config_.logging.log_file = match[1];
    }
    
    return true;
}

std::string RuntimeConfig::generate_json_config() const {
    std::ostringstream json;
    
    json << "{\n";
    json << "  \"network\": {\n";
    json << "    \"video_receive_port\": " << config_.network_receiver.listen_port << ",\n";
    json << "    \"command_send_ip\": \"" << config_.network_sender.target_ip << "\",\n";
    json << "    \"command_send_port\": " << config_.network_sender.target_port << ",\n";
    json << "    \"receive_buffer_size\": " << config_.network_receiver.receive_buffer_size << ",\n";
    json << "    \"heartbeat_interval_ms\": " << config_.network_sender.heartbeat_interval_ms << "\n";
    json << "  },\n";
    
    json << "  \"decoder\": {\n";
    json << "    \"enable_hardware_acceleration\": " << (config_.decoder.enable_hardware_acceleration ? "true" : "false") << ",\n";
    json << "    \"max_frame_queue_size\": " << config_.decoder.max_frame_queue_size << ",\n";
    json << "    \"decoder_threads\": " << config_.decoder.decoder_threads << "\n";
    json << "  },\n";
    
    json << "  \"display\": {\n";
    json << "    \"window_width\": " << config_.display.window_width << ",\n";
    json << "    \"window_height\": " << config_.display.window_height << ",\n";
    json << "    \"fullscreen\": " << (config_.display.fullscreen ? "true" : "false") << ",\n";
    json << "    \"vsync\": " << (config_.display.vsync ? "true" : "false") << ",\n";
    json << "    \"show_stats\": " << (config_.display.show_stats ? "true" : "false") << "\n";
    json << "  },\n";
    
    json << "  \"gamepad\": {\n";
    json << "    \"device_index\": " << config_.gamepad.device_index << ",\n";
    json << "    \"deadzone\": " << config_.gamepad.global_deadzone << ",\n";
    json << "    \"servo_mappings\": [\n";
    
    for (size_t i = 0; i < config_.gamepad.servo_mappings.size(); i++) {
        const auto& mapping = config_.gamepad.servo_mappings[i];
        json << "      {\n";
        json << "        \"servo_id\": " << static_cast<int>(mapping.servo_id) << ",\n";
        json << "        \"axis\": " << mapping.axis << ",\n";
        json << "        \"invert\": " << (mapping.invert ? "true" : "false") << ",\n";
        json << "        \"min_value\": " << mapping.min_value << ",\n";
        json << "        \"max_value\": " << mapping.max_value << "\n";
        json << "      }";
        if (i < config_.gamepad.servo_mappings.size() - 1) json << ",";
        json << "\n";
    }
    
    json << "    ],\n";
    json << "    \"button_mappings\": [\n";
    
    for (size_t i = 0; i < config_.gamepad.button_mappings.size(); i++) {
        const auto& mapping = config_.gamepad.button_mappings[i];
        json << "      {\n";
        json << "        \"button\": " << mapping.button << ",\n";
        json << "        \"action\": \"" << mapping.action << "\"\n";
        json << "      }";
        if (i < config_.gamepad.button_mappings.size() - 1) json << ",";
        json << "\n";
    }
    
    json << "    ]\n";
    json << "  },\n";
    
    json << "  \"logging\": {\n";
    json << "    \"log_level\": \"" << config_.logging.log_level << "\",\n";
    json << "    \"log_file\": \"" << config_.logging.log_file << "\",\n";
    json << "    \"enable_console_output\": " << (config_.logging.enable_console_output ? "true" : "false") << "\n";
    json << "  },\n";
    
    json << "  \"performance\": {\n";
    json << "    \"enable_monitoring\": " << (config_.performance.enable_monitoring ? "true" : "false") << ",\n";
    json << "    \"stats_update_interval_ms\": " << config_.performance.stats_update_interval_ms << ",\n";
    json << "    \"max_latency_ms\": " << config_.performance.max_latency_ms << "\n";
    json << "  }\n";
    json << "}\n";
    
    return json.str();
}

} // namespace config
