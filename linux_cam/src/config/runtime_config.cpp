#include "config/runtime_config.h"
#include "utils/logger.h"

#include <fstream>
#include <sstream>
#include <sys/mman.h>
#include <sched.h>
#include <pthread.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

// Simple JSON parsing (minimal implementation)
#include <regex>

namespace config {

RuntimeConfig& RuntimeConfig::instance() {
    static RuntimeConfig instance;
    return instance;
}

bool RuntimeConfig::load_from_file(const std::string& config_file) {
    std::ifstream file(config_file);
    if (!file.is_open()) {
        last_validation_error_ = "Cannot open config file: " + config_file;
        return false;
    }
    
    std::stringstream buffer;
    buffer << file.rdbuf();
    
    return load_from_json(buffer.str());
}

bool RuntimeConfig::load_from_json(const std::string& json_content) {
    // Simple JSON parsing - in production, use a proper JSON library
    config_ = create_default_config();
    
    // Parse basic string values
    std::regex string_pattern("\"(\\w+)\"\\s*:\\s*\"([^\"]*)\"");
    std::regex number_pattern("\"(\\w+)\"\\s*:\\s*(\\d+(?:\\.\\d+)?)");
    std::regex bool_pattern("\"(\\w+)\"\\s*:\\s*(true|false)");
    
    std::smatch match;
    std::string content = json_content;
    
    // Parse string values
    auto string_iter = std::sregex_iterator(content.begin(), content.end(), string_pattern);
    auto string_end = std::sregex_iterator();
    
    for (std::sregex_iterator i = string_iter; i != string_end; ++i) {
        std::smatch match = *i;
        std::string key = match[1].str();
        std::string value = match[2].str();
        
        // Map to config values
        if (key == "device_path") config_.camera.device_path = value;
        else if (key == "target_ip") config_.streamer.target_ip = value;
        else if (key == "log_level") config_.log_level = value;
        else if (key == "log_file") config_.log_file = value;
        else if (key == "i2c_device") config_.servo_controller.i2c_device = value;
        else if (key == "bind_address") config_.command_receiver.bind_address = value;
        else if (key == "preset") config_.encoder.preset = value;
        else if (key == "profile") config_.encoder.profile = value;
        else if (key == "pixel_format") {
            // Convert pixel format string to V4L2 constant
            if (value == "V4L2_PIX_FMT_YUYV") config_.camera.pixel_format = V4L2_PIX_FMT_YUYV;
            else if (value == "V4L2_PIX_FMT_NV12") config_.camera.pixel_format = V4L2_PIX_FMT_NV12;
            else if (value == "V4L2_PIX_FMT_MJPEG") config_.camera.pixel_format = V4L2_PIX_FMT_MJPEG;
            else if (value == "V4L2_PIX_FMT_RGB24") config_.camera.pixel_format = V4L2_PIX_FMT_RGB24;
        }
    }
    
    // Parse numeric values
    auto number_iter = std::sregex_iterator(content.begin(), content.end(), number_pattern);
    auto number_end = std::sregex_iterator();
    
    for (std::sregex_iterator i = number_iter; i != number_end; ++i) {
        std::smatch match = *i;
        std::string key = match[1].str();
        double value = std::stod(match[2].str());
        
        // Map to config values
        if (key == "width") {
            config_.camera.width = static_cast<uint32_t>(value);
            config_.encoder.width = static_cast<uint32_t>(value);  // Match encoder to camera
        }
        else if (key == "height") {
            config_.camera.height = static_cast<uint32_t>(value);
            config_.encoder.height = static_cast<uint32_t>(value);  // Match encoder to camera
        }
        else if (key == "fps") {
            config_.camera.fps = static_cast<uint32_t>(value);
            config_.encoder.fps = static_cast<uint32_t>(value);  // Match encoder to camera
        }
        else if (key == "bitrate") config_.encoder.bitrate = static_cast<uint32_t>(value);
        else if (key == "target_port") config_.streamer.target_port = static_cast<uint16_t>(value);
        else if (key == "listen_port") config_.command_receiver.listen_port = static_cast<uint16_t>(value);
        else if (key == "controller_address") config_.servo_controller.controller_address = static_cast<uint8_t>(value);
        else if (key == "rt_priority") config_.rt_priority = static_cast<int>(value);
    }
    
    // Parse boolean values
    auto bool_iter = std::sregex_iterator(content.begin(), content.end(), bool_pattern);
    auto bool_end = std::sregex_iterator();
    
    for (std::sregex_iterator i = bool_iter; i != bool_end; ++i) {
        std::smatch match = *i;
        std::string key = match[1].str();
        bool value = (match[2].str() == "true");
        
        // Map to config values
        if (key == "enable_logging") config_.enable_logging = value;
        else if (key == "enable_rt_scheduling") config_.enable_rt_scheduling = value;
        else if (key == "enable_cpu_affinity") config_.enable_cpu_affinity = value;
        else if (key == "enable_memory_locking") config_.enable_memory_locking = value;
        else if (key == "use_hardware") config_.encoder.use_hardware = value;
        else if (key == "enable_smooth_movement") config_.servo_controller.enable_smooth_movement = value;
        else if (key == "enable_heartbeat") config_.command_receiver.enable_heartbeat = value;
    }
    
    return validate_config();
}

bool RuntimeConfig::save_to_file(const std::string& config_file) const {
    std::ofstream file(config_file);
    if (!file.is_open()) {
        return false;
    }
    
    file << serialize_to_json();
    return file.good();
}

bool RuntimeConfig::validate_config() const {
    // Validate camera config
    if (config_.camera.width == 0 || config_.camera.height == 0) {
        last_validation_error_ = "Invalid camera resolution";
        return false;
    }
    
    if (config_.camera.fps == 0 || config_.camera.fps > 120) {
        last_validation_error_ = "Invalid camera FPS";
        return false;
    }
    
    // Validate encoder config
    if (config_.encoder.bitrate == 0) {
        last_validation_error_ = "Invalid encoder bitrate";
        return false;
    }
    
    // Validate network config
    if (config_.streamer.target_port == 0 || config_.command_receiver.listen_port == 0) {
        last_validation_error_ = "Invalid network ports";
        return false;
    }
    
    // Validate RT config
    if (config_.rt_priority < 1 || config_.rt_priority > 99) {
        last_validation_error_ = "Invalid RT priority (must be 1-99)";
        return false;
    }
    
    return true;
}

std::string RuntimeConfig::get_validation_errors() const {
    return last_validation_error_;
}

ApplicationConfig RuntimeConfig::create_default_config() {
    ApplicationConfig config;
    
    // Camera defaults
    config.camera.device_path = "/dev/video0";
    config.camera.width = 1920;
    config.camera.height = 1080;
    config.camera.fps = 30;
    config.camera.buffer_count = 4;
    
    // Encoder defaults
    config.encoder.width = 1920;
    config.encoder.height = 1080;
    config.encoder.fps = 30;
    config.encoder.bitrate = 2000000; // 2 Mbps
    config.encoder.gop_size = 30;
    config.encoder.use_hardware = true;
    config.encoder.preset = "ultrafast";
    config.encoder.profile = "baseline";
    
    // Streamer defaults
    config.streamer.target_ip = "192.168.1.100";
    config.streamer.target_port = 5000;
    config.streamer.mtu = 1400;
    config.streamer.enable_fragmentation = true;
    
    // Command receiver defaults
    config.command_receiver.listen_port = 5001;
    config.command_receiver.bind_address = "0.0.0.0";
    config.command_receiver.timeout_ms = 100;
    config.command_receiver.enable_heartbeat = true;
    config.command_receiver.heartbeat_timeout_ms = 5000;
    
    // Servo controller defaults
    config.servo_controller.i2c_device = "/dev/i2c-1";
    config.servo_controller.controller_address = 0x40;
    config.servo_controller.pwm_frequency = 50;
    config.servo_controller.enable_smooth_movement = true;
    
    // Application defaults
    config.enable_logging = true;
    config.log_level = "INFO";
    config.log_file = "/var/log/linux_cam.log";
    config.enable_performance_monitoring = true;
    config.stats_update_interval_ms = 1000;
    
    // RT defaults
    config.enable_rt_scheduling = true;
    config.rt_priority = 50;
    config.enable_cpu_affinity = true;
    config.cpu_affinity_mask = 0x02; // CPU 1
    config.enable_memory_locking = true;
    
    return config;
}

ApplicationConfig RuntimeConfig::create_high_performance_config() {
    auto config = create_default_config();
    
    // High performance settings
    config.encoder.preset = "fast";
    config.encoder.bitrate = 5000000; // 5 Mbps
    config.camera.fps = 60;
    config.encoder.fps = 60;
    config.rt_priority = 80;
    config.enable_cpu_affinity = true;
    config.cpu_affinity_mask = 0x0E; // CPUs 1,2,3
    
    return config;
}

ApplicationConfig RuntimeConfig::create_low_latency_config() {
    auto config = create_default_config();
    
    // Low latency settings
    config.encoder.preset = "ultrafast";
    config.encoder.gop_size = 15; // Smaller GOP for lower latency
    config.encoder.bitrate = 1000000; // 1 Mbps for lower latency
    config.camera.buffer_count = 2; // Minimal buffering
    config.streamer.mtu = 1200; // Smaller MTU for lower latency
    config.rt_priority = 90;
    
    return config;
}

std::string RuntimeConfig::serialize_to_json() const {
    std::ostringstream json;
    
    json << "{\n";
    json << "  \"camera\": {\n";
    json << "    \"device_path\": \"" << config_.camera.device_path << "\",\n";
    json << "    \"width\": " << config_.camera.width << ",\n";
    json << "    \"height\": " << config_.camera.height << ",\n";
    json << "    \"fps\": " << config_.camera.fps << ",\n";
    json << "    \"buffer_count\": " << config_.camera.buffer_count << "\n";
    json << "  },\n";
    
    json << "  \"encoder\": {\n";
    json << "    \"bitrate\": " << config_.encoder.bitrate << ",\n";
    json << "    \"preset\": \"" << config_.encoder.preset << "\",\n";
    json << "    \"profile\": \"" << config_.encoder.profile << "\",\n";
    json << "    \"use_hardware\": " << (config_.encoder.use_hardware ? "true" : "false") << "\n";
    json << "  },\n";
    
    json << "  \"streamer\": {\n";
    json << "    \"target_ip\": \"" << config_.streamer.target_ip << "\",\n";
    json << "    \"target_port\": " << config_.streamer.target_port << "\n";
    json << "  },\n";
    
    json << "  \"command_receiver\": {\n";
    json << "    \"listen_port\": " << config_.command_receiver.listen_port << ",\n";
    json << "    \"enable_heartbeat\": " << (config_.command_receiver.enable_heartbeat ? "true" : "false") << "\n";
    json << "  },\n";
    
    json << "  \"servo_controller\": {\n";
    json << "    \"i2c_device\": \"" << config_.servo_controller.i2c_device << "\",\n";
    json << "    \"controller_address\": " << static_cast<int>(config_.servo_controller.controller_address) << ",\n";
    json << "    \"enable_smooth_movement\": " << (config_.servo_controller.enable_smooth_movement ? "true" : "false") << "\n";
    json << "  },\n";
    
    json << "  \"application\": {\n";
    json << "    \"log_level\": \"" << config_.log_level << "\",\n";
    json << "    \"enable_rt_scheduling\": " << (config_.enable_rt_scheduling ? "true" : "false") << ",\n";
    json << "    \"rt_priority\": " << config_.rt_priority << "\n";
    json << "  }\n";
    json << "}\n";
    
    return json.str();
}

namespace rt_utils {

bool set_realtime_scheduling(int priority) {
    struct sched_param param;
    param.sched_priority = priority;
    
    if (sched_setscheduler(0, SCHED_FIFO, &param) < 0) {
        LOG_ERROR("Failed to set realtime scheduling: " + std::string(strerror(errno)));
        return false;
    }
    
    LOG_INFO("Realtime scheduling enabled with priority " + std::to_string(priority));
    return true;
}

bool set_cpu_affinity(int cpu_mask) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    
    for (int cpu = 0; cpu < 32; cpu++) {
        if (cpu_mask & (1 << cpu)) {
            CPU_SET(cpu, &cpuset);
        }
    }
    
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) < 0) {
        LOG_ERROR("Failed to set CPU affinity: " + std::string(strerror(errno)));
        return false;
    }
    
    LOG_INFO("CPU affinity set to mask 0x" + std::to_string(cpu_mask));
    return true;
}

bool lock_memory() {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0) {
        LOG_ERROR("Failed to lock memory: " + std::string(strerror(errno)));
        return false;
    }
    
    LOG_INFO("Memory locking enabled");
    return true;
}

bool configure_rt_optimizations(const ApplicationConfig& config) {
    bool success = true;
    
    if (config.enable_rt_scheduling) {
        success &= set_realtime_scheduling(config.rt_priority);
    }
    
    if (config.enable_cpu_affinity) {
        success &= set_cpu_affinity(config.cpu_affinity_mask);
    }
    
    if (config.enable_memory_locking) {
        success &= lock_memory();
    }
    
    return success;
}

} // namespace rt_utils

} // namespace config
