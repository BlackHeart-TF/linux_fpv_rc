#pragma once

#include <string>
#include <memory>
#include "camera/csi_camera.h"
#include "encoder/h264_encoder.h"
#include "network/udp_streamer.h"
#include "network/command_receiver.h"
#include "servo/i2c_servo_controller.h"

namespace config {

struct ApplicationConfig {
    camera::CSICamera::Config camera;
    encoder::H264Encoder::Config encoder;
    network::UDPStreamer::Config streamer;
    network::CommandReceiver::Config command_receiver;
    servo::I2CServoController::Config servo_controller;
    
    // Application-level settings
    bool enable_logging = true;
    std::string log_level = "INFO";
    std::string log_file = "/var/log/linux_cam.log";
    bool enable_performance_monitoring = true;
    uint32_t stats_update_interval_ms = 1000;
    
    // RT kernel optimizations
    bool enable_rt_scheduling = true;
    int rt_priority = 50;
    bool enable_cpu_affinity = true;
    int cpu_affinity_mask = 0x02; // CPU 1
    bool enable_memory_locking = true;
};

class RuntimeConfig {
public:
    static RuntimeConfig& instance();
    
    bool load_from_file(const std::string& config_file);
    bool load_from_json(const std::string& json_content);
    bool save_to_file(const std::string& config_file) const;
    
    const ApplicationConfig& get_config() const { return config_; }
    ApplicationConfig& get_mutable_config() { return config_; }
    
    // Individual component configs
    const camera::CSICamera::Config& camera_config() const { return config_.camera; }
    const encoder::H264Encoder::Config& encoder_config() const { return config_.encoder; }
    const network::UDPStreamer::Config& streamer_config() const { return config_.streamer; }
    const network::CommandReceiver::Config& command_receiver_config() const { return config_.command_receiver; }
    const servo::I2CServoController::Config& servo_controller_config() const { return config_.servo_controller; }
    
    // Validation
    bool validate_config() const;
    std::string get_validation_errors() const;
    
    // Default configurations
    static ApplicationConfig create_default_config();
    static ApplicationConfig create_high_performance_config();
    static ApplicationConfig create_low_latency_config();

private:
    RuntimeConfig() = default;
    ApplicationConfig config_;
    mutable std::string last_validation_error_;
    
    bool parse_json_config(const std::string& json_content);
    std::string serialize_to_json() const;
};

// Helper functions for RT optimizations
namespace rt_utils {
    bool set_realtime_scheduling(int priority);
    bool set_cpu_affinity(int cpu_mask);
    bool lock_memory();
    bool configure_rt_optimizations(const ApplicationConfig& config);
}

} // namespace config
