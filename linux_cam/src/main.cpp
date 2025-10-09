#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <memory>
#include <thread>
#include <chrono>
#include <fstream>

#include "config/runtime_config.h"
#include "camera/csi_camera.h"
#include "encoder/h264_encoder.h"
#include "network/udp_streamer.h"
#include "network/command_receiver.h"
#include "servo/i2c_servo_controller.h"
#include "utils/logger.h"
#include "utils/performance_monitor.h"

class LinuxCamApplication {
public:
    LinuxCamApplication() : running_(false) {}
    
    bool initialize(const std::string& config_file) {
        // Load configuration
        auto& config_manager = config::RuntimeConfig::instance();
        
        std::string config_to_load = config_file;
        if (config_to_load.empty()) {
            // Try ./config.json first, then /etc/linux_cam/config.json
            if (std::ifstream("./config.json").good()) {
                config_to_load = "./config.json";
            } else if (std::ifstream("/etc/linux_cam/config.json").good()) {
                config_to_load = "/etc/linux_cam/config.json";
            }
        }
        
        if (!config_to_load.empty()) {
            if (!config_manager.load_from_file(config_to_load)) {
                std::cerr << "Failed to load config file: " << config_to_load << std::endl;
                return false;
            }
        } else {
            std::cerr << "No config file specified and no default config found!" << std::endl;
            return false;
        }
        
        const auto& config = config_manager.get_config();
        
        // Initialize logger
        utils::Logger::instance().initialize(config.log_file, 
            config.log_level == "DEBUG" ? utils::LogLevel::DEBUG :
            config.log_level == "INFO" ? utils::LogLevel::INFO :
            config.log_level == "WARNING" ? utils::LogLevel::WARNING :
            config.log_level == "ERROR" ? utils::LogLevel::ERROR :
            utils::LogLevel::FATAL);
        
        LOG_INFO("Starting Linux Camera Application");
        
        // Apply RT optimizations if enabled
        if (config.enable_rt_scheduling || config.enable_cpu_affinity || config.enable_memory_locking) {
            if (!config::rt_utils::configure_rt_optimizations(config)) {
                LOG_WARNING("Failed to apply some RT optimizations, continuing anyway");
            }
        }
        
        // Initialize performance monitoring
        if (config.enable_performance_monitoring) {
            utils::PerformanceMonitor::instance().initialize(config.stats_update_interval_ms);
            utils::PerformanceMonitor::instance().start_monitoring();
        }
        
        // Initialize components
        try {
            // Camera
            camera_ = std::make_unique<camera::CSICamera>(config.camera);
            if (!camera_->initialize()) {
                LOG_FATAL("Failed to initialize camera");
                return false;
            }
            
            // Encoder
            encoder_ = std::make_unique<encoder::H264Encoder>(config.encoder);
            if (!encoder_->initialize()) {
                LOG_FATAL("Failed to initialize H.264 encoder");
                return false;
            }
            
            // UDP Streamer
            streamer_ = std::make_unique<network::UDPStreamer>(config.streamer);
            if (!streamer_->initialize()) {
                LOG_FATAL("Failed to initialize UDP streamer");
                return false;
            }
            
            // Command Receiver
            command_receiver_ = std::make_unique<network::CommandReceiver>(config.command_receiver);
            if (!command_receiver_->initialize()) {
                LOG_FATAL("Failed to initialize command receiver");
                return false;
            }
            
            // Servo Controller (optional - skip if I2C device doesn't exist)
            servo_controller_ = std::make_unique<servo::I2CServoController>(config.servo_controller);
            if (!servo_controller_->initialize()) {
                LOG_WARNING("Failed to initialize servo controller (I2C device may not exist) - continuing without servo support");
                servo_controller_.reset(); // Don't use it if it failed to initialize
            }
            
            LOG_INFO("All components initialized successfully");
            
        } catch (const std::exception& e) {
            LOG_FATAL(std::string("Exception during initialization: ") + e.what());
            return false;
        }
        
        // Setup callbacks
        setup_callbacks();
        
        return true;
    }
    
    void run() {
        LOG_INFO("Starting application main loop");
        running_ = true;
        
        // Start all components
        command_receiver_->start_listening();
        streamer_->start_streaming();
        
        // Start camera capture (this will drive the pipeline)
        if (!camera_->start_capture([this](const camera::FrameData& frame) {
            this->on_frame_captured(frame);
        })) {
            LOG_FATAL("Failed to start camera capture");
            return;
        }
        
        LOG_INFO("All components started, entering main loop");
        
        // Main application loop
        while (running_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            // Check connection status (only log occasionally, not every 100ms)
            static auto last_warning_time = std::chrono::steady_clock::now();
            auto now_steady = std::chrono::steady_clock::now();
            
            if (!command_receiver_->is_connected()) {
                auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                auto last_heartbeat = command_receiver_->get_last_heartbeat_time();
                
                if (now - last_heartbeat > 10000) { // 10 seconds
                    // Only log warning once per 5 seconds to avoid spam
                    if (std::chrono::duration_cast<std::chrono::seconds>(now_steady - last_warning_time).count() >= 5) {
                        LOG_WARNING("No command packets received for 10 seconds");
                        last_warning_time = now_steady;
                    }
                    
                    // Optionally disable servos for safety
                    if (servo_controller_) {
                        servo_controller_->disable_all_servos();
                    }
                }
            }
            
            // Print performance stats periodically
            static auto last_stats_time = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_stats_time).count() >= 10) {
                LOG_INFO(utils::PerformanceMonitor::instance().get_performance_report());
                last_stats_time = now;
            }
        }
        
        LOG_INFO("Shutting down application");
        shutdown();
    }
    
    void stop() {
        LOG_INFO("Stop signal received");
        running_ = false;
    }
    
private:
    std::atomic<bool> running_;
    
    std::unique_ptr<camera::CSICamera> camera_;
    std::unique_ptr<encoder::H264Encoder> encoder_;
    std::unique_ptr<network::UDPStreamer> streamer_;
    std::unique_ptr<network::CommandReceiver> command_receiver_;
    std::unique_ptr<servo::I2CServoController> servo_controller_;
    
    void setup_callbacks() {
        // Setup encoder callback to send frames to streamer
        encoder_->set_encoded_frame_callback([this](const encoder::EncodedFrame& frame) {
            streamer_->queue_frame(frame);
            utils::PerformanceMonitor::instance().record_frame_transmitted(0, frame.size);
        });
        
        // Setup command receiver callbacks
        command_receiver_->set_servo_command_callback(
            [this](uint8_t servo_id, uint16_t position, uint8_t speed) {
                if (servo_controller_) {
                    servo_controller_->set_servo_position(servo_id, position, speed);
                    utils::PerformanceMonitor::instance().record_servo_command_processed();
                } else {
                    LOG_DEBUG("Servo command received but no servo controller available");
                }
            });
        
        command_receiver_->set_multi_servo_command_callback(
            [this](const std::vector<network::ServoPosition>& servos) {
                if (servo_controller_) {
                    servo_controller_->set_multiple_servos(servos);
                    utils::PerformanceMonitor::instance().record_servo_command_processed();
                } else {
                    LOG_DEBUG("Multi-servo command received but no servo controller available");
                }
            });
        
        command_receiver_->set_system_command_callback(
            [this](network::CommandOpcode opcode, const uint8_t* params, uint8_t length) {
                switch (opcode) {
                    case network::CommandOpcode::SYSTEM_SHUTDOWN:
                        LOG_INFO("Shutdown command received");
                        this->stop();
                        break;
                    case network::CommandOpcode::FORCE_IFRAME:
                        LOG_INFO("Force I-frame command received");
                        encoder_->force_next_iframe();
                        break;
                    case network::CommandOpcode::SET_BITRATE:
                        if (length >= 4) {
                            uint32_t new_bitrate = *reinterpret_cast<const uint32_t*>(params);
                            LOG_INFO("Set bitrate command received: " + std::to_string(new_bitrate));
                            // TODO: Implement dynamic bitrate change
                        }
                        break;
                    case network::CommandOpcode::HEARTBEAT:
                        // Heartbeat is handled automatically by command receiver
                        break;
                    default:
                        LOG_WARNING("Unknown system command received: " + std::to_string(static_cast<uint8_t>(opcode)));
                        break;
                }
            });
    }
    
    void on_frame_captured(const camera::FrameData& frame) {
        utils::PerformanceMonitor::instance().record_frame_captured();
        
        LOG_DEBUG("Frame captured: size=" + std::to_string(frame.size) + " timestamp=" + std::to_string(frame.timestamp_us));
        
        // Encode frame
        if (!encoder_->encode_frame(frame)) {
            LOG_ERROR("Failed to encode frame");
            utils::PerformanceMonitor::instance().record_frame_dropped();
        } else {
            LOG_DEBUG("Frame encoded successfully");
        }
    }
    
    void shutdown() {
        // Stop all components in reverse order
        if (camera_) camera_->stop_capture();
        if (streamer_) streamer_->stop_streaming();
        if (command_receiver_) command_receiver_->stop_listening();
        if (servo_controller_) servo_controller_->shutdown();
        
        if (utils::PerformanceMonitor::instance().get_metrics().frames_captured > 0) {
            utils::PerformanceMonitor::instance().stop_monitoring();
        }
        
        LOG_INFO("Application shutdown complete");
        utils::Logger::instance().close();
    }
};

// Global application instance for signal handling
std::unique_ptr<LinuxCamApplication> g_app;

void signal_handler(int signal) {
    if (g_app) {
        g_app->stop();
    }
}

int main(int argc, char* argv[]) {
    std::string config_file;
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            config_file = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [--config config_file.json] [--help]\n";
            std::cout << "  --config: Specify configuration file (default: use built-in config)\n";
            std::cout << "  --help:   Show this help message\n";
            return 0;
        }
    }
    
    // Setup signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    try {
        g_app = std::make_unique<LinuxCamApplication>();
        
        if (!g_app->initialize(config_file)) {
            std::cerr << "Failed to initialize application" << std::endl;
            return 1;
        }
        
        g_app->run();
        
    } catch (const std::exception& e) {
        std::cerr << "Unhandled exception: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
