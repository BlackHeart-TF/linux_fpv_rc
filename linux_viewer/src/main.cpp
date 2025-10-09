#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <memory>
#include <thread>
#include <chrono>
#include <fstream>
#include <SDL2/SDL.h>

#include "config/runtime_config.h"
#include "decoder/h264_decoder.h"
#include "gamepad/gamepad_manager.h"
#include "network/udp_receiver.h"
#include "network/command_sender.h"
#include "utils/logger.h"
#include "utils/performance_monitor.h"

class LinuxViewerApplication {
public:
    LinuxViewerApplication() : running_(false), window_(nullptr), renderer_(nullptr), texture_(nullptr) {}
    
    bool initialize(const std::string& config_file) {
        // Load configuration
        auto& config_manager = config::RuntimeConfig::instance();
        
        std::string config_to_load = config_file;
        if (config_to_load.empty()) {
            // Try ./config/viewer_config.json first, then /etc/linux_viewer/viewer_config.json
            if (std::ifstream("./config/viewer_config.json").good()) {
                config_to_load = "./config/viewer_config.json";
            } else if (std::ifstream("/etc/linux_viewer/viewer_config.json").good()) {
                config_to_load = "/etc/linux_viewer/viewer_config.json";
            }
        }
        
        if (!config_to_load.empty() && !config_manager.load_from_file(config_to_load)) {
            std::cerr << "Failed to load config file: " << config_to_load << std::endl;
            std::cerr << "Using default configuration." << std::endl;
        } else {
            config_manager.set_config(config::RuntimeConfig::get_default_config());
        }
        
        const auto& config = config_manager.get_config();
        
        // Initialize logger
        utils::Logger::instance().initialize(config.logging.log_file, 
            config.logging.log_level == "DEBUG" ? utils::LogLevel::DEBUG :
            config.logging.log_level == "INFO" ? utils::LogLevel::INFO :
            config.logging.log_level == "WARNING" ? utils::LogLevel::WARNING :
            config.logging.log_level == "ERROR" ? utils::LogLevel::ERROR :
            utils::LogLevel::FATAL,
            config.logging.enable_console_output);
        
        LOG_INFO("Starting Linux Viewer Application");
        
        // Initialize performance monitoring
        if (config.performance.enable_monitoring) {
            utils::PerformanceMonitor::instance().initialize(config.performance.stats_update_interval_ms);
            utils::PerformanceMonitor::instance().start_monitoring();
        }
        
        // Initialize SDL
        if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK | SDL_INIT_GAMECONTROLLER) < 0) {
            LOG_FATAL("Failed to initialize SDL: " + std::string(SDL_GetError()));
            return false;
        }
        
        // Initialize components
        try {
            // Create SDL window and renderer
            if (!setup_display(config)) {
                LOG_FATAL("Failed to setup display");
                return false;
            }
            
            // UDP Receiver
            receiver_ = std::make_unique<network::UDPReceiver>(config.network_receiver);
            if (!receiver_->initialize()) {
                LOG_FATAL("Failed to initialize UDP receiver");
                return false;
            }
            
            // Command Sender
            sender_ = std::make_unique<network::CommandSender>(config.network_sender);
            if (!sender_->initialize()) {
                LOG_FATAL("Failed to initialize command sender");
                return false;
            }
            
            // H.264 Decoder
            decoder_ = std::make_unique<decoder::H264Decoder>(config.decoder);
            if (!decoder_->initialize()) {
                LOG_FATAL("Failed to initialize H.264 decoder");
                return false;
            }
            
            // Gamepad Manager
            gamepad_ = std::make_unique<gamepad::GamepadManager>(config.gamepad);
            if (!gamepad_->initialize()) {
                LOG_WARNING("Failed to initialize gamepad manager - continuing without gamepad support");
                gamepad_.reset();
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
        receiver_->start_receiving();
        sender_->start_sending();
        decoder_->initialize(); // Start decoder thread
        
        if (gamepad_) {
            gamepad_->start_input_processing();
        }
        
        LOG_INFO("All components started, entering main loop");
        
        // Main application loop
        auto last_stats_time = std::chrono::steady_clock::now();
        
        while (running_) {
            // Handle SDL events
            SDL_Event event;
            while (SDL_PollEvent(&event)) {
                if (event.type == SDL_QUIT) {
                    LOG_INFO("SDL_QUIT event received");
                    running_ = false;
                } else if (event.type == SDL_KEYDOWN) {
                    if (event.key.keysym.sym == SDLK_ESCAPE) {
                        LOG_INFO("Escape key pressed");
                        running_ = false;
                    }
                } else if (event.type == SDL_WINDOWEVENT) {
                    if (event.window.event == SDL_WINDOWEVENT_CLOSE) {
                        LOG_INFO("Window close event received");
                        running_ = false;
                    }
                }
            }
            
            // Render frame
            render_frame();
            
            // Print performance stats periodically
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_stats_time).count() >= 10) {
                LOG_INFO(utils::PerformanceMonitor::instance().get_performance_report());
                last_stats_time = now;
            }
            
            // Small sleep to prevent busy waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60 FPS
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
    
    // SDL components
    SDL_Window* window_;
    SDL_Renderer* renderer_;
    SDL_Texture* texture_;
    
    // Application components
    std::unique_ptr<network::UDPReceiver> receiver_;
    std::unique_ptr<network::CommandSender> sender_;
    std::unique_ptr<decoder::H264Decoder> decoder_;
    std::unique_ptr<gamepad::GamepadManager> gamepad_;
    
    // Current frame data
    std::mutex frame_mutex_;
    decoder::DecodedFrame current_frame_;
    bool frame_ready_;
    
    bool setup_display(const config::ViewerConfig& config) {
        // Create window
        Uint32 window_flags = SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE;
        if (config.display.fullscreen) {
            window_flags |= SDL_WINDOW_FULLSCREEN;
        }
        
        window_ = SDL_CreateWindow("Linux FPV Viewer",
                                  SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                  config.display.window_width, config.display.window_height,
                                  window_flags);
        
        if (!window_) {
            LOG_ERROR("Failed to create SDL window: " + std::string(SDL_GetError()));
            return false;
        }
        
        // Create renderer
        Uint32 renderer_flags = SDL_RENDERER_ACCELERATED;
        if (config.display.vsync) {
            renderer_flags |= SDL_RENDERER_PRESENTVSYNC;
        }
        
        renderer_ = SDL_CreateRenderer(window_, -1, renderer_flags);
        if (!renderer_) {
            LOG_ERROR("Failed to create SDL renderer: " + std::string(SDL_GetError()));
            return false;
        }
        
        // Create texture for video frames
        texture_ = SDL_CreateTexture(renderer_, SDL_PIXELFORMAT_YV12,
                                   SDL_TEXTUREACCESS_STREAMING,
                                   config.display.window_width, config.display.window_height);
        
        if (!texture_) {
            LOG_ERROR("Failed to create SDL texture: " + std::string(SDL_GetError()));
            return false;
        }
        
        frame_ready_ = false;
        
        LOG_INFO("Display setup complete - Window: " + std::to_string(config.display.window_width) + 
                "x" + std::to_string(config.display.window_height));
        
        return true;
    }
    
    void setup_callbacks() {
        // Setup receiver callback to send frames to decoder
        receiver_->set_frame_received_callback([this](const decoder::EncodedPacket& packet) {
            decoder_->decode_packet(packet);
            utils::PerformanceMonitor::instance().record_frame_received();
        });
        
        // Setup decoder callback to display frames
        decoder_->set_decoded_frame_callback([this](const decoder::DecodedFrame& frame) {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            current_frame_ = frame;
            frame_ready_ = true;
            utils::PerformanceMonitor::instance().record_frame_decoded(0); // TODO: measure actual decode time
        });
        
        if (gamepad_) {
            // Setup gamepad callbacks
            gamepad_->set_servo_command_callback(
                [this](uint8_t servo_id, uint16_t position, uint8_t speed) {
                    sender_->send_servo_command(servo_id, position, speed);
                    utils::PerformanceMonitor::instance().record_gamepad_command();
                });
            
            gamepad_->set_button_action_callback(
                [this](const std::string& action) {
                    handle_button_action(action);
                    utils::PerformanceMonitor::instance().record_button_press();
                });
        }
    }
    
    void handle_sdl_event(const SDL_Event& event) {
        switch (event.type) {
            case SDL_QUIT:
                stop();
                break;
                
            case SDL_KEYDOWN:
                handle_keyboard_input(event.key);
                break;
                
            case SDL_WINDOWEVENT:
                if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
                    handle_window_resize(event.window.data1, event.window.data2);
                }
                break;
        }
    }
    
    void handle_keyboard_input(const SDL_KeyboardEvent& key) {
        switch (key.keysym.sym) {
            case SDLK_ESCAPE:
            case SDLK_q:
                stop();
                break;
                
            case SDLK_f:
                toggle_fullscreen();
                break;
                
            case SDLK_c:
                if (gamepad_) {
                    gamepad_->center_all_servos();
                }
                break;
                
            case SDLK_i:
                sender_->send_force_iframe();
                break;
                
            case SDLK_s:
                LOG_INFO(utils::PerformanceMonitor::instance().get_performance_report());
                break;
        }
    }
    
    void handle_button_action(const std::string& action) {
        if (action == "center_servos") {
            if (gamepad_) {
                gamepad_->center_all_servos();
            }
        } else if (action == "force_iframe") {
            sender_->send_force_iframe();
        } else {
            LOG_DEBUG("Unknown button action: " + action);
        }
    }
    
    void handle_window_resize(int width, int height) {
        if (texture_) {
            SDL_DestroyTexture(texture_);
            texture_ = SDL_CreateTexture(renderer_, SDL_PIXELFORMAT_YV12,
                                       SDL_TEXTUREACCESS_STREAMING, width, height);
        }
        
        LOG_INFO("Window resized to " + std::to_string(width) + "x" + std::to_string(height));
    }
    
    void toggle_fullscreen() {
        Uint32 flags = SDL_GetWindowFlags(window_);
        if (flags & SDL_WINDOW_FULLSCREEN) {
            SDL_SetWindowFullscreen(window_, 0);
            LOG_INFO("Exited fullscreen mode");
        } else {
            SDL_SetWindowFullscreen(window_, SDL_WINDOW_FULLSCREEN_DESKTOP);
            LOG_INFO("Entered fullscreen mode");
        }
    }
    
    void render_frame() {
        // Clear screen
        SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 255);
        SDL_RenderClear(renderer_);
        
        // Render video frame if available
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (frame_ready_ && current_frame_.data[0]) {
                render_video_frame();
                frame_ready_ = false;
                utils::PerformanceMonitor::instance().record_frame_displayed(0); // TODO: measure actual display time
            }
        }
        
        // Render UI overlay
        render_ui_overlay();
        
        // Present frame
        SDL_RenderPresent(renderer_);
    }
    
    void render_video_frame() {
        if (!texture_ || !current_frame_.data[0]) {
            return;
        }
        
        // Update texture with YUV data
        void* pixels;
        int pitch;
        
        if (SDL_LockTexture(texture_, nullptr, &pixels, &pitch) == 0) {
            // Copy Y plane
            uint8_t* y_plane = static_cast<uint8_t*>(pixels);
            for (int y = 0; y < current_frame_.height; y++) {
                memcpy(y_plane + y * pitch, 
                       current_frame_.data[0] + y * current_frame_.linesize[0],
                       current_frame_.width);
            }
            
            // Copy U and V planes (for YUV420P)
            if (current_frame_.pixel_format == AV_PIX_FMT_YUV420P) {
                uint8_t* u_plane = y_plane + pitch * current_frame_.height;
                uint8_t* v_plane = u_plane + (pitch / 2) * (current_frame_.height / 2);
                
                for (int y = 0; y < current_frame_.height / 2; y++) {
                    memcpy(u_plane + y * (pitch / 2),
                           current_frame_.data[1] + y * current_frame_.linesize[1],
                           current_frame_.width / 2);
                    memcpy(v_plane + y * (pitch / 2),
                           current_frame_.data[2] + y * current_frame_.linesize[2],
                           current_frame_.width / 2);
                }
            }
            
            SDL_UnlockTexture(texture_);
        }
        
        // Render texture to screen
        SDL_RenderCopy(renderer_, texture_, nullptr, nullptr);
    }
    
    void render_ui_overlay() {
        const auto& config = config::RuntimeConfig::instance().get_config();
        if (!config.display.show_stats) {
            return;
        }
        
        // Simple text rendering would require SDL_ttf
        // For now, just render some basic status indicators
        
        // Connection status indicator
        auto receiver_stats = receiver_->get_stats();
        auto sender_stats = sender_->get_stats();
        
        // Green dot if receiving data, red if not
        SDL_Rect status_rect = {10, 10, 20, 20};
        if (receiver_stats.packets_received > 0 && receiver_stats.current_bitrate > 0) {
            SDL_SetRenderDrawColor(renderer_, 0, 255, 0, 255); // Green
        } else {
            SDL_SetRenderDrawColor(renderer_, 255, 0, 0, 255); // Red
        }
        SDL_RenderFillRect(renderer_, &status_rect);
        
        // Command status indicator
        SDL_Rect cmd_status_rect = {40, 10, 20, 20};
        if (sender_stats.connection_active) {
            SDL_SetRenderDrawColor(renderer_, 0, 255, 0, 255); // Green
        } else {
            SDL_SetRenderDrawColor(renderer_, 255, 0, 0, 255); // Red
        }
        SDL_RenderFillRect(renderer_, &cmd_status_rect);
    }
    
    void shutdown() {
        // Stop all components in reverse order
        if (gamepad_) gamepad_->stop_input_processing();
        if (decoder_) decoder_->shutdown();
        if (sender_) sender_->stop_sending();
        if (receiver_) receiver_->stop_receiving();
        
        // Cleanup SDL
        if (texture_) {
            SDL_DestroyTexture(texture_);
            texture_ = nullptr;
        }
        
        if (renderer_) {
            SDL_DestroyRenderer(renderer_);
            renderer_ = nullptr;
        }
        
        if (window_) {
            SDL_DestroyWindow(window_);
            window_ = nullptr;
        }
        
        SDL_Quit();
        
        if (utils::PerformanceMonitor::instance().is_monitoring()) {
            utils::PerformanceMonitor::instance().stop_monitoring();
        }
        
        LOG_INFO("Application shutdown complete");
        utils::Logger::instance().close();
    }
};

// Global application instance for signal handling
std::unique_ptr<LinuxViewerApplication> g_app;

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
            std::cout << "\nControls:\n";
            std::cout << "  ESC/Q:    Quit application\n";
            std::cout << "  F:        Toggle fullscreen\n";
            std::cout << "  C:        Center all servos\n";
            std::cout << "  I:        Force I-frame\n";
            std::cout << "  S:        Show performance stats\n";
            return 0;
        }
    }
    
    // Setup signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    try {
        g_app = std::make_unique<LinuxViewerApplication>();
        
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
