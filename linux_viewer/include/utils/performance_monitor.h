#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>

namespace utils {

struct PerformanceMetrics {
    // Video metrics
    uint64_t frames_received = 0;
    uint64_t frames_decoded = 0;
    uint64_t frames_displayed = 0;
    uint64_t frames_dropped = 0;
    
    // Network metrics
    uint64_t packets_received = 0;
    uint64_t bytes_received = 0;
    double current_bitrate = 0.0;
    double packet_loss_rate = 0.0;
    
    // Gamepad metrics
    uint64_t gamepad_commands_sent = 0;
    uint64_t button_presses = 0;
    
    // Timing metrics
    double average_decode_time_ms = 0.0;
    double average_display_time_ms = 0.0;
    double current_fps = 0.0;
    double target_fps = 30.0;
    
    // System metrics
    double cpu_usage_percent = 0.0;
    double memory_usage_mb = 0.0;
    int64_t uptime_seconds = 0;
};

class PerformanceMonitor {
public:
    static PerformanceMonitor& instance();
    
    void initialize(uint32_t update_interval_ms = 1000);
    void start_monitoring();
    void stop_monitoring();
    bool is_monitoring() const;
    
    // Metric recording
    void record_frame_received();
    void record_frame_decoded(double decode_time_ms);
    void record_frame_displayed(double display_time_ms);
    void record_frame_dropped();
    void record_packet_received(size_t bytes);
    void record_gamepad_command();
    void record_button_press();
    
    // Getters
    PerformanceMetrics get_metrics() const;
    std::string get_performance_report() const;
    
    // Setters
    void set_bitrate(double bitrate);
    void set_packet_loss_rate(double loss_rate);
    void set_target_fps(double fps);

private:
    PerformanceMonitor() = default;
    ~PerformanceMonitor();
    
    PerformanceMonitor(const PerformanceMonitor&) = delete;
    PerformanceMonitor& operator=(const PerformanceMonitor&) = delete;
    
    mutable std::mutex metrics_mutex_;
    PerformanceMetrics metrics_;
    
    std::atomic<bool> monitoring_;
    uint32_t update_interval_ms_;
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point last_fps_update_;
    
    // FPS calculation
    uint64_t last_frames_decoded_;
    uint64_t last_frames_displayed_;
    
    void update_fps_metrics();
    void update_system_metrics();
    double get_cpu_usage();
    double get_memory_usage();
};

} // namespace utils
