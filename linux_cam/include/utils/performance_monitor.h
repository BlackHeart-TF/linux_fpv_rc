#pragma once

#include <chrono>
#include <atomic>
#include <mutex>
#include <unordered_map>
#include <string>
#include <thread>
#include <functional>

namespace utils {

struct PerformanceMetrics {
    // Frame processing metrics
    std::atomic<uint64_t> frames_captured{0};
    std::atomic<uint64_t> frames_encoded{0};
    std::atomic<uint64_t> frames_transmitted{0};
    std::atomic<uint64_t> frames_dropped{0};
    
    // Timing metrics (microseconds)
    std::atomic<uint64_t> avg_capture_time_us{0};
    std::atomic<uint64_t> avg_encode_time_us{0};
    std::atomic<uint64_t> avg_transmit_time_us{0};
    std::atomic<uint64_t> total_pipeline_time_us{0};
    
    // Network metrics
    std::atomic<uint64_t> bytes_transmitted{0};
    std::atomic<uint64_t> packets_sent{0};
    std::atomic<uint64_t> packets_lost{0};
    std::atomic<double> current_bitrate{0.0};
    
    // System metrics
    std::atomic<double> cpu_usage{0.0};
    std::atomic<uint64_t> memory_usage_bytes{0};
    std::atomic<uint32_t> temperature_celsius{0};
    
    // Command processing
    std::atomic<uint64_t> commands_received{0};
    std::atomic<uint64_t> servo_commands_processed{0};
    std::atomic<uint64_t> invalid_commands{0};
};

class PerformanceTimer {
public:
    PerformanceTimer();
    void start();
    uint64_t stop_and_get_us();
    uint64_t get_elapsed_us() const;

private:
    std::chrono::high_resolution_clock::time_point start_time_;
    std::atomic<bool> running_{false};
};

class PerformanceMonitor {
public:
    static PerformanceMonitor& instance();
    
    void initialize(uint32_t update_interval_ms = 1000);
    void start_monitoring();
    void stop_monitoring();
    
    // Metric updates
    void record_frame_captured();
    void record_frame_encoded(uint64_t encode_time_us);
    void record_frame_transmitted(uint64_t transmit_time_us, size_t bytes);
    void record_frame_dropped();
    void record_capture_time(uint64_t time_us);
    void record_command_received();
    void record_servo_command_processed();
    void record_invalid_command();
    
    // Get metrics
    const PerformanceMetrics& get_metrics() const { return metrics_; }
    std::string get_performance_report() const;
    
    // Real-time statistics
    double get_current_fps() const;
    double get_current_bitrate_mbps() const;
    double get_pipeline_efficiency() const; // frames_transmitted / frames_captured
    
    // System monitoring
    void update_system_metrics();

private:
    PerformanceMonitor() = default;
    
    PerformanceMetrics metrics_;
    
    std::atomic<bool> monitoring_{false};
    std::thread monitor_thread_;
    uint32_t update_interval_ms_{1000};
    
    // For rate calculations
    mutable std::mutex rate_mutex_;
    std::chrono::steady_clock::time_point last_update_time_;
    uint64_t last_frames_captured_{0};
    uint64_t last_bytes_transmitted_{0};
    
    void monitoring_loop();
    double read_cpu_usage();
    uint64_t read_memory_usage();
    uint32_t read_cpu_temperature();
    void update_rates();
};

// RAII timer for automatic performance measurement
class ScopedTimer {
public:
    using Callback = std::function<void(uint64_t)>;
    
    explicit ScopedTimer(Callback callback);
    ~ScopedTimer();

private:
    PerformanceTimer timer_;
    Callback callback_;
};

// Convenience macros for performance measurement
#define MEASURE_SCOPE(callback) utils::ScopedTimer _timer(callback)
#define MEASURE_CAPTURE_TIME() MEASURE_SCOPE([](uint64_t us) { \
    utils::PerformanceMonitor::instance().record_capture_time(us); })
#define MEASURE_ENCODE_TIME() MEASURE_SCOPE([](uint64_t us) { \
    utils::PerformanceMonitor::instance().record_frame_encoded(us); })

} // namespace utils
