#include "utils/performance_monitor.h"
#include "utils/logger.h"

#include <fstream>
#include <sstream>
#include <thread>
#include <iomanip>

namespace utils {

// PerformanceTimer implementation
PerformanceTimer::PerformanceTimer() {
}

void PerformanceTimer::start() {
    start_time_ = std::chrono::high_resolution_clock::now();
    running_ = true;
}

uint64_t PerformanceTimer::stop_and_get_us() {
    if (!running_) {
        return 0;
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time_);
    running_ = false;
    
    return duration.count();
}

uint64_t PerformanceTimer::get_elapsed_us() const {
    if (!running_) {
        return 0;
    }
    
    auto current_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time_);
    
    return duration.count();
}

// PerformanceMonitor implementation
PerformanceMonitor& PerformanceMonitor::instance() {
    static PerformanceMonitor instance;
    return instance;
}

void PerformanceMonitor::initialize(uint32_t update_interval_ms) {
    update_interval_ms_ = update_interval_ms;
    last_update_time_ = std::chrono::steady_clock::now();
}

void PerformanceMonitor::start_monitoring() {
    if (monitoring_) {
        return;
    }
    
    monitoring_ = true;
    monitor_thread_ = std::thread(&PerformanceMonitor::monitoring_loop, this);
    
    LOG_INFO("Performance monitoring started");
}

void PerformanceMonitor::stop_monitoring() {
    if (!monitoring_) {
        return;
    }
    
    monitoring_ = false;
    
    if (monitor_thread_.joinable()) {
        monitor_thread_.join();
    }
    
    LOG_INFO("Performance monitoring stopped");
}

void PerformanceMonitor::record_frame_captured() {
    metrics_.frames_captured++;
}

void PerformanceMonitor::record_frame_encoded(uint64_t encode_time_us) {
    metrics_.frames_encoded++;
    
    // Update running average encode time
    static uint64_t total_encode_time = 0;
    static uint64_t encode_count = 0;
    
    total_encode_time += encode_time_us;
    encode_count++;
    metrics_.avg_encode_time_us = total_encode_time / encode_count;
}

void PerformanceMonitor::record_frame_transmitted(uint64_t transmit_time_us, size_t bytes) {
    metrics_.frames_transmitted++;
    metrics_.bytes_transmitted += bytes;
    
    // Update running average transmit time
    static uint64_t total_transmit_time = 0;
    static uint64_t transmit_count = 0;
    
    total_transmit_time += transmit_time_us;
    transmit_count++;
    metrics_.avg_transmit_time_us = total_transmit_time / transmit_count;
}

void PerformanceMonitor::record_frame_dropped() {
    metrics_.frames_dropped++;
}

void PerformanceMonitor::record_capture_time(uint64_t time_us) {
    // Update running average capture time
    static uint64_t total_capture_time = 0;
    static uint64_t capture_count = 0;
    
    total_capture_time += time_us;
    capture_count++;
    metrics_.avg_capture_time_us = total_capture_time / capture_count;
}

void PerformanceMonitor::record_command_received() {
    metrics_.commands_received++;
}

void PerformanceMonitor::record_servo_command_processed() {
    metrics_.servo_commands_processed++;
}

void PerformanceMonitor::record_invalid_command() {
    metrics_.invalid_commands++;
}

std::string PerformanceMonitor::get_performance_report() const {
    std::ostringstream report;
    
    report << "=== Performance Report ===\n";
    report << "Frames: Captured=" << metrics_.frames_captured 
           << ", Encoded=" << metrics_.frames_encoded
           << ", Transmitted=" << metrics_.frames_transmitted
           << ", Dropped=" << metrics_.frames_dropped << "\n";
    
    report << "Current FPS: " << std::fixed << std::setprecision(1) << get_current_fps() << "\n";
    report << "Current Bitrate: " << std::fixed << std::setprecision(2) 
           << get_current_bitrate_mbps() << " Mbps\n";
    report << "Pipeline Efficiency: " << std::fixed << std::setprecision(1) 
           << (get_pipeline_efficiency() * 100.0) << "%\n";
    
    report << "Timing (avg μs): Capture=" << metrics_.avg_capture_time_us
           << ", Encode=" << metrics_.avg_encode_time_us
           << ", Transmit=" << metrics_.avg_transmit_time_us << "\n";
    
    report << "Network: Packets=" << metrics_.packets_sent
           << ", Lost=" << metrics_.packets_lost
           << ", Bytes=" << metrics_.bytes_transmitted << "\n";
    
    report << "Commands: Received=" << metrics_.commands_received
           << ", Servo=" << metrics_.servo_commands_processed
           << ", Invalid=" << metrics_.invalid_commands << "\n";
    
    report << "System: CPU=" << std::fixed << std::setprecision(1) << metrics_.cpu_usage << "%"
           << ", Memory=" << (metrics_.memory_usage_bytes / 1024 / 1024) << "MB"
           << ", Temp=" << metrics_.temperature_celsius << "°C\n";
    
    return report.str();
}

double PerformanceMonitor::get_current_fps() const {
    std::lock_guard<std::mutex> lock(rate_mutex_);
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_update_time_).count();
    
    if (elapsed == 0) return 0.0;
    
    uint64_t frames_diff = metrics_.frames_captured - last_frames_captured_;
    return static_cast<double>(frames_diff) / elapsed;
}

double PerformanceMonitor::get_current_bitrate_mbps() const {
    return metrics_.current_bitrate / 1000000.0; // Convert to Mbps
}

double PerformanceMonitor::get_pipeline_efficiency() const {
    if (metrics_.frames_captured == 0) return 0.0;
    return static_cast<double>(metrics_.frames_transmitted) / metrics_.frames_captured;
}

void PerformanceMonitor::update_system_metrics() {
    metrics_.cpu_usage = read_cpu_usage();
    metrics_.memory_usage_bytes = read_memory_usage();
    metrics_.temperature_celsius = read_cpu_temperature();
}

void PerformanceMonitor::monitoring_loop() {
    while (monitoring_) {
        update_system_metrics();
        update_rates();
        
        std::this_thread::sleep_for(std::chrono::milliseconds(update_interval_ms_));
    }
}

double PerformanceMonitor::read_cpu_usage() {
    static uint64_t last_total = 0;
    static uint64_t last_idle = 0;
    
    std::ifstream file("/proc/stat");
    if (!file.is_open()) {
        return 0.0;
    }
    
    std::string line;
    std::getline(file, line);
    
    // Parse CPU line: cpu user nice system idle iowait irq softirq steal guest guest_nice
    std::istringstream iss(line);
    std::string cpu_label;
    uint64_t user, nice, system, idle, iowait, irq, softirq, steal, guest, guest_nice;
    
    iss >> cpu_label >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal >> guest >> guest_nice;
    
    uint64_t total = user + nice + system + idle + iowait + irq + softirq + steal;
    uint64_t total_diff = total - last_total;
    uint64_t idle_diff = idle - last_idle;
    
    double cpu_usage = 0.0;
    if (total_diff > 0) {
        cpu_usage = 100.0 * (1.0 - static_cast<double>(idle_diff) / total_diff);
    }
    
    last_total = total;
    last_idle = idle;
    
    return cpu_usage;
}

uint64_t PerformanceMonitor::read_memory_usage() {
    std::ifstream file("/proc/self/status");
    if (!file.is_open()) {
        return 0;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        if (line.find("VmRSS:") == 0) {
            std::istringstream iss(line);
            std::string label;
            uint64_t value;
            std::string unit;
            
            iss >> label >> value >> unit;
            
            // Convert kB to bytes
            return value * 1024;
        }
    }
    
    return 0;
}

uint32_t PerformanceMonitor::read_cpu_temperature() {
    // Try common temperature sources
    const char* temp_files[] = {
        "/sys/class/thermal/thermal_zone0/temp",
        "/sys/class/hwmon/hwmon0/temp1_input",
        "/sys/class/hwmon/hwmon1/temp1_input",
        nullptr
    };
    
    for (const char** temp_file = temp_files; *temp_file; temp_file++) {
        std::ifstream file(*temp_file);
        if (file.is_open()) {
            uint32_t temp_millicelsius;
            file >> temp_millicelsius;
            
            // Convert from millicelsius to celsius
            return temp_millicelsius / 1000;
        }
    }
    
    return 0; // Temperature not available
}

void PerformanceMonitor::update_rates() {
    std::lock_guard<std::mutex> lock(rate_mutex_);
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_time_).count();
    
    if (elapsed >= 1000) { // Update every second
        uint64_t frames_diff = metrics_.frames_captured - last_frames_captured_;
        uint64_t bytes_diff = metrics_.bytes_transmitted - last_bytes_transmitted_;
        
        // Calculate current bitrate (bits per second)
        if (elapsed > 0) {
            metrics_.current_bitrate = (bytes_diff * 8000.0) / elapsed; // bits per second
        }
        
        last_update_time_ = now;
        last_frames_captured_ = metrics_.frames_captured;
        last_bytes_transmitted_ = metrics_.bytes_transmitted;
    }
}

// ScopedTimer implementation
ScopedTimer::ScopedTimer(Callback callback) : callback_(callback) {
    timer_.start();
}

ScopedTimer::~ScopedTimer() {
    uint64_t elapsed = timer_.stop_and_get_us();
    if (callback_) {
        callback_(elapsed);
    }
}

} // namespace utils
