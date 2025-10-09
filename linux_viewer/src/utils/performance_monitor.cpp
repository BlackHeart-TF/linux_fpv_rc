#include "utils/performance_monitor.h"
#include <sstream>
#include <iomanip>
#include <fstream>
#include <thread>

namespace utils {

PerformanceMonitor& PerformanceMonitor::instance() {
    static PerformanceMonitor instance;
    return instance;
}

PerformanceMonitor::~PerformanceMonitor() {
    stop_monitoring();
}

void PerformanceMonitor::initialize(uint32_t update_interval_ms) {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    
    update_interval_ms_ = update_interval_ms;
    start_time_ = std::chrono::steady_clock::now();
    last_fps_update_ = start_time_;
    last_frames_decoded_ = 0;
    last_frames_displayed_ = 0;
    
    // Reset metrics
    metrics_ = PerformanceMetrics{};
}

void PerformanceMonitor::start_monitoring() {
    monitoring_ = true;
}

void PerformanceMonitor::stop_monitoring() {
    monitoring_ = false;
}

bool PerformanceMonitor::is_monitoring() const {
    return monitoring_;
}

void PerformanceMonitor::record_frame_received() {
    if (!monitoring_) return;
    
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    metrics_.frames_received++;
}

void PerformanceMonitor::record_frame_decoded(double decode_time_ms) {
    if (!monitoring_) return;
    
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    metrics_.frames_decoded++;
    
    // Update average decode time (exponential moving average)
    const double alpha = 0.1;
    metrics_.average_decode_time_ms = alpha * decode_time_ms + (1.0 - alpha) * metrics_.average_decode_time_ms;
    
    update_fps_metrics();
}

void PerformanceMonitor::record_frame_displayed(double display_time_ms) {
    if (!monitoring_) return;
    
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    metrics_.frames_displayed++;
    
    // Update average display time (exponential moving average)
    const double alpha = 0.1;
    metrics_.average_display_time_ms = alpha * display_time_ms + (1.0 - alpha) * metrics_.average_display_time_ms;
}

void PerformanceMonitor::record_frame_dropped() {
    if (!monitoring_) return;
    
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    metrics_.frames_dropped++;
}

void PerformanceMonitor::record_packet_received(size_t bytes) {
    if (!monitoring_) return;
    
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    metrics_.packets_received++;
    metrics_.bytes_received += bytes;
}

void PerformanceMonitor::record_gamepad_command() {
    if (!monitoring_) return;
    
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    metrics_.gamepad_commands_sent++;
}

void PerformanceMonitor::record_button_press() {
    if (!monitoring_) return;
    
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    metrics_.button_presses++;
}

PerformanceMetrics PerformanceMonitor::get_metrics() const {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    
    PerformanceMetrics metrics = metrics_;
    
    // Update uptime
    auto now = std::chrono::steady_clock::now();
    metrics.uptime_seconds = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_).count();
    
    // Update system metrics
    const_cast<PerformanceMonitor*>(this)->update_system_metrics();
    metrics.cpu_usage_percent = metrics_.cpu_usage_percent;
    metrics.memory_usage_mb = metrics_.memory_usage_mb;
    
    return metrics;
}

std::string PerformanceMonitor::get_performance_report() const {
    PerformanceMetrics metrics = get_metrics();
    
    std::ostringstream report;
    report << std::fixed << std::setprecision(2);
    
    report << "=== Performance Report ===\n";
    report << "Uptime: " << metrics.uptime_seconds << "s\n";
    report << "Video: " << metrics.frames_received << " received, " 
           << metrics.frames_decoded << " decoded, " 
           << metrics.frames_displayed << " displayed, "
           << metrics.frames_dropped << " dropped\n";
    report << "FPS: " << metrics.current_fps << "/" << metrics.target_fps << "\n";
    report << "Decode time: " << metrics.average_decode_time_ms << "ms avg\n";
    report << "Display time: " << metrics.average_display_time_ms << "ms avg\n";
    report << "Network: " << metrics.packets_received << " packets, " 
           << (metrics.bytes_received / 1024.0 / 1024.0) << " MB\n";
    report << "Bitrate: " << (metrics.current_bitrate / 1000000.0) << " Mbps\n";
    report << "Packet loss: " << metrics.packet_loss_rate << "%\n";
    report << "Gamepad: " << metrics.gamepad_commands_sent << " commands, " 
           << metrics.button_presses << " button presses\n";
    report << "System: CPU " << metrics.cpu_usage_percent << "%, RAM " 
           << metrics.memory_usage_mb << " MB\n";
    
    return report.str();
}

void PerformanceMonitor::set_bitrate(double bitrate) {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    metrics_.current_bitrate = bitrate;
}

void PerformanceMonitor::set_packet_loss_rate(double loss_rate) {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    metrics_.packet_loss_rate = loss_rate;
}

void PerformanceMonitor::set_target_fps(double fps) {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    metrics_.target_fps = fps;
}

void PerformanceMonitor::update_fps_metrics() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fps_update_).count();
    
    if (elapsed >= 1000) { // Update every second
        uint64_t frames_diff = metrics_.frames_decoded - last_frames_decoded_;
        metrics_.current_fps = frames_diff * 1000.0 / elapsed;
        
        last_fps_update_ = now;
        last_frames_decoded_ = metrics_.frames_decoded;
    }
}

void PerformanceMonitor::update_system_metrics() {
    metrics_.cpu_usage_percent = get_cpu_usage();
    metrics_.memory_usage_mb = get_memory_usage();
}

double PerformanceMonitor::get_cpu_usage() {
    static auto last_time = std::chrono::steady_clock::now();
    static long long last_total_time = 0;
    static long long last_idle_time = 0;
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
    
    // Only update every 2 seconds to avoid too frequent reads
    if (elapsed < 2000) {
        return metrics_.cpu_usage_percent;
    }
    
    std::ifstream stat_file("/proc/stat");
    if (!stat_file.is_open()) {
        return 0.0;
    }
    
    std::string line;
    std::getline(stat_file, line);
    
    // Parse CPU times from /proc/stat
    long long user, nice, system, idle, iowait, irq, softirq, steal;
    if (sscanf(line.c_str(), "cpu %lld %lld %lld %lld %lld %lld %lld %lld",
               &user, &nice, &system, &idle, &iowait, &irq, &softirq, &steal) == 8) {
        
        long long total_time = user + nice + system + idle + iowait + irq + softirq + steal;
        long long idle_time = idle + iowait;
        
        if (last_total_time > 0) {
            long long total_diff = total_time - last_total_time;
            long long idle_diff = idle_time - last_idle_time;
            
            if (total_diff > 0) {
                double cpu_usage = 100.0 * (total_diff - idle_diff) / total_diff;
                last_time = now;
                last_total_time = total_time;
                last_idle_time = idle_time;
                return cpu_usage;
            }
        }
        
        last_total_time = total_time;
        last_idle_time = idle_time;
    }
    
    last_time = now;
    return 0.0;
}

double PerformanceMonitor::get_memory_usage() {
    std::ifstream status_file("/proc/self/status");
    if (!status_file.is_open()) {
        return 0.0;
    }
    
    std::string line;
    while (std::getline(status_file, line)) {
        if (line.substr(0, 6) == "VmRSS:") {
            size_t pos = line.find_first_of("0123456789");
            if (pos != std::string::npos) {
                long kb = std::stol(line.substr(pos));
                return kb / 1024.0; // Convert KB to MB
            }
        }
    }
    
    return 0.0;
}

} // namespace utils
