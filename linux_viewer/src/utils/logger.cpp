#include "utils/logger.h"
#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>

namespace utils {

Logger& Logger::instance() {
    static Logger instance;
    return instance;
}

Logger::~Logger() {
    close();
}

void Logger::initialize(const std::string& log_file, LogLevel level, bool console_output) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    log_level_ = level;
    console_output_ = console_output;
    
    if (!log_file.empty()) {
        file_stream_ = std::make_unique<std::ofstream>(log_file, std::ios::app);
        if (!file_stream_->is_open()) {
            std::cerr << "Failed to open log file: " << log_file << std::endl;
            file_stream_.reset();
        }
    }
    
    initialized_ = true;
}

void Logger::close() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (file_stream_) {
        file_stream_->close();
        file_stream_.reset();
    }
    
    initialized_ = false;
}

void Logger::log(LogLevel level, const std::string& message, const std::string& file, int line) {
    if (!initialized_ || level < log_level_) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::string timestamp = get_timestamp();
    std::string level_str = level_to_string(level);
    
    // Extract filename from full path
    std::string filename = file;
    size_t pos = filename.find_last_of("/\\");
    if (pos != std::string::npos) {
        filename = filename.substr(pos + 1);
    }
    
    std::ostringstream log_line;
    log_line << "[" << timestamp << "] [" << level_str << "]";
    
    if (!filename.empty() && line > 0) {
        log_line << " [" << filename << ":" << line << "]";
    }
    
    log_line << " " << message;
    
    // Output to console
    if (console_output_) {
        if (level >= LogLevel::ERROR) {
            std::cerr << log_line.str() << std::endl;
        } else {
            std::cout << log_line.str() << std::endl;
        }
    }
    
    // Output to file
    if (file_stream_ && file_stream_->is_open()) {
        *file_stream_ << log_line.str() << std::endl;
        file_stream_->flush();
    }
}

void Logger::set_log_level(LogLevel level) {
    std::lock_guard<std::mutex> lock(mutex_);
    log_level_ = level;
}

LogLevel Logger::get_log_level() const {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(mutex_));
    return log_level_;
}

std::string Logger::level_to_string(LogLevel level) const {
    switch (level) {
        case LogLevel::DEBUG:   return "DEBUG";
        case LogLevel::INFO:    return "INFO";
        case LogLevel::WARNING: return "WARN";
        case LogLevel::ERROR:   return "ERROR";
        case LogLevel::FATAL:   return "FATAL";
        default:                return "UNKNOWN";
    }
}

std::string Logger::get_timestamp() const {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    oss << "." << std::setfill('0') << std::setw(3) << ms.count();
    
    return oss.str();
}

} // namespace utils
