#include "utils/logger.h"

#include <iostream>
#include <chrono>
#include <iomanip>
#include <cstdarg>
#include <cstring>
#include <ctime>

namespace utils {

Logger& Logger::instance() {
    static Logger instance;
    return instance;
}

void Logger::initialize(const std::string& log_file, LogLevel level) {
    std::lock_guard<std::mutex> lock(log_mutex_);
    
    current_level_ = level;
    
    if (!log_file.empty()) {
        log_file_ = std::make_unique<std::ofstream>(log_file, std::ios::app);
        if (!log_file_->is_open()) {
            std::cerr << "Failed to open log file: " << log_file << std::endl;
            log_file_.reset();
        }
    }
    
    initialized_ = true;
}

void Logger::set_level(LogLevel level) {
    current_level_ = level;
}

void Logger::set_console_output(bool enable) {
    console_output_ = enable;
}

void Logger::log(LogLevel level, const std::string& message, const char* file, int line) {
    if (!initialized_ || level < current_level_) {
        return;
    }
    
    write_log(level, message, file, line);
}

void Logger::debug(const std::string& message, const char* file, int line) {
    log(LogLevel::DEBUG, message, file, line);
}

void Logger::info(const std::string& message, const char* file, int line) {
    log(LogLevel::INFO, message, file, line);
}

void Logger::warning(const std::string& message, const char* file, int line) {
    log(LogLevel::WARNING, message, file, line);
}

void Logger::error(const std::string& message, const char* file, int line) {
    log(LogLevel::ERROR, message, file, line);
}

void Logger::fatal(const std::string& message, const char* file, int line) {
    log(LogLevel::FATAL, message, file, line);
}

void Logger::log_rt(LogLevel level, const char* format, ...) {
    if (!initialized_ || level < current_level_) {
        return;
    }
    
    // RT-optimized logging with pre-allocated buffer
    static thread_local char buffer[1024];
    
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    write_log(level, std::string(buffer), nullptr, 0);
}

void Logger::flush() {
    std::lock_guard<std::mutex> lock(log_mutex_);
    if (log_file_) {
        log_file_->flush();
    }
    std::cout.flush();
    std::cerr.flush();
}

void Logger::close() {
    std::lock_guard<std::mutex> lock(log_mutex_);
    if (log_file_) {
        log_file_->close();
        log_file_.reset();
    }
    initialized_ = false;
}

std::string Logger::level_to_string(LogLevel level) const {
    switch (level) {
        case LogLevel::DEBUG: return "DEBUG";
        case LogLevel::INFO: return "INFO";
        case LogLevel::WARNING: return "WARNING";
        case LogLevel::ERROR: return "ERROR";
        case LogLevel::FATAL: return "FATAL";
        default: return "UNKNOWN";
    }
}

std::string Logger::get_timestamp() const {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    oss << '.' << std::setfill('0') << std::setw(3) << ms.count();
    
    return oss.str();
}

void Logger::write_log(LogLevel level, const std::string& message, const char* file, int line) {
    std::lock_guard<std::mutex> lock(log_mutex_);
    
    std::ostringstream log_line;
    log_line << "[" << get_timestamp() << "] [" << level_to_string(level) << "] ";
    
    if (file && line > 0) {
        // Extract just the filename from full path
        const char* filename = strrchr(file, '/');
        filename = filename ? filename + 1 : file;
        log_line << "[" << filename << ":" << line << "] ";
    }
    
    log_line << message;
    
    // Write to file if available
    if (log_file_ && log_file_->is_open()) {
        *log_file_ << log_line.str() << std::endl;
    }
    
    // Write to console if enabled
    if (console_output_) {
        if (level >= LogLevel::ERROR) {
            std::cerr << log_line.str() << std::endl;
        } else {
            std::cout << log_line.str() << std::endl;
        }
    }
}

// LogStream implementation
LogStream::LogStream(LogLevel level, const char* file, int line)
    : level_(level), file_(file), line_(line) {
}

LogStream::~LogStream() {
    Logger::instance().log(level_, stream_.str(), file_, line_);
}

} // namespace utils
