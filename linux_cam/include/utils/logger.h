#pragma once

#include <string>
#include <fstream>
#include <mutex>
#include <atomic>
#include <memory>
#include <sstream>

namespace utils {

enum class LogLevel {
    DEBUG = 0,
    INFO = 1,
    WARNING = 2,
    ERROR = 3,
    FATAL = 4
};

class Logger {
public:
    static Logger& instance();
    
    void initialize(const std::string& log_file, LogLevel level = LogLevel::INFO);
    void set_level(LogLevel level);
    void set_console_output(bool enable);
    
    void log(LogLevel level, const std::string& message, const char* file = nullptr, int line = 0);
    
    // Convenience methods
    void debug(const std::string& message, const char* file = nullptr, int line = 0);
    void info(const std::string& message, const char* file = nullptr, int line = 0);
    void warning(const std::string& message, const char* file = nullptr, int line = 0);
    void error(const std::string& message, const char* file = nullptr, int line = 0);
    void fatal(const std::string& message, const char* file = nullptr, int line = 0);
    
    // Performance-optimized logging for RT applications
    void log_rt(LogLevel level, const char* format, ...);
    
    void flush();
    void close();

private:
    Logger() = default;
    
    std::mutex log_mutex_;
    std::unique_ptr<std::ofstream> log_file_;
    std::atomic<LogLevel> current_level_{LogLevel::INFO};
    std::atomic<bool> console_output_{true};
    std::atomic<bool> initialized_{false};
    
    std::string level_to_string(LogLevel level) const;
    std::string get_timestamp() const;
    void write_log(LogLevel level, const std::string& message, const char* file, int line);
};

// Stream-based logging for convenience
class LogStream {
public:
    LogStream(LogLevel level, const char* file = nullptr, int line = 0);
    ~LogStream();
    
    template<typename T>
    LogStream& operator<<(const T& value) {
        stream_ << value;
        return *this;
    }

private:
    LogLevel level_;
    const char* file_;
    int line_;
    std::ostringstream stream_;
};

} // namespace utils

// Convenience macros for logging with file/line info
#define LOG_DEBUG(msg) utils::Logger::instance().debug(msg, __FILE__, __LINE__)
#define LOG_INFO(msg) utils::Logger::instance().info(msg, __FILE__, __LINE__)
#define LOG_WARNING(msg) utils::Logger::instance().warning(msg, __FILE__, __LINE__)
#define LOG_ERROR(msg) utils::Logger::instance().error(msg, __FILE__, __LINE__)
#define LOG_FATAL(msg) utils::Logger::instance().fatal(msg, __FILE__, __LINE__)

// Stream-based logging
#define LOG_STREAM(level) utils::LogStream(level, __FILE__, __LINE__)
#define LOG_DEBUG_STREAM() LOG_STREAM(utils::LogLevel::DEBUG)
#define LOG_INFO_STREAM() LOG_STREAM(utils::LogLevel::INFO)
#define LOG_WARNING_STREAM() LOG_STREAM(utils::LogLevel::WARNING)
#define LOG_ERROR_STREAM() LOG_STREAM(utils::LogLevel::ERROR)
#define LOG_FATAL_STREAM() LOG_STREAM(utils::LogLevel::FATAL)
