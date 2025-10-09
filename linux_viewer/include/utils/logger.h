#pragma once

#include <string>
#include <fstream>
#include <mutex>
#include <memory>

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
    
    void initialize(const std::string& log_file = "", LogLevel level = LogLevel::INFO, bool console_output = true);
    void close();
    
    void log(LogLevel level, const std::string& message, const std::string& file = "", int line = 0);
    
    void set_log_level(LogLevel level);
    LogLevel get_log_level() const;
    
private:
    Logger() = default;
    ~Logger();
    
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    
    std::mutex mutex_;
    std::unique_ptr<std::ofstream> file_stream_;
    LogLevel log_level_ = LogLevel::INFO;
    bool console_output_ = true;
    bool initialized_ = false;
    
    std::string level_to_string(LogLevel level) const;
    std::string get_timestamp() const;
};

// Convenience macros
#define LOG_DEBUG(msg) utils::Logger::instance().log(utils::LogLevel::DEBUG, msg, __FILE__, __LINE__)
#define LOG_INFO(msg) utils::Logger::instance().log(utils::LogLevel::INFO, msg, __FILE__, __LINE__)
#define LOG_WARNING(msg) utils::Logger::instance().log(utils::LogLevel::WARNING, msg, __FILE__, __LINE__)
#define LOG_ERROR(msg) utils::Logger::instance().log(utils::LogLevel::ERROR, msg, __FILE__, __LINE__)
#define LOG_FATAL(msg) utils::Logger::instance().log(utils::LogLevel::FATAL, msg, __FILE__, __LINE__)

} // namespace utils
