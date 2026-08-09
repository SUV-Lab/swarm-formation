#ifndef LOG_MANAGER_HPP
#define LOG_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <string>
#include <memory>
#include <mutex>
#include <chrono>
#include <iomanip>
#include <sstream>

namespace swarm_formation {

/**
 * @brief Integrated log manager class
 * 
 * ROS2 node's all logs are saved and managed by this class.
 * - Real-time log file saving
 * - Log level filtering
 * - Timestamp automatically added
 * - Thread safety
 */
class LogManager {
public:
    using Ptr = std::shared_ptr<LogManager>;
    
    enum LogLevel {
        DEBUG = 0,
        INFO = 1,
        WARN = 2,
        ERROR = 3,
        FATAL = 4
    };

    /**
     * @brief LogManager constructor
     * @param node_name Node name (used in log file name)
     * @param log_dir Log save directory (default: "./logs/runtime")
     * @param min_level Minimum log level (default: INFO)
     */
    LogManager(const std::string& node_name, 
               const std::string& log_dir = "./logs/runtime",
               LogLevel min_level = INFO);

    ~LogManager();

    /**
     * @brief Log message recording
     */
    void log(LogLevel level, const std::string& message);
    void debug(const std::string& message);
    void info(const std::string& message);
    void warn(const std::string& message);
    void error(const std::string& message);
    void fatal(const std::string& message);

    /**
     * @brief Formatted log message recording (printf style)
     */
    template<typename... Args>
    void logf(LogLevel level, const std::string& format, Args... args);

    template<typename... Args>
    void debugf(const std::string& format, Args... args);

    template<typename... Args>
    void infof(const std::string& format, Args... args);

    template<typename... Args>
    void warnf(const std::string& format, Args... args);

    template<typename... Args>
    void errorf(const std::string& format, Args... args);

    template<typename... Args>
    void fatalf(const std::string& format, Args... args);

    /**
     * @brief Flush the log file (force save)
     */
    void flush();

    /**
     * @brief Set the minimum log level
     */
    void setMinLevel(LogLevel level);

    /**
     * @brief Rotate the log file
     */
    void rotateLog();

private:
    std::string node_name_;
    std::string log_dir_;
    LogLevel min_level_;
    std::unique_ptr<std::ofstream> log_file_;
    std::mutex mutex_;
    
    std::string getCurrentTimestamp();
    std::string getLevelString(LogLevel level);
    void createLogDirectory();
    void openLogFile();
    std::string formatString(const std::string& format, ...);
};

// Template 구현부
template<typename... Args>
void LogManager::logf(LogLevel level, const std::string& format, Args... args) {
    if (level < min_level_) return;
    
    char buffer[1024];
    snprintf(buffer, sizeof(buffer), format.c_str(), args...);
    log(level, std::string(buffer));
}

template<typename... Args>
void LogManager::debugf(const std::string& format, Args... args) {
    logf(DEBUG, format, args...);
}

template<typename... Args>
void LogManager::infof(const std::string& format, Args... args) {
    logf(INFO, format, args...);
}

template<typename... Args>
void LogManager::warnf(const std::string& format, Args... args) {
    logf(WARN, format, args...);
}

template<typename... Args>
void LogManager::errorf(const std::string& format, Args... args) {
    logf(ERROR, format, args...);
}

template<typename... Args>
void LogManager::fatalf(const std::string& format, Args... args) {
    logf(FATAL, format, args...);
}

/**
 * @brief ROS2 node log manager wrapper class
 */
class RosLogManager {
public:
    RosLogManager(rclcpp::Node::SharedPtr node, 
                  const std::string& log_dir = "./logs/runtime",
                  LogManager::LogLevel min_level = LogManager::INFO);

    void setDebugLoggingEnabled(bool enabled);
    bool isDebugLoggingEnabled() const;

    void debug(const std::string& message);
    void info(const std::string& message);
    void warn(const std::string& message);
    void error(const std::string& message);
    void fatal(const std::string& message);

    void debug_conditional(const std::string& message);
    void info_conditional(const std::string& message);

    template<typename... Args>
    void debugf(const std::string& format, Args... args);

    template<typename... Args>
    void infof(const std::string& format, Args... args);

    template<typename... Args>
    void warnf(const std::string& format, Args... args);

    template<typename... Args>
    void errorf(const std::string& format, Args... args);

    template<typename... Args>
    void fatalf(const std::string& format, Args... args);

    template<typename... Args>
    void debugf_conditional(const std::string& format, Args... args);

    template<typename... Args>
    void infof_conditional(const std::string& format, Args... args);

    void flush();
    void rotateLog();

private:
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<LogManager> log_manager_;
    bool debug_logging_enabled_;
};

template<typename... Args>
void RosLogManager::debugf(const std::string& format, Args... args) {
    char buffer[1024];
    snprintf(buffer, sizeof(buffer), format.c_str(), args...);
    debug(std::string(buffer));
}

template<typename... Args>
void RosLogManager::infof(const std::string& format, Args... args) {
    char buffer[1024];
    snprintf(buffer, sizeof(buffer), format.c_str(), args...);
    info(std::string(buffer));
}

template<typename... Args>
void RosLogManager::warnf(const std::string& format, Args... args) {
    char buffer[1024];
    snprintf(buffer, sizeof(buffer), format.c_str(), args...);
    warn(std::string(buffer));
}

template<typename... Args>
void RosLogManager::errorf(const std::string& format, Args... args) {
    char buffer[1024];
    snprintf(buffer, sizeof(buffer), format.c_str(), args...);
    error(std::string(buffer));
}

template<typename... Args>
void RosLogManager::fatalf(const std::string& format, Args... args) {
    char buffer[1024];
    snprintf(buffer, sizeof(buffer), format.c_str(), args...);
    fatal(std::string(buffer));
}

template<typename... Args>
void RosLogManager::debugf_conditional(const std::string& format, Args... args) {
    if (debug_logging_enabled_) {
        char buffer[1024];
        snprintf(buffer, sizeof(buffer), format.c_str(), args...);
        debug(std::string(buffer));
    }
}

template<typename... Args>
void RosLogManager::infof_conditional(const std::string& format, Args... args) {
    if (debug_logging_enabled_) {
        char buffer[1024];
        snprintf(buffer, sizeof(buffer), format.c_str(), args...);
        info(std::string(buffer));
    }
}

} // namespace swarm_formation

#define LOG_INFO_CONDITIONAL(log_manager, message_string_only) \
    do { \
        if ((log_manager)->isDebugLoggingEnabled()) { \
            (log_manager)->info(message_string_only); \
        } \
    } while(0)

#define LOG_INFOF_CONDITIONAL(log_manager, ...) \
    do { \
        if ((log_manager)->isDebugLoggingEnabled()) { \
            (log_manager)->infof(__VA_ARGS__); \
        } \
    } while(0)

#define LOG_DEBUG_CONDITIONAL(log_manager, message_string_only) \
    do { \
        if ((log_manager)->isDebugLoggingEnabled()) { \
            (log_manager)->debug(message_string_only); \
        } \
    } while(0)

#define LOG_DEBUGF_CONDITIONAL(log_manager, ...) \
    do { \
        if ((log_manager)->isDebugLoggingEnabled()) { \
            (log_manager)->debugf(__VA_ARGS__); \
        } \
    } while(0)

#endif // LOG_MANAGER_HPP
