#include "log_manager.hpp"
#include <filesystem>
#include <cstdarg>

namespace swarm_formation {

LogManager::LogManager(const std::string& node_name,
                       const std::string& log_dir,
                       LogLevel min_level)
    : node_name_(node_name), log_dir_(log_dir), min_level_(min_level) {
    // Check environment variable to disable logging
    const char* disable_logging = std::getenv("SWARM_DISABLE_FILE_LOGGING");
    if (disable_logging != nullptr && std::string(disable_logging) == "1") {
        // Logging disabled - do not create log file
        return;
    }

    createLogDirectory();
    openLogFile();
}

LogManager::~LogManager() {
    if (log_file_ && log_file_->is_open()) {
        log_file_->close();
    }
}

void LogManager::log(LogLevel level, const std::string& message) {
    if (level < min_level_) return;
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (log_file_ && log_file_->is_open()) {
        *log_file_ << "[" << getCurrentTimestamp() << "] "
                   << "[" << getLevelString(level) << "] "
                   << "[" << node_name_ << "] "
                   << message << std::endl;
        log_file_->flush();
    }
}

void LogManager::debug(const std::string& message) {
    log(DEBUG, message);
}

void LogManager::info(const std::string& message) {
    log(INFO, message);
}

void LogManager::warn(const std::string& message) {
    log(WARN, message);
}

void LogManager::error(const std::string& message) {
    log(ERROR, message);
}

void LogManager::fatal(const std::string& message) {
    log(FATAL, message);
}

void LogManager::flush() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (log_file_ && log_file_->is_open()) {
        log_file_->flush();
    }
}

void LogManager::setMinLevel(LogLevel level) {
    min_level_ = level;
}

void LogManager::rotateLog() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (log_file_ && log_file_->is_open()) {
        log_file_->close();
    }
    openLogFile();
}

std::string LogManager::getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    ss << "." << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}

std::string LogManager::getLevelString(LogLevel level) {
    switch (level) {
        case DEBUG: return "DEBUG";
        case INFO:  return "INFO ";
        case WARN:  return "WARN ";
        case ERROR: return "ERROR";
        case FATAL: return "FATAL";
        default:    return "UNKNW";
    }
}

void LogManager::createLogDirectory() {
    try {
        std::filesystem::create_directories(log_dir_);
    } catch (const std::exception& e) {
        // 디렉토리 생성 실패 시 현재 디렉토리에 저장
        log_dir_ = ".";
    }
}

void LogManager::openLogFile() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    
    std::stringstream filename;
    filename << log_dir_ << "/" << node_name_ << "_"
             << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
             << ".log";
    
    log_file_ = std::make_unique<std::ofstream>(filename.str(), std::ios::app);
    
    if (log_file_->is_open()) {
        *log_file_ << "=== Log started at " << getCurrentTimestamp() 
                   << " for node: " << node_name_ << " ===" << std::endl;
    }
}

RosLogManager::RosLogManager(rclcpp::Node::SharedPtr node, 
                             const std::string& log_dir,
                             LogManager::LogLevel min_level)
    : node_(node), debug_logging_enabled_(false) {
    log_manager_ = std::make_unique<LogManager>(node->get_name(), log_dir, min_level);
}

void RosLogManager::debug(const std::string& message) {
    RCLCPP_DEBUG(node_->get_logger(), "%s", message.c_str());
    log_manager_->debug(message);
}

void RosLogManager::info(const std::string& message) {
    RCLCPP_INFO(node_->get_logger(), "%s", message.c_str());
    log_manager_->info(message);
}

void RosLogManager::warn(const std::string& message) {
    RCLCPP_WARN(node_->get_logger(), "%s", message.c_str());
    log_manager_->warn(message);
}

void RosLogManager::error(const std::string& message) {
    RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
    log_manager_->error(message);
}

void RosLogManager::fatal(const std::string& message) {
    RCLCPP_FATAL(node_->get_logger(), "%s", message.c_str());
    log_manager_->fatal(message);
}

void RosLogManager::flush() {
    log_manager_->flush();
}

void RosLogManager::rotateLog() {
    log_manager_->rotateLog();
}

void RosLogManager::setDebugLoggingEnabled(bool enabled) {
    debug_logging_enabled_ = enabled;
}

bool RosLogManager::isDebugLoggingEnabled() const {
    return debug_logging_enabled_;
}

void RosLogManager::debug_conditional(const std::string& message) {
    if (debug_logging_enabled_) {
        RCLCPP_DEBUG(node_->get_logger(), "%s", message.c_str());
        log_manager_->debug(message);
    }
}

void RosLogManager::info_conditional(const std::string& message) {
    if (debug_logging_enabled_) {
        RCLCPP_INFO(node_->get_logger(), "%s", message.c_str());
        log_manager_->info(message);
    }
}

} // namespace swarm_formation
