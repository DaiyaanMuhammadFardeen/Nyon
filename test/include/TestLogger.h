#pragma once

#include <iostream>
#include <sstream>
#include <string>
#include <chrono>
#include <iomanip>

/**
 * @brief Advanced test logger with variable capture and debugging capabilities.
 * 
 * This logger automatically captures variable values during test execution
 * and provides detailed debugging information without manual step-through debugging.
 */
class TestLogger
{
public:
    enum class LogLevel
    {
        DEBUG,
        INFO,
        WARNING,
        ERROR,
        FATAL
    };

    static TestLogger& GetInstance()
    {
        static TestLogger instance;
        return instance;
    }

    template<typename T>
    void Log(LogLevel level, const std::string& message, const T& value, 
             const std::string& variableName = "", int lineNumber = 0, 
             const std::string& functionName = "", const std::string& fileName = "")
    {
        std::stringstream ss;
        ss << "[" << GetCurrentTimestamp() << "] ";
        ss << "[" << LogLevelToString(level) << "] ";
        
        if (!fileName.empty()) {
            ss << "[" << ExtractFileName(fileName) << ":" << lineNumber << "] ";
        }
        
        if (!functionName.empty()) {
            ss << "[" << functionName << "] ";
        }
        
        ss << message;
        
        if (!variableName.empty()) {
            ss << " | Variable '" << variableName << "' = " << value;
        }
        
        std::cout << ss.str() << std::endl;
        
        // Store in history for test failure analysis
        m_LogHistory.push_back(ss.str());
    }

    void LogSimple(LogLevel level, const std::string& message)
    {
        Log(level, message, "", "", 0, "", "");
    }

    const std::vector<std::string>& GetLogHistory() const
    {
        return m_LogHistory;
    }

    void ClearLogHistory()
    {
        m_LogHistory.clear();
    }

    // Macro helpers for automatic line/function capture
    template<typename T>
    void LogVar(LogLevel level, const T& value, const std::string& varName)
    {
        Log(level, "Variable captured", value, varName, __LINE__, __FUNCTION__, __FILE__);
    }

private:
    TestLogger() = default;
    
    std::string LogLevelToString(LogLevel level)
    {
        switch (level) {
            case LogLevel::DEBUG: return "DEBUG";
            case LogLevel::INFO: return "INFO";
            case LogLevel::WARNING: return "WARN";
            case LogLevel::ERROR: return "ERROR";
            case LogLevel::FATAL: return "FATAL";
            default: return "UNKNOWN";
        }
    }

    std::string GetCurrentTimestamp()
    {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t), "%H:%M:%S");
        ss << "." << std::setfill('0') << std::setw(3) << ms.count();
        return ss.str();
    }

    std::string ExtractFileName(const std::string& fullPath)
    {
        size_t pos = fullPath.find_last_of("/\\");
        return pos != std::string::npos ? fullPath.substr(pos + 1) : fullPath;
    }

    std::vector<std::string> m_LogHistory;
};

// Convenience macros for easy logging
#define LOG_DEBUG(msg) TestLogger::GetInstance().LogSimple(TestLogger::LogLevel::DEBUG, msg)
#define LOG_INFO(msg) TestLogger::GetInstance().LogSimple(TestLogger::LogLevel::INFO, msg)
#define LOG_WARN(msg) TestLogger::GetInstance().LogSimple(TestLogger::LogLevel::WARNING, msg)
#define LOG_ERROR(msg) TestLogger::GetInstance().LogSimple(TestLogger::LogLevel::ERROR, msg)

#define LOG_VAR_DEBUG(var) TestLogger::GetInstance().LogVar(TestLogger::LogLevel::DEBUG, var, #var)
#define LOG_VAR_INFO(var) TestLogger::GetInstance().LogVar(TestLogger::LogLevel::INFO, var, #var)

#define LOG_FUNC_ENTER() TestLogger::GetInstance().Log(TestLogger::LogLevel::DEBUG, \
    "Entering function", "", "", __LINE__, __FUNCTION__, __FILE__)
#define LOG_FUNC_EXIT() TestLogger::GetInstance().Log(TestLogger::LogLevel::DEBUG, "Exiting function", "", "", __LINE__, __FUNCTION__, __FILE__)