// Debug.hpp
#pragma once

#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>

enum class LogLevel { DEBUG, INFO, WARNING, ERROR, FATAL };

class Logger {
   public:
	static Logger& getInstance() {
		static Logger instance;
		return instance;
	}

	void init(const std::string& logFilePath, LogLevel minLevel = LogLevel::INFO) {
		std::lock_guard<std::mutex> lock(mutex_);
		logFile_.open(logFilePath, std::ios::out | std::ios::app);
		minLevel_ = minLevel;

		if (logFile_.is_open()) {
			logFile_ << "\n\n--- Log Session Started: " << getCurrentTimeStamp() << " ---\n\n";
			initialized_ = true;
		} else {
			std::cerr << "Failed to open log file: " << logFilePath << std::endl;
		}
	}

	void log(LogLevel level, const std::string& message, const std::string& file, int line) {
		if (level < minLevel_)
			return;

		std::string levelStr;
		switch (level) {
		case LogLevel::DEBUG:
			levelStr = "DEBUG";
			break;
		case LogLevel::INFO:
			levelStr = "INFO";
			break;
		case LogLevel::WARNING:
			levelStr = "WARNING";
			break;
		case LogLevel::ERROR:
			levelStr = "ERROR";
			break;
		case LogLevel::FATAL:
			levelStr = "FATAL";
			break;
		}

		std::stringstream ss;
		ss << getCurrentTimeStamp() << " [" << levelStr << "] " << file << ":" << line << " - " << message;

		std::lock_guard<std::mutex> lock(mutex_);

		// Output to console
		std::cout << ss.str() << std::endl;

		// Output to file if initialized
		if (initialized_ && logFile_.is_open()) {
			logFile_ << ss.str() << std::endl;
			logFile_.flush();
		}
	}

	~Logger() {
		if (logFile_.is_open()) {
			std::lock_guard<std::mutex> lock(mutex_);
			logFile_ << "\n--- Log Session Ended: " << getCurrentTimeStamp() << " ---\n";
			logFile_.close();
		}
	}

   private:
	Logger() : initialized_(false), minLevel_(LogLevel::INFO) {}
	Logger(const Logger&) = delete;
	Logger& operator=(const Logger&) = delete;

	std::string getCurrentTimeStamp() {
		auto now = std::chrono::system_clock::now();
		auto time = std::chrono::system_clock::to_time_t(now);
		auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

		std::stringstream ss;
		ss << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S") << '.' << std::setfill('0')
		   << std::setw(3) << ms.count();
		return ss.str();
	}

	std::ofstream logFile_;
	std::mutex mutex_;
	bool initialized_;
	LogLevel minLevel_;
};

// Convenience macros
#define LOG_DEBUG(message) Logger::getInstance().log(LogLevel::DEBUG, message, __FILE__, __LINE__)

#define LOG_INFO(message) Logger::getInstance().log(LogLevel::INFO, message, __FILE__, __LINE__)

#define LOG_WARNING(message) Logger::getInstance().log(LogLevel::WARNING, message, __FILE__, __LINE__)

#define LOG_ERROR(message) Logger::getInstance().log(LogLevel::ERROR, message, __FILE__, __LINE__)

#define LOG_FATAL(message) Logger::getInstance().log(LogLevel::FATAL, message, __FILE__, __LINE__)