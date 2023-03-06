//
// Created by standa on 6.3.23.
//
#pragma once

#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/fmt/ostr.h"

namespace AtlasFusion {

    class Log {
    public:
        static void Init();

        inline static std::shared_ptr<spdlog::logger>& GetLogger() { return logger_; }
    private:
        static std::shared_ptr<spdlog::logger> logger_;
    };
}

// log macros
#define LOG_CRITICAL(...)  ::AtlasFusion::Log::GetLogger()->critical(__VA_ARGS__)
#define LOG_ERROR(...)     ::AtlasFusion::Log::GetLogger()->error(__VA_ARGS__)
#define LOG_WARN(...)      ::AtlasFusion::Log::GetLogger()->warn(__VA_ARGS__)
#define LOG_INFO(...)      ::AtlasFusion::Log::GetLogger()->info(__VA_ARGS__)
//#define LOG_TRACE(...)     AtlasFusion::Log::GetLogger()->trace(__VA_ARGS__)
#define LOG_TRACE(...)