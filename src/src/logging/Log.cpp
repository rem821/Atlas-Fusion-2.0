//
// Created by standa on 6.3.23.
//
#include <logging/Log.h>

namespace AtlasFusion {

    std::shared_ptr<spdlog::logger> Log::logger_;

    void Log::Init() {
        spdlog::set_pattern("%^[%T] %n: %v%$");

        logger_ = spdlog::stdout_color_mt("ATLAS_FUSION");
        logger_->set_level(spdlog::level::trace);
    }
}