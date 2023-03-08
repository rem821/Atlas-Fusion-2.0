#pragma once

#include <sstream>

#define HEX_ADDR(addr) \
({ std::stringstream ss; ss << "0x" << std::hex << reinterpret_cast<uintptr_t>(addr); ss.str(); })

#define STAMP_TO_NANOSEC(stamp) (((uint64_t)stamp.sec * 1000000000) + stamp.nanosec)
#define NANOSEC_TO_STAMP_SEC(nanosec) (std::floor((double) nanosec / 1000000000.0f))
#define NANOSEC_TO_STAMP_NANOSEC(nanosec) (nanosec - (NANOSEC_TO_STAMP_SEC(nanosec) * 1000000000))