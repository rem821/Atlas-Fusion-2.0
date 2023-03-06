#pragma once
#include <sstream>

#define HEX_ADDR(addr) \
({ std::stringstream ss; ss << "0x" << std::hex << reinterpret_cast<uintptr_t>(addr); ss.str(); })