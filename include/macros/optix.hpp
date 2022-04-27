#pragma once

#define CHECK_OPTIX(call)                                                                    \
do {                                                                                         \
	OptixResult res = call;                                                                  \
	if (res != OPTIX_SUCCESS) {                                                                 \
		auto msg = fmt::format("Optix Error: {} -> {} ({}) @ {}:{}",                \
		#call, static_cast<int>(res), optixGetErrorName(res), __FILE__, __LINE__); \
		throw std::runtime_error(msg);                                                       \
	}                                                                                        \
} while (0)
