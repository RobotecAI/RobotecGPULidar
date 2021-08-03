#pragma once

#include <algorithm>
#include <chrono>
#include <fmt/color.h>
#include <fmt/os.h>
#include <functional>
#include <map>
#include <numeric>
#include <vector>

#define PROFILE_GPU_LIDAR 0

struct PerfProbe {
    using InternalClock = std::chrono::high_resolution_clock;

    static std::map<std::string, InternalClock::time_point> clock_start;
    static std::map<std::string, std::vector<double>> clock_measures;

    PerfProbe(std::string tag);

    ~PerfProbe();

    static void saveToFileAndReset();

    std::string tag;
};
