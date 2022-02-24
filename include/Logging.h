#pragma once

// TODO(prybicki): Consider more real-world logging with ability to set level and redirect to a file.
#include <fmt/format.h>
#include <fmt/color.h>
using namespace fmt; // forgive me master

template <typename... Args>
void logInfo(Args&&... args) {
#ifndef NDEBUG
    print(stderr, fg(color::cornflower_blue), std::forward<Args>(args)...);
#endif
}

template <typename... Args>
void logWarn(Args&&... args) {
#ifndef NDEBUG
    print(stderr, fg(color::yellow), std::forward<Args>(args)...);
#endif
}

template <typename... Args>
void logError(Args&&... args) {
#ifndef NDEBUG
    print(stderr, fg(color::red), std::forward<Args>(args)...);
#endif
}
