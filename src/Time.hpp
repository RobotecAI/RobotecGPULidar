// Copyright 2023 Robotec.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <macros/cuda.hpp>

#if RGL_BUILD_ROS2_EXTENSION
#include <builtin_interfaces/msg/time.hpp>
#endif

struct Time
{
	friend Time operator-(const Time& lhs, const Time& rhs) noexcept;
	// TODO: We can use std::strong_ordering if we upgrade to CUDA 12
	friend bool operator==(const Time& lhs, const Time& rhs) noexcept;

	static Time zero() { return Time(0); }
	static Time seconds(double seconds) { return Time(static_cast<uint64_t>(seconds * 1.0e9)); }
	static Time nanoseconds(uint64_t nanoseconds) { return Time(nanoseconds); }

	HostDevFn double asSeconds() const { return static_cast<double>(timeNs) * 1.0e-9; };
	HostDevFn double asMilliseconds() const { return static_cast<double>(timeNs) * 1.0e-6; };
	HostDevFn uint64_t asNanoseconds() const { return timeNs; }

#if RGL_BUILD_ROS2_EXTENSION
	builtin_interfaces::msg::Time asRos2Msg() const
	{
		auto msg = builtin_interfaces::msg::Time();
		msg.sec = timeNs / static_cast<uint64_t>(1e9);
		msg.nanosec = timeNs % static_cast<uint64_t>(1e9);
		return msg;
	}
#endif

private:
	explicit Time(uint64_t nanoseconds) { timeNs = nanoseconds; }

	uint64_t timeNs;
};

inline Time operator-(const Time& lhs, const Time& rhs) noexcept { return Time::nanoseconds(lhs.timeNs - rhs.timeNs); }
inline bool operator==(const Time& lhs, const Time& rhs) noexcept { return lhs.timeNs == rhs.timeNs; }
