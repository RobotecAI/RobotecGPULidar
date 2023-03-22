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

#ifdef RGL_BUILD_ROS2_EXTENSION
#include <builtin_interfaces/msg/time.hpp>
#endif

struct Time
{
	static Time zero() { return Time(0); }
	static Time seconds(double seconds) { return Time(static_cast<uint64_t>(seconds * 1.0e9)); }
	static Time nanoseconds(uint64_t nanoseconds) { return Time(nanoseconds); }

	double asSeconds() { return static_cast<double>(timeNs) * 1.0e-9; };
	uint64_t asNanoseconds() { return timeNs; }

#ifdef RGL_BUILD_ROS2_EXTENSION
	builtin_interfaces::msg::Time asRos2Msg()
	{
		auto msg = builtin_interfaces::msg::Time();
		msg.sec = timeNs / static_cast<uint64_t>(1e9);
		msg.nanosec = timeNs % static_cast<uint64_t>(1e9);
		return msg;
	}
#endif

private:
	Time(uint64_t nanoseconds) { timeNs = nanoseconds; }

	uint64_t timeNs;
};
