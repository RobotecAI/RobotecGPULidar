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
	using SecNanosecPair = std::pair<int32_t, uint32_t>;

	static Time zero()
	{ return Time(0, 0); }

	Time(int32_t seconds, uint32_t nanoseconds)
	{ time = std::make_pair(seconds, nanoseconds); }
	Time(double seconds)
	{ time = std::make_pair(static_cast<int32_t>(seconds), static_cast<uint32_t>(seconds * 1.0e9)); };

	double asDouble()
	{ return static_cast<double>(time.first) + static_cast<double>(time.second) * 1.0e-9; };
	SecNanosecPair asSecNanosec()
	{ return time; };

	#ifdef RGL_BUILD_ROS2_EXTENSION
	builtin_interfaces::msg::Time asRos2Msg()
	{
		auto msg = builtin_interfaces::msg::Time();
		msg.sec = time.first;
		msg.nanosec = time.second;
		return msg;
	}
	#endif

	inline bool operator==(const Time& other) const
	{ return time.first == other.time.first && time.second == other.time.second; }

private:
	SecNanosecPair time;
};
