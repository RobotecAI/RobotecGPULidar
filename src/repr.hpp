// Copyright 2022 Robotec.AI
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

#include <rgl/api/core.h>
#include <spdlog/fmt/fmt.h>

template<>
struct fmt::formatter<rgl_vec2f>
{
	template<typename ParseContext>
	constexpr auto parse(ParseContext& ctx)
	{
		return ctx.begin();
	}

	template<typename FormatContext>
	auto format(const rgl_vec2f& v, FormatContext& ctx)
	{
		return fmt::format_to(ctx.out(), "({}, {})", v.value[0], v.value[1]);
	}
};

template<>
struct fmt::formatter<rgl_vec3f>
{
	template<typename ParseContext>
	constexpr auto parse(ParseContext& ctx)
	{
		return ctx.begin();
	}

	template<typename FormatContext>
	auto format(const rgl_vec3f& v, FormatContext& ctx)
	{
		return fmt::format_to(ctx.out(), "({}, {}, {})", v.value[0], v.value[1], v.value[2]);
	}
};

template<>
struct fmt::formatter<rgl_vec3i>
{
	template<typename ParseContext>
	constexpr auto parse(ParseContext& ctx)
	{
		return ctx.begin();
	}

	template<typename FormatContext>
	auto format(const rgl_vec3i& v, FormatContext& ctx)
	{
		return fmt::format_to(ctx.out(), "({}, {}, {})", v.value[0], v.value[1], v.value[2]);
	}
};

template<>
struct fmt::formatter<rgl_mat3x4f>
{
	template<typename ParseContext>
	constexpr auto parse(ParseContext& ctx)
	{
		return ctx.begin();
	}

	template<typename FormatContext>
	auto format(const rgl_mat3x4f& m, FormatContext& ctx)
	{
		// TODO: add scale and rotation, if you are bored enough
		return fmt::format_to(ctx.out(), "Mat3x4{{T={{{}, {}, {}}}}}", m.value[0][3], m.value[1][3], m.value[2][3]);
	}
};

template<typename ArrayT>
std::string repr(ArrayT* elements, long long elemCount = 1, int elemLimit = 3)
{
#define PUSH(...) fmt::format_to(std::back_inserter(out), __VA_ARGS__)
	if (elements == nullptr) {
		return "<null>";
	}
	if (elemCount == 0) {
		return "<empty>";
	}
	int suffixLimit = elemLimit / 2;
	int prefixLimit = elemLimit - suffixLimit; // Prefix is +1 longer in case of uneven limit
	int prefixLength = std::min(prefixLimit, (int) elemCount);
	int suffixLength = std::min(suffixLimit, (int) elemCount - prefixLength);
	auto out = fmt::memory_buffer();

	if (elemCount > 1) {
		PUSH("[{}]", elemCount);
		PUSH("{{");
	}

	for (int i = 0; i < prefixLength; ++i) {
		PUSH("{}", elements[i]);
		if (i != prefixLength - 1) {
			PUSH(", ");
		}
	}

	int sum = prefixLength + suffixLength;
	if (0 < sum && sum < elemCount) {
		PUSH(", ...");
	}

	// Suffix
	if (suffixLength > 0) {
		PUSH(", ");
		for (int i = elemCount - suffixLength; i < elemCount; ++i) {
			PUSH("{}", elements[i]);
			if (i != elemCount - 1) {
				PUSH(", ");
			}
		}
	}
	if (elemCount > 1) {
		PUSH("}}");
	}
	return to_string(out);
#undef PUSH
}

static inline std::string repr(rgl_node_t node) { return fmt::format("{}", (void*) node); }

static inline std::string repr(rgl_node_t* node)
{
	std::string nodePointee = (node != nullptr) ? fmt::format("->{}", repr(*node)) : "";
	return fmt::format("{}{}", (void*) node, nodePointee);
}

template<>
struct fmt::formatter<rgl_radar_scope_t>
{
	template<typename ParseContext>
	constexpr auto parse(ParseContext& ctx)
	{
		return ctx.begin();
	}

	template<typename FormatContext>
	auto format(const rgl_radar_scope_t& v, FormatContext& ctx)
	{
		return fmt::format_to(ctx.out(), "(begin={}, end={}, separation_thresholds: [distance={}, speed={}, azimuth={}])",
		                      v.begin_distance, v.end_distance, v.distance_separation_threshold,
		                      v.radial_speed_separation_threshold, v.azimuth_separation_threshold);
	}
};

template<>
struct fmt::formatter<rgl_bone_weights_t>
{
	template<typename ParseContext>
	constexpr auto parse(ParseContext& ctx)
	{
		return ctx.begin();
	}

	template<typename FormatContext>
	auto format(const rgl_bone_weights_t& v, FormatContext& ctx)
	{
		return fmt::format_to(ctx.out(), "(b=({},{},{},{}), w=({},{},{},{})", v.bone_indexes[0], v.bone_indexes[1],
		                      v.bone_indexes[2], v.bone_indexes[3], v.weights[0], v.weights[1], v.weights[2], v.weights[3]);
	}
};
