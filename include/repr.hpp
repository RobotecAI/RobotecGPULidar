#pragma once

#include <rgl/api/experimental.h>
#include <spdlog/fmt/fmt.h>

template<>
struct fmt::formatter<rgl_vec3f>
{
	template<typename ParseContext>
	constexpr auto parse(ParseContext& ctx) { return ctx.begin(); }

	template<typename FormatContext>
	auto format(const rgl_vec3f& v, FormatContext& ctx) {
		return fmt::format_to(ctx.out(), "({}, {}, {})", v.value[0], v.value[1], v.value[2]);
	}
};

template<>
struct fmt::formatter<rgl_vec3i>
{
	template<typename ParseContext>
	constexpr auto parse(ParseContext& ctx) { return ctx.begin(); }

	template<typename FormatContext>
	auto format(const rgl_vec3i& v, FormatContext& ctx) {
		return fmt::format_to(ctx.out(), "({}, {}, {})", v.value[0], v.value[1], v.value[2]);
	}
};

template<>
struct fmt::formatter<rgl_mat3x4f>
{
	template<typename ParseContext>
	constexpr auto parse(ParseContext& ctx) { return ctx.begin(); }

	template<typename FormatContext>
	auto format(const rgl_mat3x4f& m, FormatContext& ctx) {
		// TODO: add scale and rotation, if you are bored enough
		return fmt::format_to(ctx.out(), "Mat3x4{{T={{{}, {}, {}}}}}", m.value[0][3], m.value[1][3], m.value[2][3]);
	}
};

template<typename ArrayT>
std::string repr(ArrayT* elements, long long elemCount=1, int elemLimit=3)
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

static inline std::string repr(rgl_node_t node)
{
	return fmt::format("{}", (void*) node);
}

static inline std::string repr(rgl_node_t* node)
{
	std::string nodePointee = (node != nullptr) ? fmt::format("->{}", repr(*node)) : "";
	return fmt::format("{}{}", (void*) node, nodePointee);
}

