#include <stdexcept>
#include <rgl/api/core.h>
#include <spdlog/fmt/bundled/format.h>

template<rgl_texture_format_t>
struct TextureFormat {};

#define FORMAT(NAME, TYPE)                             \
template<>                                             \
struct TextureFormat<NAME>                                    \
{                                                      \
	using type = TYPE;                                 \
	static constexpr std::size_t size = sizeof(TYPE);  \
}

FORMAT(RGL_TEXTURE_TYPE_INT, int);
FORMAT(RGL_TEXTURE_TYPE_FLOAT, float);

inline std::size_t getFormatSize(rgl_texture_format_t type)
{
	switch (type) {
		case RGL_TEXTURE_TYPE_INT: return TextureFormat<RGL_TEXTURE_TYPE_INT>::size;
		case RGL_TEXTURE_TYPE_FLOAT: return TextureFormat<RGL_TEXTURE_TYPE_FLOAT>::size;
	}
	throw std::invalid_argument(fmt::format("getFormatSize: unknown RGL texture format {}", type));
}


inline std::string toString(rgl_texture_format_t type)
{
	switch (type) {
		case RGL_TEXTURE_TYPE_INT: return "RGL_TEXTURE_TYPE_INT";
		case RGL_TEXTURE_TYPE_FLOAT: return "RGL_TEXTURE_TYPE_FLOAT";

		default: return fmt::format("<unknown format {}>", static_cast<int>(type));
	}
}