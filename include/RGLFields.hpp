#pragma once

#include <set>

#include <rgl/api/experimental.h>
#include <VArray.hpp>

/*
 * Definition of RGL PointCloud Fields is in the API header.
 * This file defines some internal properties, such as byte sizes and C++ types.
 */

template<rgl_field_t>
struct RGLField {};

#define RGL_FIELD(NAME, TYPE)                         \
template<>                                            \
struct RGLField<RGL_FIELD_##NAME>                     \
{                                                     \
	using Type = TYPE;                                \
	static constexpr std::size_t size = sizeof(TYPE); \
}

RGL_FIELD(XYZ_F32, Vec3f);
RGL_FIELD(INTENSITY_F32, float);
RGL_FIELD(RING_ID_U16, uint16_t);
RGL_FIELD(AZIMUTH_F32, float);
RGL_FIELD(DISTANCE_F32, float);
RGL_FIELD(RETURN_TYPE_U8, uint8_t);
RGL_FIELD(TIME_STAMP_F64, double);
RGL_FIELD(PADDING_8, uint8_t);
RGL_FIELD(PADDING_16, uint16_t);
RGL_FIELD(PADDING_32, uint32_t);

inline std::size_t getFieldSize(rgl_field_t type)
{
	switch (type) {
		case RGL_FIELD_XYZ_F32: return RGLField<RGL_FIELD_XYZ_F32>::size;
		case RGL_FIELD_INTENSITY_F32: return RGLField<RGL_FIELD_INTENSITY_F32>::size;
		case RGL_FIELD_RING_ID_U16: return RGLField<RGL_FIELD_RING_ID_U16>::size;
		case RGL_FIELD_AZIMUTH_F32: return RGLField<RGL_FIELD_AZIMUTH_F32>::size;
		case RGL_FIELD_DISTANCE_F32: return RGLField<RGL_FIELD_DISTANCE_F32>::size;
		case RGL_FIELD_RETURN_TYPE_U8: return RGLField<RGL_FIELD_RETURN_TYPE_U8>::size;
		case RGL_FIELD_TIME_STAMP_F64: return RGLField<RGL_FIELD_TIME_STAMP_F64>::size;
		case RGL_FIELD_PADDING_8: return RGLField<RGL_FIELD_PADDING_8>::size;
		case RGL_FIELD_PADDING_16: return RGLField<RGL_FIELD_PADDING_16>::size;
		case RGL_FIELD_PADDING_32: return RGLField<RGL_FIELD_PADDING_32>::size;
	}
	throw std::invalid_argument(fmt::format("getFieldSize: unknown RGL field {}", type));
}

inline bool isDummy(rgl_field_t type)
{
	static std::set<rgl_field_t> dummies = {
		RGL_FIELD_PADDING_8,
		RGL_FIELD_PADDING_16,
		RGL_FIELD_PADDING_32,
	};
	return dummies.contains(type);
}

inline std::shared_ptr<VArray> createVArray(rgl_field_t type, std::size_t initialSize)
{
	if (isDummy(type)) {
		throw std::invalid_argument(fmt::format("Cannot create VArray for non-instantiable type {}", type));
	}
	switch (type) {
		case RGL_FIELD_XYZ_F32:
			return VArray::create<RGLField<RGL_FIELD_XYZ_F32>::Type>(initialSize);
		case RGL_FIELD_INTENSITY_F32:
			return VArray::create<RGLField<RGL_FIELD_INTENSITY_F32>::Type>(initialSize);
		case RGL_FIELD_RING_ID_U16:
			return VArray::create<RGLField<RGL_FIELD_RING_ID_U16>::Type>(initialSize);
		case RGL_FIELD_AZIMUTH_F32:
			return VArray::create<RGLField<RGL_FIELD_AZIMUTH_F32>::Type>(initialSize);
		case RGL_FIELD_DISTANCE_F32:
			return VArray::create<RGLField<RGL_FIELD_DISTANCE_F32>::Type>(initialSize);
		case RGL_FIELD_RETURN_TYPE_U8:
			return VArray::create<RGLField<RGL_FIELD_RETURN_TYPE_U8>::Type>(initialSize);
		case RGL_FIELD_TIME_STAMP_F64:
			return VArray::create<RGLField<RGL_FIELD_TIME_STAMP_F64>::Type>(initialSize);
	}
	throw std::invalid_argument(fmt::format("createVArray: unknown RGL field {}", type));
}
