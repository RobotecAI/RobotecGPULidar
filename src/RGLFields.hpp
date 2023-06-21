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

#include <set>
#include <cfloat>

#include <rgl/api/core.h>
#include <VArray.hpp>

/*
 * Definition of RGL PointCloud Fields is in the API header.
 * This file defines some internal properties, such as byte sizes and C++ types.
 */

#define NON_HIT_VALUE FLT_MAX

// Shorter versions to avoid long type names
#define XYZ_F32 RGL_FIELD_XYZ_F32
#define IS_HIT_I32 RGL_FIELD_IS_HIT_I32
#define RAY_IDX_U32 RGL_FIELD_RAY_IDX_U32
#define ENTITY_ID_I32 RGL_FIELD_ENTITY_ID_I32
#define INTENSITY_F32 RGL_FIELD_INTENSITY_F32
#define RING_ID_U16 RGL_FIELD_RING_ID_U16
#define AZIMUTH_F32 RGL_FIELD_AZIMUTH_F32
#define DISTANCE_F32 RGL_FIELD_DISTANCE_F32
#define RETURN_TYPE_U8 RGL_FIELD_RETURN_TYPE_U8
#define TIME_STAMP_F64 RGL_FIELD_TIME_STAMP_F64
#define PADDING_8 RGL_FIELD_PADDING_8
#define PADDING_16 RGL_FIELD_PADDING_16
#define PADDING_32 RGL_FIELD_PADDING_32

template<rgl_field_t>
struct Field {};

#define FIELD(NAME, TYPE)                             \
template<>                                            \
struct Field<NAME>                                    \
{                                                     \
	using type = TYPE;                                \
	static constexpr std::size_t size = sizeof(TYPE); \
}

FIELD(XYZ_F32, Vec3f);
FIELD(RAY_IDX_U32, uint32_t);  // PCL uses uint32_t
FIELD(ENTITY_ID_I32, int32_t);
FIELD(INTENSITY_F32, float);
FIELD(IS_HIT_I32, int32_t);  // Signed may be faster
FIELD(DISTANCE_F32, float);
FIELD(AZIMUTH_F32, float);
FIELD(RING_ID_U16, uint16_t);
FIELD(RETURN_TYPE_U8, uint8_t);
FIELD(TIME_STAMP_F64, double);
FIELD(PADDING_8, uint8_t);
FIELD(PADDING_16, uint16_t);
FIELD(PADDING_32, uint32_t);

inline std::size_t getFieldSize(rgl_field_t type)
{
	switch (type) {
		case XYZ_F32: return Field<XYZ_F32>::size;
		case RAY_IDX_U32: return Field<RAY_IDX_U32>::size;
		case ENTITY_ID_I32: return Field<ENTITY_ID_I32>::size;
		case IS_HIT_I32: return Field<IS_HIT_I32>::size;
		case INTENSITY_F32: return Field<INTENSITY_F32>::size;
		case RING_ID_U16: return Field<RING_ID_U16>::size;
		case AZIMUTH_F32: return Field<AZIMUTH_F32>::size;
		case DISTANCE_F32: return Field<DISTANCE_F32>::size;
		case RETURN_TYPE_U8: return Field<RETURN_TYPE_U8>::size;
		case TIME_STAMP_F64: return Field<TIME_STAMP_F64>::size;
		case PADDING_8: return Field<PADDING_8>::size;
		case PADDING_16: return Field<PADDING_16>::size;
		case PADDING_32: return Field<PADDING_32>::size;
	}
	throw std::invalid_argument(fmt::format("getFieldSize: unknown RGL field {}", type));
}

inline std::size_t getPointSize(const std::vector<rgl_field_t>& fields)
{
	std::size_t size = 0;
	for (auto&& field : fields) {
		size += getFieldSize(field);
	}
	return size;
}

inline bool isDummy(rgl_field_t type)
{
	static std::set<rgl_field_t> dummies = {
		PADDING_8,
		PADDING_16,
		PADDING_32,
	};
	return dummies.find(type) != dummies.end();
}

inline VArray::Ptr createVArray(rgl_field_t type, std::size_t initialSize)
{
	if (isDummy(type)) {
		throw std::invalid_argument(fmt::format("Cannot create VArray for non-instantiable type {}", type));
	}
	switch (type) {
		case XYZ_F32: return VArray::create<Field<XYZ_F32>::type>(initialSize);
		case RAY_IDX_U32: return VArray::create<Field<RAY_IDX_U32>::type>(initialSize);
		case ENTITY_ID_I32: return VArray::create<Field<ENTITY_ID_I32>::type>(initialSize);
		case INTENSITY_F32: return VArray::create<Field<INTENSITY_F32>::type>(initialSize);
		case RING_ID_U16: return VArray::create<Field<RING_ID_U16>::type>(initialSize);
		case AZIMUTH_F32: return VArray::create<Field<AZIMUTH_F32>::type>(initialSize);
		case DISTANCE_F32: return VArray::create<Field<DISTANCE_F32>::type>(initialSize);
		case RETURN_TYPE_U8: return VArray::create<Field<RETURN_TYPE_U8>::type>(initialSize);
		case TIME_STAMP_F64: return VArray::create<Field<TIME_STAMP_F64>::type>(initialSize);
		case IS_HIT_I32: return VArray::create<Field<IS_HIT_I32>::type>(initialSize);
	}
	throw std::invalid_argument(fmt::format("createVArray: unknown RGL field {}", type));
}

inline std::string toString(rgl_field_t type)
{
	switch (type) {
		case XYZ_F32: return "XYZ_F32";
		case IS_HIT_I32: return "IS_HIT_I32";
		case RAY_IDX_U32: return "RAY_IDX_U32";
		case ENTITY_ID_I32: return "ENTITY_ID_I32";
		case INTENSITY_F32: return "INTENSITY_F32";
		case RING_ID_U16: return "RING_ID_U16";
		case AZIMUTH_F32: return "AZIMUTH_F32";
		case DISTANCE_F32: return "DISTANCE_F32";
		case RETURN_TYPE_U8: return "RETURN_TYPE_U8";
		case TIME_STAMP_F64: return "TIME_STAMP_F64";
		case PADDING_8: return "PADDING_8";
		case PADDING_16: return "PADDING_16";
		case PADDING_32: return "PADDING_32";
		default: return fmt::format("<unknown field {}>", static_cast<int>(type));
	}
}

#if RGL_BUILD_ROS2_EXTENSION
#include <sensor_msgs/msg/point_cloud2.hpp>

inline std::vector<uint8_t> toRos2Fields(rgl_field_t type)
{
	switch (type) {
		case XYZ_F32: return { sensor_msgs::msg::PointField::FLOAT32,
		                       sensor_msgs::msg::PointField::FLOAT32,
		                       sensor_msgs::msg::PointField::FLOAT32 };
		case IS_HIT_I32: return { sensor_msgs::msg::PointField::INT32 };
		case RAY_IDX_U32: return { sensor_msgs::msg::PointField::UINT32 };
		case ENTITY_ID_I32: return { sensor_msgs::msg::PointField::INT32 };
		case INTENSITY_F32: return { sensor_msgs::msg::PointField::FLOAT32 };
		case RING_ID_U16: return { sensor_msgs::msg::PointField::UINT16 };
		case AZIMUTH_F32: return { sensor_msgs::msg::PointField::FLOAT32 };
		case DISTANCE_F32: return { sensor_msgs::msg::PointField::FLOAT32 };
		case RETURN_TYPE_U8: return { sensor_msgs::msg::PointField::UINT8 };
		case TIME_STAMP_F64: return { sensor_msgs::msg::PointField::FLOAT64 };
		case PADDING_8: return { };
		case PADDING_16: return { };
		case PADDING_32: return { };
	}
	throw std::invalid_argument(fmt::format("toRos2Fields: unknown RGL field {}", type));
}

inline std::vector<std::string> toRos2Names(rgl_field_t type)
{
	switch (type) {
		case XYZ_F32: return { "x", "y", "z" };
		case IS_HIT_I32: return { "is_hit" };
		case ENTITY_ID_I32: return { "entity_id" };
		case RAY_IDX_U32: return { "ray_idx" };
		case INTENSITY_F32: return { "intensity" };
		case RING_ID_U16: return { "ring" };
		case AZIMUTH_F32: return { "azimuth" };
		case DISTANCE_F32: return { "distance" };
		case RETURN_TYPE_U8: return { "return_type" };
		case TIME_STAMP_F64: return { "time_stamp" };
		case PADDING_8: return { };
		case PADDING_16: return { };
		case PADDING_32: return { };
	}
	throw std::invalid_argument(fmt::format("toRos2Names: unknown RGL field {}", type));
}

inline std::vector<std::size_t> toRos2Sizes(rgl_field_t type)
{
	switch (type) {
		case XYZ_F32: return { sizeof(float), sizeof(float), sizeof(float) };
		case IS_HIT_I32: return { getFieldSize(type) };
		case RAY_IDX_U32: return { getFieldSize(type) };
		case ENTITY_ID_I32: return { getFieldSize(type) };
		case INTENSITY_F32: return { getFieldSize(type) };
		case RING_ID_U16: return { getFieldSize(type) };
		case AZIMUTH_F32: return { getFieldSize(type) };
		case DISTANCE_F32: return { getFieldSize(type) };
		case RETURN_TYPE_U8: return { getFieldSize(type) };
		case TIME_STAMP_F64: return { getFieldSize(type) };
		case PADDING_8: return { getFieldSize(type) };
		case PADDING_16: return { getFieldSize(type) };
		case PADDING_32: return { getFieldSize(type) };
	}
	throw std::invalid_argument(fmt::format("toRos2Sizes: unknown RGL field {}", type));
}
#endif
