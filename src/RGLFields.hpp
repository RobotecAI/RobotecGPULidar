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
#include <vector>

#include <math/Vector.hpp>
#include <math/Mat3x4f.hpp>
#include <rgl/api/core.h>

/*
 * Definition of RGL PointCloud Fields is in the API header.
 * This file defines some internal properties, such as byte sizes and C++ types.
 *
 * Important: If you change available fields, you must also update:
 * - test/include/helpers/fieldGenerators.hpp: add/delete fieldGenerator
 * - test/include/helpers/testPointCloud.hpp: add/delete fieldGenerator inside fieldGenerators
 */

typedef unsigned char TextureTexelFormat;

// Shorter versions to avoid long type names
#define XYZ_VEC3_F32 RGL_FIELD_XYZ_VEC3_F32
#define IS_HIT_I32 RGL_FIELD_IS_HIT_I32
#define IS_GROUND_I32 RGL_FIELD_IS_GROUND_I32
#define RAY_IDX_U32 RGL_FIELD_RAY_IDX_U32
#define ENTITY_ID_I32 RGL_FIELD_ENTITY_ID_I32
#define INTENSITY_F32 RGL_FIELD_INTENSITY_F32
#define INTENSITY_U8 RGL_FIELD_INTENSITY_U8
#define LASER_RETRO_F32 RGL_FIELD_LASER_RETRO_F32
#define RING_ID_U16 RGL_FIELD_RING_ID_U16
#define AZIMUTH_F32 RGL_FIELD_AZIMUTH_F32
#define ELEVATION_F32 RGL_FIELD_ELEVATION_F32
#define DISTANCE_F32 RGL_FIELD_DISTANCE_F32
#define RETURN_TYPE_U8 RGL_FIELD_RETURN_TYPE_U8
#define TIME_STAMP_F64 RGL_FIELD_TIME_STAMP_F64
#define TIME_STAMP_U32 RGL_FIELD_TIME_STAMP_U32
#define ABSOLUTE_VELOCITY_VEC3_F32 RGL_FIELD_ABSOLUTE_VELOCITY_VEC3_F32
#define RELATIVE_VELOCITY_VEC3_F32 RGL_FIELD_RELATIVE_VELOCITY_VEC3_F32
#define RADIAL_SPEED_F32 RGL_FIELD_RADIAL_SPEED_F32
#define POWER_F32 RGL_FIELD_POWER_F32
#define RCS_F32 RGL_FIELD_RCS_F32
#define NOISE_F32 RGL_FIELD_NOISE_F32
#define SNR_F32 RGL_FIELD_SNR_F32
#define NORMAL_VEC3_F32 RGL_FIELD_NORMAL_VEC3_F32
#define INCIDENT_ANGLE_F32 RGL_FIELD_INCIDENT_ANGLE_F32
#define RAY_POSE_MAT3x4_F32 RGL_FIELD_RAY_POSE_MAT3x4_F32
#define PADDING_8 RGL_FIELD_PADDING_8
#define PADDING_16 RGL_FIELD_PADDING_16
#define PADDING_32 RGL_FIELD_PADDING_32

inline const std::set<rgl_field_t>& getAllRealFields()
{
	static std::set<rgl_field_t> allRealFields = {
	    XYZ_VEC3_F32,
	    IS_HIT_I32,
	    IS_GROUND_I32,
	    RAY_IDX_U32,
	    ENTITY_ID_I32,
	    INTENSITY_F32,
	    INTENSITY_U8,
	    LASER_RETRO_F32,
	    RING_ID_U16,
	    AZIMUTH_F32,
	    ELEVATION_F32,
	    DISTANCE_F32,
	    RETURN_TYPE_U8,
	    TIME_STAMP_F64,
	    TIME_STAMP_U32,
	    ABSOLUTE_VELOCITY_VEC3_F32,
	    RELATIVE_VELOCITY_VEC3_F32,
	    RADIAL_SPEED_F32,
	    POWER_F32,
	    RCS_F32,
	    NOISE_F32,
	    SNR_F32,
	    NORMAL_VEC3_F32,
	    INCIDENT_ANGLE_F32,
	    RAY_POSE_MAT3x4_F32,
	};
	return allRealFields;
}

inline const std::set<rgl_field_t>& getAllPaddings()
{
	static std::set<rgl_field_t> allPaddings = {
	    PADDING_8,
	    PADDING_16,
	    PADDING_32,
	};
	return allPaddings;
}

template<rgl_field_t>
struct Field
{};

#define FIELD(NAME, TYPE)                                                                                                      \
	template<>                                                                                                                 \
	struct Field<NAME>                                                                                                         \
	{                                                                                                                          \
		using type = TYPE;                                                                                                     \
		static constexpr std::size_t size = sizeof(TYPE);                                                                      \
	}

FIELD(XYZ_VEC3_F32, Vec3f);
FIELD(RAY_IDX_U32, uint32_t); // PCL uses uint32_t
FIELD(ENTITY_ID_I32, int32_t);
FIELD(INTENSITY_F32, float);
FIELD(INTENSITY_U8, uint8_t);
FIELD(LASER_RETRO_F32, float);
FIELD(IS_HIT_I32, int32_t);    // Signed may be faster
FIELD(IS_GROUND_I32, int32_t); // Signed may be faster
FIELD(DISTANCE_F32, float);
FIELD(AZIMUTH_F32, float);
FIELD(ELEVATION_F32, float);
FIELD(RING_ID_U16, uint16_t);
FIELD(RETURN_TYPE_U8, uint8_t);
FIELD(TIME_STAMP_F64, double);
FIELD(TIME_STAMP_U32, uint32_t);
FIELD(PADDING_8, uint8_t);
FIELD(PADDING_16, uint16_t);
FIELD(PADDING_32, uint32_t);
FIELD(ABSOLUTE_VELOCITY_VEC3_F32, Vec3f);
FIELD(RELATIVE_VELOCITY_VEC3_F32, Vec3f);
FIELD(RADIAL_SPEED_F32, float);
FIELD(POWER_F32, float);
FIELD(RCS_F32, float);
FIELD(NOISE_F32, float);
FIELD(SNR_F32, float);
FIELD(NORMAL_VEC3_F32, Vec3f);
FIELD(INCIDENT_ANGLE_F32, float);
FIELD(RAY_POSE_MAT3x4_F32, Mat3x4f);

inline std::size_t getFieldSize(rgl_field_t type)
{
	switch (type) {
		case XYZ_VEC3_F32: return Field<XYZ_VEC3_F32>::size;
		case RAY_IDX_U32: return Field<RAY_IDX_U32>::size;
		case ENTITY_ID_I32: return Field<ENTITY_ID_I32>::size;
		case IS_HIT_I32: return Field<IS_HIT_I32>::size;
		case IS_GROUND_I32: return Field<IS_GROUND_I32>::size;
		case INTENSITY_F32: return Field<INTENSITY_F32>::size;
		case INTENSITY_U8: return Field<INTENSITY_U8>::size;
		case LASER_RETRO_F32: return Field<LASER_RETRO_F32>::size;
		case RING_ID_U16: return Field<RING_ID_U16>::size;
		case AZIMUTH_F32: return Field<AZIMUTH_F32>::size;
		case ELEVATION_F32: return Field<ELEVATION_F32>::size;
		case DISTANCE_F32: return Field<DISTANCE_F32>::size;
		case RETURN_TYPE_U8: return Field<RETURN_TYPE_U8>::size;
		case TIME_STAMP_F64: return Field<TIME_STAMP_F64>::size;
		case TIME_STAMP_U32: return Field<TIME_STAMP_U32>::size;
		case ABSOLUTE_VELOCITY_VEC3_F32: return Field<ABSOLUTE_VELOCITY_VEC3_F32>::size;
		case RELATIVE_VELOCITY_VEC3_F32: return Field<RELATIVE_VELOCITY_VEC3_F32>::size;
		case RADIAL_SPEED_F32: return Field<RADIAL_SPEED_F32>::size;
		case POWER_F32: return Field<POWER_F32>::size;
		case RCS_F32: return Field<RCS_F32>::size;
		case NOISE_F32: return Field<NOISE_F32>::size;
		case SNR_F32: return Field<SNR_F32>::size;
		case NORMAL_VEC3_F32: return Field<NORMAL_VEC3_F32>::size;
		case INCIDENT_ANGLE_F32: return Field<INCIDENT_ANGLE_F32>::size;
		case RAY_POSE_MAT3x4_F32: return Field<RAY_POSE_MAT3x4_F32>::size;
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

// This header is included by nvcc (GPU-side compiler) which fails to compile IAnyArray and subclasses.
// Therefore, we use forward declaration and shared_ptr<IAnyArray> instead of IAnyArray::Ptr
struct IAnyArray;
template<template<typename> typename Subclass, typename... Args>
inline std::shared_ptr<IAnyArray> createArray(rgl_field_t type, Args&&... args)
{
	if (isDummy(type)) {
		throw std::invalid_argument(fmt::format("Cannot create Array<T> for non-instantiable type {}", type));
	}
	switch (type) {
		case XYZ_VEC3_F32: return Subclass<Field<XYZ_VEC3_F32>::type>::create(std::forward<Args>(args)...);
		case RAY_IDX_U32: return Subclass<Field<RAY_IDX_U32>::type>::create(std::forward<Args>(args)...);
		case ENTITY_ID_I32: return Subclass<Field<ENTITY_ID_I32>::type>::create(std::forward<Args>(args)...);
		case INTENSITY_F32: return Subclass<Field<INTENSITY_F32>::type>::create(std::forward<Args>(args)...);
		case INTENSITY_U8: return Subclass<Field<INTENSITY_U8>::type>::create(std::forward<Args>(args)...);
		case LASER_RETRO_F32: return Subclass<Field<LASER_RETRO_F32>::type>::create(std::forward<Args>(args)...);
		case RING_ID_U16: return Subclass<Field<RING_ID_U16>::type>::create(std::forward<Args>(args)...);
		case AZIMUTH_F32: return Subclass<Field<AZIMUTH_F32>::type>::create(std::forward<Args>(args)...);
		case ELEVATION_F32: return Subclass<Field<ELEVATION_F32>::type>::create(std::forward<Args>(args)...);
		case DISTANCE_F32: return Subclass<Field<DISTANCE_F32>::type>::create(std::forward<Args>(args)...);
		case RETURN_TYPE_U8: return Subclass<Field<RETURN_TYPE_U8>::type>::create(std::forward<Args>(args)...);
		case TIME_STAMP_F64: return Subclass<Field<TIME_STAMP_F64>::type>::create(std::forward<Args>(args)...);
		case TIME_STAMP_U32: return Subclass<Field<TIME_STAMP_U32>::type>::create(std::forward<Args>(args)...);
		case IS_HIT_I32: return Subclass<Field<IS_HIT_I32>::type>::create(std::forward<Args>(args)...);
		case IS_GROUND_I32: return Subclass<Field<IS_GROUND_I32>::type>::create(std::forward<Args>(args)...);
		case ABSOLUTE_VELOCITY_VEC3_F32:
			return Subclass<Field<ABSOLUTE_VELOCITY_VEC3_F32>::type>::create(std::forward<Args>(args)...);
		case RELATIVE_VELOCITY_VEC3_F32:
			return Subclass<Field<RELATIVE_VELOCITY_VEC3_F32>::type>::create(std::forward<Args>(args)...);
		case RADIAL_SPEED_F32: return Subclass<Field<RADIAL_SPEED_F32>::type>::create(std::forward<Args>(args)...);
		case POWER_F32: return Subclass<Field<POWER_F32>::type>::create(std::forward<Args>(args)...);
		case RCS_F32: return Subclass<Field<RCS_F32>::type>::create(std::forward<Args>(args)...);
		case NOISE_F32: return Subclass<Field<NOISE_F32>::type>::create(std::forward<Args>(args)...);
		case SNR_F32: return Subclass<Field<SNR_F32>::type>::create(std::forward<Args>(args)...);
		case NORMAL_VEC3_F32: return Subclass<Field<NORMAL_VEC3_F32>::type>::create(std::forward<Args>(args)...);
		case INCIDENT_ANGLE_F32: return Subclass<Field<INCIDENT_ANGLE_F32>::type>::create(std::forward<Args>(args)...);
		case RAY_POSE_MAT3x4_F32: return Subclass<Field<RAY_POSE_MAT3x4_F32>::type>::create(std::forward<Args>(args)...);
	}
	throw std::invalid_argument(fmt::format("createArray: unknown RGL field {}", type));
}

inline std::string toString(rgl_field_t type)
{
	switch (type) {
		case XYZ_VEC3_F32: return "XYZ_VEC3_F32";
		case IS_HIT_I32: return "IS_HIT_I32";
		case IS_GROUND_I32: return "IS_GROUND_I32";
		case RAY_IDX_U32: return "RAY_IDX_U32";
		case ENTITY_ID_I32: return "ENTITY_ID_I32";
		case INTENSITY_F32: return "INTENSITY_F32";
		case INTENSITY_U8: return "INTENSITY_U8";
		case LASER_RETRO_F32: return "LASER_RETRO_F32";
		case RING_ID_U16: return "RING_ID_U16";
		case AZIMUTH_F32: return "AZIMUTH_F32";
		case ELEVATION_F32: return "ELEVATION_F32";
		case DISTANCE_F32: return "DISTANCE_F32";
		case RETURN_TYPE_U8: return "RETURN_TYPE_U8";
		case TIME_STAMP_F64: return "TIME_STAMP_F64";
		case TIME_STAMP_U32: return "TIME_STAMP_U32";
		case ABSOLUTE_VELOCITY_VEC3_F32: return "ABSOLUTE_VELOCITY_VEC3_F32";
		case RELATIVE_VELOCITY_VEC3_F32: return "RELATIVE_VELOCITY_VEC3_F32";
		case RADIAL_SPEED_F32: return "RADIAL_SPEED_F32";
		case POWER_F32: return "POWER_F32";
		case RCS_F32: return "RCS_F32";
		case NOISE_F32: return "NOISE_F32";
		case SNR_F32: return "SNR_F32";
		case NORMAL_VEC3_F32: return "NORMAL_VEC3_F32";
		case INCIDENT_ANGLE_F32: return "INCIDENT_ANGLE_F32";
		case RAY_POSE_MAT3x4_F32: return "RAY_POSE_MAT3x4_F32";
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
		case XYZ_VEC3_F32:
			return {sensor_msgs::msg::PointField::FLOAT32, sensor_msgs::msg::PointField::FLOAT32,
			        sensor_msgs::msg::PointField::FLOAT32};
		case IS_HIT_I32: return {sensor_msgs::msg::PointField::INT32};
		case IS_GROUND_I32: return {sensor_msgs::msg::PointField::INT32};
		case RAY_IDX_U32: return {sensor_msgs::msg::PointField::UINT32};
		case ENTITY_ID_I32: return {sensor_msgs::msg::PointField::INT32};
		case INTENSITY_F32: return {sensor_msgs::msg::PointField::FLOAT32};
		case INTENSITY_U8: return {sensor_msgs::msg::PointField::UINT8};
		case LASER_RETRO_F32: return {sensor_msgs::msg::PointField::FLOAT32};
		case RING_ID_U16: return {sensor_msgs::msg::PointField::UINT16};
		case AZIMUTH_F32: return {sensor_msgs::msg::PointField::FLOAT32};
		case ELEVATION_F32: return {sensor_msgs::msg::PointField::FLOAT32};
		case DISTANCE_F32: return {sensor_msgs::msg::PointField::FLOAT32};
		case RETURN_TYPE_U8: return {sensor_msgs::msg::PointField::UINT8};
		case TIME_STAMP_F64: return {sensor_msgs::msg::PointField::FLOAT64};
		case TIME_STAMP_U32: return {sensor_msgs::msg::PointField::UINT32};
		case ABSOLUTE_VELOCITY_VEC3_F32:
			return {sensor_msgs::msg::PointField::FLOAT32, sensor_msgs::msg::PointField::FLOAT32,
			        sensor_msgs::msg::PointField::FLOAT32};
		case RELATIVE_VELOCITY_VEC3_F32:
			return {sensor_msgs::msg::PointField::FLOAT32, sensor_msgs::msg::PointField::FLOAT32,
			        sensor_msgs::msg::PointField::FLOAT32};
		case RADIAL_SPEED_F32: return {sensor_msgs::msg::PointField::FLOAT32};
		case POWER_F32: return {sensor_msgs::msg::PointField::FLOAT32};
		case RCS_F32: return {sensor_msgs::msg::PointField::FLOAT32};
		case NOISE_F32: return {sensor_msgs::msg::PointField::FLOAT32};
		case SNR_F32: return {sensor_msgs::msg::PointField::FLOAT32};
		case NORMAL_VEC3_F32:
			return {sensor_msgs::msg::PointField::FLOAT32, sensor_msgs::msg::PointField::FLOAT32,
			        sensor_msgs::msg::PointField::FLOAT32};
		case INCIDENT_ANGLE_F32: return {sensor_msgs::msg::PointField::FLOAT32};
		case RAY_POSE_MAT3x4_F32:
			return {
			    sensor_msgs::msg::PointField::FLOAT32, sensor_msgs::msg::PointField::FLOAT32,
			    sensor_msgs::msg::PointField::FLOAT32, sensor_msgs::msg::PointField::FLOAT32,
			    sensor_msgs::msg::PointField::FLOAT32, sensor_msgs::msg::PointField::FLOAT32,
			    sensor_msgs::msg::PointField::FLOAT32, sensor_msgs::msg::PointField::FLOAT32,
			    sensor_msgs::msg::PointField::FLOAT32, sensor_msgs::msg::PointField::FLOAT32,
			    sensor_msgs::msg::PointField::FLOAT32, sensor_msgs::msg::PointField::FLOAT32,
			};
		case PADDING_8: return {};
		case PADDING_16: return {};
		case PADDING_32: return {};
	}
	throw std::invalid_argument(fmt::format("toRos2Fields: unknown RGL field {}", type));
}

inline std::vector<std::string> toRos2Names(rgl_field_t type)
{
	// clang-format off
	switch (type) {
		case XYZ_VEC3_F32: return {"x", "y", "z"};
		case IS_HIT_I32: return {"is_hit"};
		case IS_GROUND_I32: return {"is_ground"};
		case ENTITY_ID_I32: return {"entity_id"};
		case RAY_IDX_U32: return {"ray_idx"};
		case INTENSITY_F32:
		case INTENSITY_U8:
			return {"intensity"};
		case LASER_RETRO_F32: return {"laser_retro"};
		case RING_ID_U16: return {"channel"};
		case AZIMUTH_F32: return {"azimuth"};
		case ELEVATION_F32: return {"elevation"};
		case DISTANCE_F32: return {"distance"};
		case RETURN_TYPE_U8: return {"return_type"};
		case TIME_STAMP_F64:
		case TIME_STAMP_U32:
			return {"time_stamp"};
		case ABSOLUTE_VELOCITY_VEC3_F32: return {"abs_vx", "abs_vy", "abs_vz"};
		case RELATIVE_VELOCITY_VEC3_F32: return {"rel_vx", "rel_vy", "rel_vz"};
		case RADIAL_SPEED_F32: return {"radial_speed"};
		case POWER_F32: return {"power"};
		case RCS_F32: return {"rcs"};
		case NOISE_F32: return {"noise"};
		case SNR_F32: return {"snr"};
		case NORMAL_VEC3_F32: return {"nx", "ny", "nz"};
		case INCIDENT_ANGLE_F32: return {"incident_angle"};
		case RAY_POSE_MAT3x4_F32: return {"m00", "m01", "m02", "m03", "m10", "m11", "m12", "m13", "m20", "m21", "m22", "m23"};
		case PADDING_8: return {};
		case PADDING_16: return {};
		case PADDING_32: return {};
	}
	// clang-format on
	throw std::invalid_argument(fmt::format("toRos2Names: unknown RGL field {}", type));
}

inline std::vector<std::size_t> toRos2Sizes(rgl_field_t type)
{
	switch (type) {
		// Add here all vector fields
		case XYZ_VEC3_F32: return {sizeof(float), sizeof(float), sizeof(float)};
		case ABSOLUTE_VELOCITY_VEC3_F32: return {sizeof(float), sizeof(float), sizeof(float)};
		case RELATIVE_VELOCITY_VEC3_F32: return {sizeof(float), sizeof(float), sizeof(float)};
		case NORMAL_VEC3_F32: return {sizeof(float), sizeof(float), sizeof(float)};
		case RAY_POSE_MAT3x4_F32:
			return {
			    sizeof(float), sizeof(float), sizeof(float), sizeof(float), sizeof(float), sizeof(float),
			    sizeof(float), sizeof(float), sizeof(float), sizeof(float), sizeof(float), sizeof(float),
			};
		default: return {getFieldSize(type)};
	}
}
#endif
