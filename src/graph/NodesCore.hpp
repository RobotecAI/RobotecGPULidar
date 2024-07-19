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

#include <vector>
#include <set>
#include <memory>
#include <thread>
#include <typeinfo>
#include <ranges>
#include <algorithm>
#include <random>
#include <queue>
#include <curand_kernel.h>

#include <graph/Node.hpp>
#include <graph/Interfaces.hpp>
#include <gpu/RaytraceRequestContext.hpp>
#include <gpu/nodeKernels.hpp>
#include <CacheManager.hpp>
#include <GPUFieldDescBuilder.hpp>
#include <math/Aabb.h>
#include <math/RunningStats.hpp>
#include <gpu/MultiReturn.hpp>
#include <Time.hpp>


struct FormatPointsNode : IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<FormatPointsNode>;
	void setParameters(const std::vector<rgl_field_t>& fields);

	// Node
	void enqueueExecImpl() override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override { return fields; }

	// Point cloud description
	bool hasField(rgl_field_t field) const override
	{
		return field == RGL_FIELD_DYNAMIC_FORMAT || std::find(fields.begin(), fields.end(), field) != fields.end();
	}

	// Data getters
	IAnyArray::ConstPtr getFieldData(rgl_field_t field) override;
	std::size_t getFieldPointSize(rgl_field_t field) const override;

	// Actual implementation of formatting made public for other nodes
	static void formatAsync(DeviceAsyncArray<char>::Ptr output, const IPointsNode::Ptr& input,
	                        const std::vector<rgl_field_t>& fields, GPUFieldDescBuilder& gpuFieldDescBuilder);

	// Needed to create GPUFieldDesc for other nodes
	static std::vector<std::pair<rgl_field_t, const void*>> getFieldToPointerMappings(const IPointsNode::Ptr& input,
	                                                                                  const std::vector<rgl_field_t>& fields);

private:
	std::vector<rgl_field_t> fields;
	DeviceAsyncArray<char>::Ptr output = DeviceAsyncArray<char>::create(arrayMgr);
	HostPinnedArray<char>::Ptr outputHost = HostPinnedArray<char>::create();
	GPUFieldDescBuilder gpuFieldDescBuilder;
};

struct CompactByFieldPointsNode : IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<CompactByFieldPointsNode>;
	void setParameters(rgl_field_t field);

	// Node
	void validateImpl() override;
	void enqueueExecImpl() override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override { return {IS_HIT_I32, IS_GROUND_I32}; }

	// Point cloud description
	bool isDense() const override { return true; }
	size_t getWidth() const override;
	size_t getHeight() const override { return 1; }

	// Data getters
	IAnyArray::ConstPtr getFieldData(rgl_field_t field) override;

private:
	rgl_field_t fieldToCompactBy;
	size_t width = {0};
	DeviceAsyncArray<CompactionIndexType>::Ptr inclusivePrefixSum = DeviceAsyncArray<CompactionIndexType>::create(arrayMgr);
	CacheManager<rgl_field_t, IAnyArray::Ptr> cacheManager;
	std::mutex getFieldDataMutex;
};

struct RaytraceNode : IPointsNode
{
	using Ptr = std::shared_ptr<RaytraceNode>;
	void setParameters();

	// Node
	void validateImpl() override;
	void enqueueExecImpl() override;

	// Point cloud description
	bool isDense() const override { return false; }
	bool hasField(rgl_field_t field) const override { return fieldData.contains(field); }
	size_t getWidth() const override { return raysNode ? raysNode->getRayCount() : 0; }
	size_t getHeight() const override { return 1; } // TODO: implement height in use_rays

	Mat3x4f getLookAtOriginTransform() const override { return raysNode->getCumulativeRayTransfrom().inverse(); }

	// Data getters
	IAnyArray::ConstPtr getFieldData(rgl_field_t field) override
	{
		if (field == RAY_POSE_MAT3x4_F32) {
			return raysNode->getRays();
		}
		return std::const_pointer_cast<const IAnyArray>(fieldData.at(field));
	}

	IAnyArray::ConstPtr getFieldDataMultiReturn(rgl_field_t field, rgl_return_type_t);

	// RaytraceNode specific
	void setVelocity(const Vec3f& linearVelocity, const Vec3f& angularVelocity);
	void enableRayDistortion(bool enabled) { doApplyDistortion = enabled; }
	void setNonHitDistanceValues(float nearDistance, float farDistance);
	void setNonHitsMask(const int8_t* maskRaw, size_t maskPointCount);
	void setBeamDivergence(float hDivergenceRad, float vDivergenceRad)
	{
		hBeamHalfDivergenceRad = hDivergenceRad / 2.0f;
		vBeamHalfDivergenceRad = vDivergenceRad / 2.0f;
	}
	void setDefaultIntensity(float intensity) { defaultIntensity = intensity; }

private:
	IRaysNode::Ptr raysNode;

	DeviceAsyncArray<Vec2f>::Ptr defaultRange = DeviceAsyncArray<Vec2f>::create(arrayMgr);
	bool doApplyDistortion{false};
	Vec3f sensorLinearVelocityXYZ{0, 0, 0};
	Vec3f sensorAngularVelocityRPY{0, 0, 0};

	float nearNonHitDistance{std::numeric_limits<float>::infinity()};
	float farNonHitDistance{std::numeric_limits<float>::infinity()};
	float hBeamHalfDivergenceRad = 0.0f;
	float vBeamHalfDivergenceRad = 0.0f;
	float defaultIntensity = 0.0f;

	DeviceAsyncArray<int8_t>::Ptr rayMask;

	HostPinnedArray<RaytraceRequestContext>::Ptr requestCtxHst = HostPinnedArray<RaytraceRequestContext>::create();
	DeviceAsyncArray<RaytraceRequestContext>::Ptr requestCtxDev = DeviceAsyncArray<RaytraceRequestContext>::create(arrayMgr);

	std::unordered_map<rgl_field_t, IAnyArray::Ptr> fieldData; // All should be DeviceAsyncArray

	struct MultiReturnFields
	{
		MultiReturnFields(StreamBoundObjectsManager& arrayMgr)
		  : isHit(DeviceAsyncArray<Field<IS_HIT_I32>::type>::create(arrayMgr)),
		    xyz(DeviceAsyncArray<Field<XYZ_VEC3_F32>::type>::create(arrayMgr)),
		    distance(DeviceAsyncArray<Field<DISTANCE_F32>::type>::create(arrayMgr))
		{}
		void resize(size_t size)
		{
			isHit->resize(size, false, false);
			xyz->resize(size, false, false);
			distance->resize(size, false, false);
		}
		MultiReturnPointers getPointers()
		{
			return MultiReturnPointers{
			    .isHit = isHit->getWritePtr(),
			    .xyz = xyz->getWritePtr(),
			    .distance = distance->getWritePtr(),
			};
		}
		DeviceAsyncArray<Field<IS_HIT_I32>::type>::Ptr isHit;
		DeviceAsyncArray<Field<XYZ_VEC3_F32>::type>::Ptr xyz;
		DeviceAsyncArray<Field<DISTANCE_F32>::type>::Ptr distance;
	};


	MultiReturnFields mrSamples = MultiReturnFields{arrayMgr};
	MultiReturnFields mrFirst = MultiReturnFields{arrayMgr};
	MultiReturnFields mrLast = MultiReturnFields{arrayMgr};


	template<rgl_field_t>
	auto getPtrTo();

	std::set<rgl_field_t> findFieldsToCompute();
	void setFields(const std::set<rgl_field_t>& fields);
};

struct MultiReturnSwitchNode : IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<MultiReturnSwitchNode>;
	void setParameters(rgl_return_type_t returnType) { this->returnType = returnType; }

	// Node
	void validateImpl() override
	{
		IPointsNodeSingleInput::validateImpl();
		rtxInput = getExactlyOneInputOfType<RaytraceNode>();
	}

	void enqueueExecImpl() override {}

	// Data getters
	IAnyArray::ConstPtr getFieldData(rgl_field_t field) override
	{
		if (returnType == RGL_RETURN_TYPE_NOT_DIVERGENT) {
			return rtxInput->getFieldData(field);
		}
		return rtxInput->getFieldDataMultiReturn(field, returnType);
	}

private:
	rgl_return_type_t returnType;
	RaytraceNode::Ptr rtxInput;
};

struct TransformPointsNode : IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<TransformPointsNode>;
	void setParameters(Mat3x4f transform) { this->transform = transform; }
	Mat3x4f getTransform() const { return transform; }

	// Node
	void enqueueExecImpl() override;
	std::string getArgsString() const override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override { return {XYZ_VEC3_F32}; }

	Mat3x4f getLookAtOriginTransform() const override { return transform.inverse() * input->getLookAtOriginTransform(); }

	// Data getters
	IAnyArray::ConstPtr getFieldData(rgl_field_t field) override;

private:
	Mat3x4f transform;
	DeviceAsyncArray<Field<XYZ_VEC3_F32>::type>::Ptr output = DeviceAsyncArray<Field<XYZ_VEC3_F32>::type>::create(arrayMgr);
};

struct TransformRaysNode : IRaysNodeSingleInput
{
	using Ptr = std::shared_ptr<TransformRaysNode>;
	void setParameters(Mat3x4f transform) { this->transform = transform; }

	// Node
	void enqueueExecImpl() override;

	// Data getters
	Array<Mat3x4f>::ConstPtr getRays() const override { return transformedRays; }
	Mat3x4f getCumulativeRayTransfrom() const override { return transform * input->getCumulativeRayTransfrom(); }

private:
	Mat3x4f transform;
	DeviceAsyncArray<Mat3x4f>::Ptr transformedRays = DeviceAsyncArray<Mat3x4f>::create(arrayMgr);
};

struct FromMat3x4fRaysNode : IRaysNode, INoInputNode
{
	using Ptr = std::shared_ptr<FromMat3x4fRaysNode>;
	void setParameters(const Mat3x4f* raysRaw, size_t rayCount);

	// Node
	void enqueueExecImpl() override {}

	// Transforms
	size_t getRayCount() const override { return rays->getCount(); }
	Array<Mat3x4f>::ConstPtr getRays() const override { return rays; }

	// Ring Ids
	std::optional<size_t> getRingIdsCount() const override { return std::nullopt; }
	std::optional<Array<int>::ConstPtr> getRingIds() const override { return std::nullopt; }

	// Ranges
	std::optional<size_t> getRangesCount() const override { return std::nullopt; }
	std::optional<Array<Vec2f>::ConstPtr> getRanges() const override { return std::nullopt; }

	// Firing time offsets
	std::optional<size_t> getTimeOffsetsCount() const override { return std::nullopt; }
	std::optional<Array<float>::ConstPtr> getTimeOffsets() const override { return std::nullopt; }

private:
	DeviceAsyncArray<Mat3x4f>::Ptr rays = DeviceAsyncArray<Mat3x4f>::create(arrayMgr);
};

struct SetRingIdsRaysNode : IRaysNodeSingleInput
{
	using Ptr = std::shared_ptr<SetRingIdsRaysNode>;
	void setParameters(const int* ringIdsRaw, size_t ringIdsCount);

	// Node
	void validateImpl() override;
	void enqueueExecImpl() override {}

	// Rays description
	std::optional<size_t> getRingIdsCount() const override { return ringIds->getCount(); }

	// Data getters
	std::optional<Array<int>::ConstPtr> getRingIds() const override { return ringIds; }

private:
	DeviceAsyncArray<int>::Ptr ringIds = DeviceAsyncArray<int>::create(arrayMgr);
};

struct SetRangeRaysNode : IRaysNodeSingleInput
{
	using Ptr = std::shared_ptr<SetRangeRaysNode>;
	void setParameters(const Vec2f* rangesRaw, size_t rangesCount);

	// Node
	void validateImpl() override;
	void enqueueExecImpl() override {}

	// Rays description
	std::optional<std::size_t> getRangesCount() const override { return ranges->getCount(); }

	// Data getters
	std::optional<Array<Vec2f>::ConstPtr> getRanges() const override { return ranges; }

private:
	Array<Vec2f>::Ptr ranges = DeviceAsyncArray<Vec2f>::create(arrayMgr);
};

struct SetTimeOffsetsRaysNode : IRaysNodeSingleInput
{
	using Ptr = std::shared_ptr<SetTimeOffsetsRaysNode>;
	void setParameters(const float* raysTimeOffsets, size_t timeOffsetsCount);

	// Node
	void validateImpl() override;
	void enqueueExecImpl() override {}

	// Rays description
	std::optional<size_t> getTimeOffsetsCount() const override { return timeOffsets->getCount(); }

	// Data getters
	std::optional<Array<float>::ConstPtr> getTimeOffsets() const override { return timeOffsets; }

private:
	Array<float>::Ptr timeOffsets = DeviceAsyncArray<float>::create(arrayMgr);
};

struct YieldPointsNode : IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<YieldPointsNode>;
	void setParameters(const std::vector<rgl_field_t>& fields);

	// Node
	void enqueueExecImpl() override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override { return fields; }

	// Data getters
	IAnyArray::ConstPtr getFieldData(rgl_field_t field) override { return results.at(field); }

	HostPinnedArray<Field<XYZ_VEC3_F32>::type>::Ptr getXYZCache() { return xyzHostCache; }

private:
	std::vector<rgl_field_t> fields;
	std::unordered_map<rgl_field_t, IAnyArray::ConstPtr> results;
	HostPinnedArray<Field<XYZ_VEC3_F32>::type>::Ptr xyzHostCache = HostPinnedArray<Field<XYZ_VEC3_F32>::type>::create();
};

struct SpatialMergePointsNode : IPointsNode
{
	using Ptr = std::shared_ptr<SpatialMergePointsNode>;
	void setParameters(const std::vector<rgl_field_t>& fields);

	// Node
	void validateImpl() override;
	void enqueueExecImpl() override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override
	{
		return {std::views::keys(mergedData).begin(), std::views::keys(mergedData).end()};
	}

	// Point cloud description
	bool isDense() const override;
	bool hasField(rgl_field_t field) const override { return mergedData.contains(field); }
	std::size_t getWidth() const override { return width; }
	std::size_t getHeight() const override { return 1; }

	// Data getters
	IAnyArray::ConstPtr getFieldData(rgl_field_t field) override
	{
		return std::const_pointer_cast<const IAnyArray>(mergedData.at(field));
	}

private:
	std::vector<IPointsNode::Ptr> pointInputs;
	std::unordered_map<rgl_field_t, IAnyArray::Ptr> mergedData;
	std::size_t width = 0;
};

struct TemporalMergePointsNode : IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<YieldPointsNode>;
	void setParameters(const std::vector<rgl_field_t>& fields);

	// Node
	void validateImpl() override;
	void enqueueExecImpl() override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override
	{
		return {std::views::keys(mergedData).begin(), std::views::keys(mergedData).end()};
	}

	// Point cloud description
	bool hasField(rgl_field_t field) const override { return mergedData.contains(field); }
	std::size_t getWidth() const override { return width; }

	// Data getters
	IAnyArray::ConstPtr getFieldData(rgl_field_t field) override
	{
		return std::const_pointer_cast<const IAnyArray>(mergedData.at(field));
	}

private:
	std::unordered_map<rgl_field_t, IAnyArray::Ptr> mergedData;
	std::size_t width = 0;
};

struct FromArrayPointsNode : IPointsNode, INoInputNode
{
	using Ptr = std::shared_ptr<FromArrayPointsNode>;
	void setParameters(const void* points, size_t pointCount, const std::vector<rgl_field_t>& fields);

	// Node
	void enqueueExecImpl() override {}

	// Point cloud description
	bool isDense() const override
	{
		// If point cloud doesn't contain IS_HIT field we assume all points are hits.
		return !fieldData.contains(RGL_FIELD_IS_HIT_I32);
	}
	bool hasField(rgl_field_t field) const override { return fieldData.contains(field); }
	size_t getWidth() const override { return width; }
	size_t getHeight() const override { return 1; }

	// Data getters
	IAnyArray::ConstPtr getFieldData(rgl_field_t field) override
	{
		return std::const_pointer_cast<const IAnyArray>(fieldData.at(field));
	}

private:
	GPUFieldDescBuilder gpuFieldDescBuilder;
	std::vector<std::pair<rgl_field_t, void*>> getFieldToPointerMappings(const std::vector<rgl_field_t>& fields);

	std::unordered_map<rgl_field_t, IAnyArray::Ptr> fieldData;
	size_t width = 0;
};

struct GaussianNoiseAngularRaysNode : IRaysNodeSingleInput
{
	using Ptr = std::shared_ptr<GaussianNoiseAngularRaysNode>;

	void setParameters(float mean, float stSev, rgl_axis_t rotationAxis);

	// Node
	void enqueueExecImpl() override;

	// Data getters
	Array<Mat3x4f>::ConstPtr getRays() const override { return rays; }

private:
	float mean;
	float stDev;
	rgl_axis_t rotationAxis;
	std::random_device randomDevice;

	DeviceAsyncArray<curandStatePhilox4_32_10_t>::Ptr randomizationStates =
	    DeviceAsyncArray<curandStatePhilox4_32_10_t>::create(arrayMgr);
	DeviceAsyncArray<Mat3x4f>::Ptr rays = DeviceAsyncArray<Mat3x4f>::create(arrayMgr);
};

struct GaussianNoiseAngularHitpointNode : IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<GaussianNoiseAngularHitpointNode>;

	void setParameters(float mean, float stDev, rgl_axis_t rotationAxis);

	// Node
	void validateImpl() override;
	void enqueueExecImpl() override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override { return {XYZ_VEC3_F32}; }

	// Data getters
	IAnyArray::ConstPtr getFieldData(rgl_field_t field) override;

private:
	float mean;
	float stDev;
	rgl_axis_t rotationAxis;
	std::random_device randomDevice;

	DeviceAsyncArray<curandStatePhilox4_32_10_t>::Ptr randomizationStates =
	    DeviceAsyncArray<curandStatePhilox4_32_10_t>::create(arrayMgr);
	DeviceAsyncArray<Field<XYZ_VEC3_F32>::type>::Ptr outXyz = DeviceAsyncArray<Field<XYZ_VEC3_F32>::type>::create(arrayMgr);
	DeviceAsyncArray<Field<DISTANCE_F32>::type>::Ptr outDistance = nullptr;
};

struct GaussianNoiseDistanceNode : IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<GaussianNoiseDistanceNode>;

	void setParameters(float mean, float stDevBase, float stDevRisePerMeter);

	// Node
	void enqueueExecImpl() override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override { return {XYZ_VEC3_F32, DISTANCE_F32}; };

	// Data getters
	IAnyArray::ConstPtr getFieldData(rgl_field_t field) override;

private:
	float mean;
	float stDevBase;
	float stDevRisePerMeter;
	std::random_device randomDevice;

	DeviceAsyncArray<curandStatePhilox4_32_10_t>::Ptr randomizationStates =
	    DeviceAsyncArray<curandStatePhilox4_32_10_t>::create(arrayMgr);
	DeviceAsyncArray<Field<XYZ_VEC3_F32>::type>::Ptr outXyz = DeviceAsyncArray<Field<XYZ_VEC3_F32>::type>::create(arrayMgr);
	DeviceAsyncArray<Field<DISTANCE_F32>::type>::Ptr outDistance = DeviceAsyncArray<Field<DISTANCE_F32>::type>::create(
	    arrayMgr);
};

struct RadarPostprocessPointsNode : IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<RadarPostprocessPointsNode>;

	void setParameters(const std::vector<rgl_radar_scope_t>& radarScopes, float rayAzimuthStepRad, float rayElevationStepRad,
	                   float frequency, float powerTransmitted, float cumulativeDeviceGain, float receivedNoiseMean,
	                   float receivedNoiseStDev);

	// Node
	void validateImpl() override;
	void enqueueExecImpl() override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override;

	// Point cloud description
	size_t getWidth() const override;
	size_t getHeight() const override { return 1; }

	// Data getters
	IAnyArray::ConstPtr getFieldData(rgl_field_t field) override;

	const std::vector<Aabb3Df>& getClusterAabbs() const { return clusterAabbs; }

private:
	// Data containers
	std::vector<Field<RAY_IDX_U32>::type> filteredIndicesHost;
	DeviceAsyncArray<Field<RAY_IDX_U32>::type>::Ptr filteredIndices = DeviceAsyncArray<Field<RAY_IDX_U32>::type>::create(
	    arrayMgr);
	HostPinnedArray<Field<XYZ_VEC3_F32>::type>::Ptr xyzInputHost = HostPinnedArray<Field<XYZ_VEC3_F32>::type>::create();
	HostPinnedArray<Field<DISTANCE_F32>::type>::Ptr distanceInputHost = HostPinnedArray<Field<DISTANCE_F32>::type>::create();
	HostPinnedArray<Field<AZIMUTH_F32>::type>::Ptr azimuthInputHost = HostPinnedArray<Field<AZIMUTH_F32>::type>::create();
	HostPinnedArray<Field<RADIAL_SPEED_F32>::type>::Ptr radialSpeedInputHost =
	    HostPinnedArray<Field<RADIAL_SPEED_F32>::type>::create();
	HostPinnedArray<Field<ELEVATION_F32>::type>::Ptr elevationInputHost = HostPinnedArray<Field<ELEVATION_F32>::type>::create();
	DeviceAsyncArray<Vector<3, thrust::complex<float>>>::Ptr outBUBRFactorDev =
	    DeviceAsyncArray<Vector<3, thrust::complex<float>>>::create(arrayMgr);
	HostPinnedArray<Vector<3, thrust::complex<float>>>::Ptr outBUBRFactorHost =
	    HostPinnedArray<Vector<3, thrust::complex<float>>>::create();

	HostPageableArray<Field<RCS_F32>::type>::Ptr clusterRcsHost = HostPageableArray<Field<RCS_F32>::type>::create();
	HostPageableArray<Field<POWER_F32>::type>::Ptr clusterPowerHost = HostPageableArray<Field<POWER_F32>::type>::create();
	HostPageableArray<Field<NOISE_F32>::type>::Ptr clusterNoiseHost = HostPageableArray<Field<NOISE_F32>::type>::create();
	HostPageableArray<Field<SNR_F32>::type>::Ptr clusterSnrHost = HostPageableArray<Field<SNR_F32>::type>::create();

	DeviceAsyncArray<Field<RCS_F32>::type>::Ptr clusterRcsDev = DeviceAsyncArray<Field<RCS_F32>::type>::create(arrayMgr);
	DeviceAsyncArray<Field<POWER_F32>::type>::Ptr clusterPowerDev = DeviceAsyncArray<Field<POWER_F32>::type>::create(arrayMgr);
	DeviceAsyncArray<Field<NOISE_F32>::type>::Ptr clusterNoiseDev = DeviceAsyncArray<Field<NOISE_F32>::type>::create(arrayMgr);
	DeviceAsyncArray<Field<SNR_F32>::type>::Ptr clusterSnrDev = DeviceAsyncArray<Field<SNR_F32>::type>::create(arrayMgr);

	float rayAzimuthStepRad;
	float rayElevationStepRad;
	float frequencyHz;
	float powerTransmittedDbm;
	float cumulativeDeviceGainDbi;
	float receivedNoiseMeanDb;
	float receivedNoiseStDevDb;

	std::vector<rgl_radar_scope_t> radarScopes;
	std::vector<Aabb3Df> clusterAabbs;

	std::random_device randomDevice;

	// RGL related members
	std::mutex getFieldDataMutex;
	mutable CacheManager<rgl_field_t, IAnyArray::Ptr> cacheManager;

	struct RadarCluster
	{
		RadarCluster(Field<RAY_IDX_U32>::type index, float distance, float azimuth, float radialSpeed, float elevation);
		RadarCluster(RadarCluster&& other) noexcept = default;
		RadarCluster& operator=(RadarCluster&& other) noexcept = default;

		void addPoint(Field<RAY_IDX_U32>::type index, float distance, float azimuth, float radialSpeed, float elevation);
		inline bool isCandidate(float distance, float azimuth, float radialSpeed, const rgl_radar_scope_t& separations) const;
		inline bool canMergeWith(const RadarCluster& other, const std::vector<rgl_radar_scope_t>& radarScopes) const;
		void takeIndicesFrom(RadarCluster&& other);
		Field<RAY_IDX_U32>::type findDirectionalCenterIndex(const Field<AZIMUTH_F32>::type* azimuths,
		                                                    const Field<ELEVATION_F32>::type* elevations) const;

		std::vector<Field<RAY_IDX_U32>::type> indices;
		Vector<2, Field<DISTANCE_F32>::type> minMaxDistance;
		Vector<2, Field<AZIMUTH_F32>::type> minMaxAzimuth;
		Vector<2, Field<RADIAL_SPEED_F32>::type> minMaxRadialSpeed;
		Vector<2, Field<ELEVATION_F32>::type> minMaxElevation; // For finding directional center only
	};
};

struct RadarTrackObjectsNode : IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<RadarTrackObjectsNode>;

	enum class ObjectStatus : uint8_t
	{
		Measured = 0,
		New = 1,
		Predicted = 2,
		Invalid = 255
	};

	enum class MovementStatus : uint8_t
	{
		Moved = 0,
		Stationary = 1,
		Invalid = 255
	};

	struct ObjectBounds
	{
		Field<ENTITY_ID_I32>::type mostCommonEntityId = RGL_ENTITY_INVALID_ID;
		Vec3f position{0};
		Aabb3Df aabb{};
		Vec3f absVelocity{};
		Vec3f relVelocity{};
	};

	struct ClassificationProbabilities
	{
		float existence{100.0f};
		uint8_t classCar{0};
		uint8_t classTruck{0};
		uint8_t classMotorcycle{0};
		uint8_t classBicycle{0};
		uint8_t classPedestrian{0};
		uint8_t classAnimal{0};
		uint8_t classHazard{0};
		uint8_t classUnknown{0};
	};

	struct ObjectState
	{
		uint32_t id{0};
		uint32_t creationTime{0};
		uint32_t lastMeasuredTime{0};
		ObjectStatus objectStatus{ObjectStatus::Invalid};
		MovementStatus movementStatus{MovementStatus::Invalid};
		ClassificationProbabilities classificationProbabilities{};

		RunningStats<Vec3f> position{};
		RunningStats<float> orientation{};
		RunningStats<Vec3f> absVelocity{};
		RunningStats<Vec3f> relVelocity{};
		RunningStats<Vec3f> absAccel{};
		RunningStats<Vec3f> relAccel{};
		RunningStats<float> orientationRate{};
		RunningStats<float> length{};
		RunningStats<float> width{};

		// Workaround to be able to publish objects in the sensor frame via UDP
		// Transform points node cannot transform ObjectState
		Vec3f positionSensorFrame{};
	};

	RadarTrackObjectsNode();

	void setParameters(float distanceThreshold, float azimuthThreshold, float elevationThreshold, float radialSpeedThreshold,
	                   float maxMatchingDistance, float maxPredictionTimeFrame, float movementSensitivity);

	void setObjectClasses(const int32_t* entityIds, const rgl_radar_object_class_t* objectClasses, int32_t count);

	void enqueueExecImpl() override;

	bool hasField(rgl_field_t field) const override { return fieldData.contains(field); }

	std::vector<rgl_field_t> getRequiredFieldList() const override;

	// Data getters
	IAnyArray::ConstPtr getFieldData(rgl_field_t field) override { return fieldData.at(field); }
	size_t getWidth() const override { return fieldData.empty() ? 0 : fieldData.begin()->second->getCount(); }
	size_t getHeight() const override { return 1; } // In fact, this will be only a 1-dimensional array.

	const std::list<ObjectState>& getObjectStates() const { return objectStates; }

private:
	Vec3f predictObjectPosition(const ObjectState& objectState, double deltaTimeMs) const;
	void parseEntityIdToClassProbability(Field<ENTITY_ID_I32>::type entityId, ClassificationProbabilities& probabilities);
	void createObjectState(const ObjectBounds& objectBounds, double currentTimeMs);
	void updateObjectState(ObjectState& objectState, const Vec3f& updatedPosition, const Aabb3Df& updatedAabb,
	                       ObjectStatus objectStatus, double currentTimeMs, double deltaTimeMs, const Vec3f& absVelocity,
	                       const Vec3f& relVelocity);
	void updateOutputData();

	std::list<ObjectState> objectStates;
	std::unordered_map<Field<ENTITY_ID_I32>::type, rgl_radar_object_class_t> entityIdsToClasses;
	std::unordered_map<rgl_field_t, IAnyArray::Ptr> fieldData; // All should be DeviceAsyncArray

	uint32_t objectIDCounter = 0; // Not static - I assume each ObjectTrackingNode is like a separate radar.
	std::queue<uint32_t> objectIDPoll;

	float distanceThreshold;
	float azimuthThreshold;
	float elevationThreshold;
	float radialSpeedThreshold;

	float maxMatchingDistance =
	    1.0f; // Max distance between predicted and newly detected position to match objects between frames.
	float maxPredictionTimeFrame =
	    500.0f;                        // Maximum time in milliseconds that can pass between two detections of the same object.
	                                   // In other words, how long object state can be predicted until it will be declared lost.
	float movementSensitivity = 0.01f; // Max velocity for an object to be qualified as MovementStatus::Stationary.

	Mat3x4f lookAtSensorFrameTransform{Mat3x4f::identity()};
	decltype(Time::zero().asMilliseconds()) currentTime{Time::zero().asMilliseconds()};

	HostPinnedArray<Field<XYZ_VEC3_F32>::type>::Ptr xyzHostPtr = HostPinnedArray<Field<XYZ_VEC3_F32>::type>::create();
	HostPinnedArray<Field<DISTANCE_F32>::type>::Ptr distanceHostPtr = HostPinnedArray<Field<DISTANCE_F32>::type>::create();
	HostPinnedArray<Field<AZIMUTH_F32>::type>::Ptr azimuthHostPtr = HostPinnedArray<Field<AZIMUTH_F32>::type>::create();
	HostPinnedArray<Field<ELEVATION_F32>::type>::Ptr elevationHostPtr = HostPinnedArray<Field<ELEVATION_F32>::type>::create();
	HostPinnedArray<Field<RADIAL_SPEED_F32>::type>::Ptr radialSpeedHostPtr =
	    HostPinnedArray<Field<RADIAL_SPEED_F32>::type>::create();
	HostPinnedArray<Field<ENTITY_ID_I32>::type>::Ptr entityIdHostPtr = HostPinnedArray<Field<ENTITY_ID_I32>::type>::create();
	HostPinnedArray<Field<RELATIVE_VELOCITY_VEC3_F32>::type>::Ptr velocityRelHostPtr =
	    HostPinnedArray<Field<RELATIVE_VELOCITY_VEC3_F32>::type>::create();
	HostPinnedArray<Field<ABSOLUTE_VELOCITY_VEC3_F32>::type>::Ptr velocityAbsHostPtr =
	    HostPinnedArray<Field<ABSOLUTE_VELOCITY_VEC3_F32>::type>::create();

	HostPinnedArray<Field<XYZ_VEC3_F32>::type>::Ptr outXyzHostPtr = HostPinnedArray<Field<XYZ_VEC3_F32>::type>::create();
	HostPinnedArray<Field<ENTITY_ID_I32>::type>::Ptr outEntityIdHostPtr = HostPinnedArray<Field<ENTITY_ID_I32>::type>::create();
};

struct FilterGroundPointsNode : IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<FilterGroundPointsNode>;
	void setParameters(const Vec3f& sensor_up_vector, float ground_angle_threshold);

	// Node
	void validateImpl() override;
	void enqueueExecImpl() override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override { return {XYZ_VEC3_F32, NORMAL_VEC3_F32}; };

	// Data getters
	IAnyArray::ConstPtr getFieldData(rgl_field_t field) override;

private:
	Vec3f sensor_up_vector;
	float ground_angle_threshold;
	DeviceAsyncArray<Field<IS_GROUND_I32>::type>::Ptr outNonGround = DeviceAsyncArray<Field<IS_GROUND_I32>::type>::create(
	    arrayMgr);
};
