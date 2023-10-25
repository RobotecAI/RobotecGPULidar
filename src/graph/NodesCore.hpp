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
#include <curand_kernel.h>

#include <graph/Node.hpp>
#include <graph/Interfaces.hpp>
#include <gpu/RaytraceRequestContext.hpp>
#include <gpu/nodeKernels.hpp>
#include <CacheManager.hpp>
#include <GPUFieldDescBuilder.hpp>


struct FormatPointsNode : IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<FormatPointsNode>;
	void setParameters(const std::vector<rgl_field_t>& fields);

	// Node
	void enqueueExecImpl() override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override { return fields; }

	// Point cloud description
	bool hasField(rgl_field_t field) const override { return field == RGL_FIELD_DYNAMIC_FORMAT || std::find(fields.begin(), fields.end(), field) != fields.end(); }

	// Data getters
	IAnyArray::ConstPtr getFieldData(rgl_field_t field) override;
	std::size_t getFieldPointSize(rgl_field_t field) const override;

	// Actual implementation of formatting made public for other nodes
	static void formatAsync(DeviceAsyncArray<char>::Ptr output, const IPointsNode::Ptr& input,
	                        const std::vector<rgl_field_t>& fields, GPUFieldDescBuilder& gpuFieldDescBuilder);

private:
	static std::vector<std::pair<rgl_field_t, const void*>> getFieldToPointerMappings(const IPointsNode::Ptr& input,
	                                                                                  const std::vector<rgl_field_t>& fields);

	std::vector<rgl_field_t> fields;
	DeviceAsyncArray<char>::Ptr output = DeviceAsyncArray<char>::create(arrayMgr);
	HostPinnedArray<char>::Ptr outputHost = HostPinnedArray<char>::create();
	GPUFieldDescBuilder gpuFieldDescBuilder;
};

struct CompactPointsNode : IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<CompactPointsNode>;
	void setParameters() {}

	// Node
	void enqueueExecImpl() override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override { return {IS_HIT_I32}; }

	// Point cloud description
	bool isDense() const override { return true; }
	size_t getWidth() const override;
	size_t getHeight() const override { return 1; }

	// Data getters
	IAnyArray::ConstPtr getFieldData(rgl_field_t field) override;

private:
	size_t width = {0};
	DeviceAsyncArray<CompactionIndexType>::Ptr inclusivePrefixSum = DeviceAsyncArray<CompactionIndexType>::create(arrayMgr);
	CacheManager<rgl_field_t, IAnyArray::Ptr> cacheManager;
	std::mutex getFieldDataMutex;
};

struct RaytraceNode : IPointsNode
{
	using Ptr = std::shared_ptr<RaytraceNode>;
	void setParameters(std::shared_ptr<Scene> scene, float range) { this->scene = scene; this->range = range; }

	// Node
	void validateImpl() override;
	void enqueueExecImpl() override;

	// Point cloud description
	bool isDense() const override { return false; }
	bool hasField(rgl_field_t field) const override { return fieldData.contains(field); }
	size_t getWidth() const override { return raysNode ? raysNode->getRayCount() : 0; }
	size_t getHeight() const override { return 1; }  // TODO: implement height in use_rays

	Mat3x4f getLookAtOriginTransform() const override { return raysNode->getCumulativeRayTransfrom().inverse(); }

	// Data getters
	IAnyArray::ConstPtr getFieldData(rgl_field_t field) override
	{ return std::const_pointer_cast<const IAnyArray>(fieldData.at(field)); }

private:
	float range;
	IRaysNode::Ptr raysNode;
	std::shared_ptr<Scene> scene;
	std::unordered_map<rgl_field_t, IAnyArray::Ptr> fieldData; // All should be DeviceAsyncArray
	HostPinnedArray<RaytraceRequestContext>::Ptr requestCtxHst = HostPinnedArray<RaytraceRequestContext>::create();
	DeviceAsyncArray<RaytraceRequestContext>::Ptr requestCtxDev = DeviceAsyncArray<RaytraceRequestContext>::create(
			arrayMgr);

	template<rgl_field_t>
	auto getPtrTo();

	std::set<rgl_field_t> findFieldsToCompute();
	void setFields(const std::set<rgl_field_t>& fields);
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
	std::vector<rgl_field_t> getRequiredFieldList() const override { return {XYZ_F32}; }

	Mat3x4f getLookAtOriginTransform() const override { return transform.inverse() * input->getLookAtOriginTransform(); }

	// Data getters
	IAnyArray::ConstPtr getFieldData(rgl_field_t field) override;

private:
	Mat3x4f transform;
	DeviceAsyncArray<Field<XYZ_F32>::type>::Ptr output = DeviceAsyncArray<Field<XYZ_F32>::type>::create(arrayMgr);
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

struct FromMat3x4fRaysNode : virtual IRaysNode, virtual INoInputNode
{
	using Ptr = std::shared_ptr<FromMat3x4fRaysNode>;
	void setParameters(const Mat3x4f* raysRaw, size_t rayCount);

	// Node
	void enqueueExecImpl() override {}

	// Rays description
	size_t getRayCount() const override { return rays->getCount(); }
	std::optional<size_t> getRingIdsCount() const override { return std::nullopt; }

	// Data getters
	Array<Mat3x4f>::ConstPtr getRays() const override { return rays; }
	std::optional<Array<int>::ConstPtr> getRingIds() const override { return std::nullopt; }

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

	HostPinnedArray<Field<XYZ_F32>::type>::Ptr getXYZCache() { return xyzHostCache; }

private:
	std::vector<rgl_field_t> fields;
	std::unordered_map<rgl_field_t, IAnyArray::ConstPtr> results;
	HostPinnedArray<Field<XYZ_F32>::type>::Ptr xyzHostCache = HostPinnedArray<Field<XYZ_F32>::type>::create();
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
	{ return { std::views::keys(mergedData).begin(), std::views::keys(mergedData).end() }; }

	// Point cloud description
	bool isDense() const override;
	bool hasField(rgl_field_t field) const override { return mergedData.contains(field); }
	std::size_t getWidth() const override { return width; }
	std::size_t getHeight() const override { return 1; }

	// Data getters
	IAnyArray::ConstPtr getFieldData(rgl_field_t field) override
	{ return std::const_pointer_cast<const IAnyArray>(mergedData.at(field)); }

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
	{ return { std::views::keys(mergedData).begin(), std::views::keys(mergedData).end() }; }

	// Point cloud description
	bool hasField(rgl_field_t field) const override { return mergedData.contains(field); }
	std::size_t getWidth() const override { return width; }

	// Data getters
	IAnyArray::ConstPtr getFieldData(rgl_field_t field) override
	{ return std::const_pointer_cast<const IAnyArray>(mergedData.at(field)); }

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
	bool isDense() const override { return false; }
	bool hasField(rgl_field_t field) const override { return fieldData.contains(field); }
	size_t getWidth() const override { return width; }
	size_t getHeight() const override { return 1; }

	// Data getters
	IAnyArray::ConstPtr getFieldData(rgl_field_t field) override
	{ return std::const_pointer_cast<const IAnyArray>(fieldData.at(field)); }

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
	void validateImpl() override;
	void enqueueExecImpl() override;

	// Data getters
	Array<Mat3x4f>::ConstPtr getRays() const override { return rays; }

private:
	float mean;
	float stDev;
	rgl_axis_t rotationAxis;
	std::random_device randomDevice;
	Mat3x4f lookAtOriginTransform;

	DeviceAsyncArray<curandStatePhilox4_32_10_t>::Ptr randomizationStates = DeviceAsyncArray<curandStatePhilox4_32_10_t>::create(arrayMgr);
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
	std::vector<rgl_field_t> getRequiredFieldList() const override { return {XYZ_F32}; }

	// Data getters
	IAnyArray::ConstPtr getFieldData(rgl_field_t field) override;

private:
	float mean;
	float stDev;
	rgl_axis_t rotationAxis;
	std::random_device randomDevice;
	Mat3x4f lookAtOriginTransform;

	DeviceAsyncArray<curandStatePhilox4_32_10_t>::Ptr randomizationStates = DeviceAsyncArray<curandStatePhilox4_32_10_t>::create(arrayMgr);
	DeviceAsyncArray<Field<XYZ_F32>::type>::Ptr outXyz = DeviceAsyncArray<Field<XYZ_F32>::type>::create(arrayMgr);
	DeviceAsyncArray<Field<DISTANCE_F32>::type>::Ptr outDistance = nullptr;
};

struct GaussianNoiseDistanceNode : IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<GaussianNoiseDistanceNode>;

	void setParameters(float mean, float stDevBase, float stDevRisePerMeter);

	// Node
	void validateImpl() override;
	void enqueueExecImpl() override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override { return {XYZ_F32}; };

	// Data getters
	IAnyArray::ConstPtr getFieldData(rgl_field_t field) override;

private:
	float mean;
	float stDevBase;
	float stDevRisePerMeter;
	std::random_device randomDevice;
	Mat3x4f lookAtOriginTransform;

	DeviceAsyncArray<curandStatePhilox4_32_10_t>::Ptr randomizationStates = DeviceAsyncArray<curandStatePhilox4_32_10_t>::create(arrayMgr);
	DeviceAsyncArray<Field<XYZ_F32>::type>::Ptr outXyz = DeviceAsyncArray<Field<XYZ_F32>::type>::create(arrayMgr);
	DeviceAsyncArray<Field<DISTANCE_F32>::type>::Ptr outDistance = nullptr;
};


struct SimulateSnowPointsNode :IPointsNodeSingleInput
{
        using Ptr = std::shared_ptr<SimulateSnowPointsNode>;

        void setParameters(float maxRange,
            float rainRate, float meanSnowflakeDiameter, float terminalVelocity,
            float density, int32_t numberOfChannels, float beamDivergence);

        // Node
        void validateImpl() override;
        void enqueueExecImpl() override;

        // Node requirements
        std::vector<rgl_field_t> getRequiredFieldList() const override { return {XYZ_F32}; };

        // Data getters
        IAnyArray::ConstPtr getFieldData(rgl_field_t field) override;

private:
    int32_t numberOfChannels;
    int numberOfFlakesPerChannel = 0;
    float snowfallRate;
    float terminalVelocity;
    float maxRange;
    float beamDivergence;

    std::random_device randomDevice;
    DeviceAsyncArray<curandStatePhilox4_32_10_t>::Ptr randomizationStates = DeviceAsyncArray<curandStatePhilox4_32_10_t>::create(arrayMgr);
    std::vector<DeviceAsyncArray<Vec3f>::Ptr> snowflakesDisks;  // (X, Y) coordinates of snowflakes with diameter of flake in Z coordinate.
    DeviceAsyncArray<Field<XYZ_F32>::type>::Ptr outXyz = DeviceAsyncArray<Field<XYZ_F32>::type>::create(arrayMgr);
};
