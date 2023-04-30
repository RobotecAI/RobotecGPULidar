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
#include <VArray.hpp>
#include <VArrayProxy.hpp>

/**
 * Notes for maintainers:
 *
 * Some nodes define extra methods such as get*Count() to obtain the number of elements in their output buffers.
 * This is purposeful: interface-level methods are guaranteed to return correct number of elements or throw,
 * while buffers sizes are not reliable, since they can be resized in enqueueExecImpl().
 *
 * For methods taking cudaStream as an argument, it is legal to return VArray that becomes valid only after stream
 * operations prior to the return are finished.
 */

// TODO(prybicki): Consider templatizing IPointCloudNode with its InputInterface type.
// TODO(prybicki): This would implement automatic getExactlyOneInputOfType() and method forwarding.

struct FormatPointsNode : IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<FormatPointsNode>;
	void setParameters(const std::vector<rgl_field_t>& fields);

	// Node
	void enqueueExecImpl(cudaStream_t stream) override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override { return fields; }

	// Point cloud description
	bool hasField(rgl_field_t field) const override { return std::find(fields.begin(), fields.end(), field) != fields.end(); }

	// Data getters
	VArray::ConstPtr getFieldData(rgl_field_t field, cudaStream_t stream) override;
	std::size_t getFieldPointSize(rgl_field_t field) const override;

	// Actual implementation of formatting made public for other nodes
	static void formatAsync(const VArray::Ptr& output, const IPointsNode::Ptr& input,
	                        const std::vector<rgl_field_t>& fields, cudaStream_t stream,
	                        GPUFieldDescBuilder& gpuFieldDescBuilder);

private:
	static std::vector<std::pair<rgl_field_t, const void*>> getFieldToPointerMappings(const IPointsNode::Ptr& input,
	                                                                                  const std::vector<rgl_field_t>& fields,
	                                                                                  cudaStream_t stream);

	std::vector<rgl_field_t> fields;
	VArray::Ptr output = VArray::create<char>();
	GPUFieldDescBuilder gpuFieldDescBuilder;
};

struct CompactPointsNode : IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<CompactPointsNode>;
	void setParameters() {}

	CompactPointsNode()	{ CHECK_CUDA(cudaEventCreate(&finishedEvent, cudaEventDisableTiming)); }
	virtual ~CompactPointsNode() { CHECK_CUDA_NO_THROW(cudaEventDestroy(finishedEvent)); }

	// Node
	void enqueueExecImpl(cudaStream_t stream) override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override { return {IS_HIT_I32}; }

	// Point cloud description
	bool isDense() const override { return true; }
	size_t getWidth() const override;
	size_t getHeight() const override { return 1; }

	// Data getters
	VArray::ConstPtr getFieldData(rgl_field_t field, cudaStream_t stream) override;

private:
	size_t width = {0};
	cudaEvent_t finishedEvent = nullptr;
	VArrayProxy<CompactionIndexType>::Ptr inclusivePrefixSum = VArrayProxy<CompactionIndexType>::create();
	CacheManager<rgl_field_t, VArray::Ptr> cacheManager;
};

struct RaytraceNode : IPointsNode
{
	using Ptr = std::shared_ptr<RaytraceNode>;
	void setParameters(std::shared_ptr<Scene> scene, float range) { this->scene = scene; this->range = range; }

	// Node
	void validateImpl() override;
	void enqueueExecImpl(cudaStream_t stream) override;

	// Point cloud description
	bool isDense() const override { return false; }
	bool hasField(rgl_field_t field) const override { return fieldData.contains(field); }
	size_t getWidth() const override { return raysNode ? raysNode->getRayCount() : 0; }
	size_t getHeight() const override { return 1; }  // TODO: implement height in use_rays

	Mat3x4f getLookAtOriginTransform() const override { return raysNode->getCumulativeRayTransfrom().inverse(); }

	// Data getters
	VArray::ConstPtr getFieldData(rgl_field_t field, cudaStream_t stream) override
	{ return std::const_pointer_cast<const VArray>(fieldData.at(field)); }

private:
	float range;
	std::shared_ptr<Scene> scene;
	IRaysNode::Ptr raysNode;
	VArrayProxy<RaytraceRequestContext>::Ptr requestCtx = VArrayProxy<RaytraceRequestContext>::create(1);
	std::unordered_map<rgl_field_t, VArray::Ptr> fieldData;

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
	void enqueueExecImpl(cudaStream_t stream) override;
	std::string getArgsString() const override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override { return {XYZ_F32}; }

	Mat3x4f getLookAtOriginTransform() const override { return transform.inverse() * input->getLookAtOriginTransform(); }

	// Data getters
	VArray::ConstPtr getFieldData(rgl_field_t field, cudaStream_t stream) override;

private:
	Mat3x4f transform;
	VArrayProxy<Field<XYZ_F32>::type>::Ptr output = VArrayProxy<Field<XYZ_F32>::type>::create();
};

struct TransformRaysNode : IRaysNodeSingleInput
{
	using Ptr = std::shared_ptr<TransformRaysNode>;
	void setParameters(Mat3x4f transform) { this->transform = transform; }

	// Node
	void enqueueExecImpl(cudaStream_t stream) override;

	// Data getters
	VArrayProxy<Mat3x4f>::ConstPtr getRays() const override { return rays; }
	Mat3x4f getCumulativeRayTransfrom() const override { return transform * input->getCumulativeRayTransfrom(); }

private:
	Mat3x4f transform;
	VArrayProxy<Mat3x4f>::Ptr rays = VArrayProxy<Mat3x4f>::create();
};

struct FromMat3x4fRaysNode : virtual IRaysNode, virtual INoInputNode
{
	using Ptr = std::shared_ptr<FromMat3x4fRaysNode>;
	void setParameters(const Mat3x4f* raysRaw, size_t rayCount);

	// Node
	void enqueueExecImpl(cudaStream_t stream) override {}

	// Rays description
	size_t getRayCount() const override { return rays->getCount(); }
	std::optional<size_t> getRingIdsCount() const override { return std::nullopt; }

	// Data getters
	VArrayProxy<Mat3x4f>::ConstPtr getRays() const override { return rays; }
	std::optional<VArrayProxy<int>::ConstPtr> getRingIds() const override { return std::nullopt; }

private:
	VArrayProxy<Mat3x4f>::Ptr rays = VArrayProxy<Mat3x4f>::create();
};

struct SetRingIdsRaysNode : IRaysNodeSingleInput
{
	using Ptr = std::shared_ptr<SetRingIdsRaysNode>;
	void setParameters(const int* ringIdsRaw, size_t ringIdsCount);

	// Node
	void validateImpl() override;
	void enqueueExecImpl(cudaStream_t stream) override {}

	// Rays description
	std::optional<size_t> getRingIdsCount() const override { return ringIds->getCount(); }

	// Data getters
	std::optional<VArrayProxy<int>::ConstPtr> getRingIds() const override { return ringIds; }

private:
	VArrayProxy<int>::Ptr ringIds = VArrayProxy<int>::create();
};

struct YieldPointsNode : IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<YieldPointsNode>;
	void setParameters(const std::vector<rgl_field_t>& fields);

	// Node
	void enqueueExecImpl(cudaStream_t stream) override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override { return fields; }

	// Data getters
	VArray::ConstPtr getFieldData(rgl_field_t field, cudaStream_t stream) override
	{ return results.at(field); }

private:
	std::vector<rgl_field_t> fields;
	std::unordered_map<rgl_field_t, VArray::ConstPtr> results;
};

struct SpatialMergePointsNode : IPointsNode
{
	using Ptr = std::shared_ptr<SpatialMergePointsNode>;
	void setParameters(const std::vector<rgl_field_t>& fields);

	// Node
	void validateImpl() override;
	void enqueueExecImpl(cudaStream_t stream) override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override
	{ return { std::views::keys(mergedData).begin(), std::views::keys(mergedData).end() }; }

	// Point cloud description
	bool isDense() const override;
	bool hasField(rgl_field_t field) const override { return mergedData.contains(field); }
	std::size_t getWidth() const override { return width; }
	std::size_t getHeight() const override { return 1; }

	// Data getters
	VArray::ConstPtr getFieldData(rgl_field_t field, cudaStream_t stream) override
	{ return std::const_pointer_cast<const VArray>(mergedData.at(field)); }

private:
	std::vector<IPointsNode::Ptr> pointInputs;
	std::unordered_map<rgl_field_t, VArray::Ptr> mergedData;
	std::size_t width = 0;
};

struct TemporalMergePointsNode : IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<YieldPointsNode>;
	void setParameters(const std::vector<rgl_field_t>& fields);

	// Node
	void validateImpl() override;
	void enqueueExecImpl(cudaStream_t stream) override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override
	{ return { std::views::keys(mergedData).begin(), std::views::keys(mergedData).end() }; }

	// Point cloud description
	bool hasField(rgl_field_t field) const override { return mergedData.contains(field); }
	std::size_t getWidth() const override { return width; }

	// Data getters
	VArray::ConstPtr getFieldData(rgl_field_t field, cudaStream_t stream) override
	{ return std::const_pointer_cast<const VArray>(mergedData.at(field)); }

private:
	std::unordered_map<rgl_field_t, VArray::Ptr> mergedData;
	std::size_t width = 0;
};

struct FromArrayPointsNode : IPointsNode, INoInputNode
{
	using Ptr = std::shared_ptr<FromArrayPointsNode>;
	void setParameters(const void* points, size_t pointCount, const std::vector<rgl_field_t>& fields);

	// Node
	void enqueueExecImpl(cudaStream_t stream) override {}

	// Point cloud description
	bool isDense() const override { return false; }
	bool hasField(rgl_field_t field) const override { return fieldData.contains(field); }
	size_t getWidth() const override { return width; }
	size_t getHeight() const override { return 1; }

	// Data getters
	VArray::ConstPtr getFieldData(rgl_field_t field, cudaStream_t stream) override
	{ return std::const_pointer_cast<const VArray>(fieldData.at(field)); }

private:
	GPUFieldDescBuilder gpuFieldDescBuilder;
	std::vector<std::pair<rgl_field_t, void*>> getFieldToPointerMappings(const std::vector<rgl_field_t>& fields);

	std::unordered_map<rgl_field_t, VArray::Ptr> fieldData;
	size_t width = 0;
};

struct GaussianNoiseAngularRaysNode : IRaysNodeSingleInput
{
	using Ptr = std::shared_ptr<GaussianNoiseAngularRaysNode>;

	void setParameters(float mean, float stSev, rgl_axis_t rotationAxis);

	// Node
	void validateImpl() override;
	void enqueueExecImpl(cudaStream_t stream) override;

	// Data getters
	VArrayProxy<Mat3x4f>::ConstPtr getRays() const override { return rays; }

private:
	float mean;
	float stDev;
	rgl_axis_t rotationAxis;
	std::random_device randomDevice;
	Mat3x4f lookAtOriginTransform;

	VArrayProxy<curandStatePhilox4_32_10_t>::Ptr randomizationStates = VArrayProxy<curandStatePhilox4_32_10_t>::create();
	VArrayProxy<Mat3x4f>::Ptr rays = VArrayProxy<Mat3x4f>::create();
};

struct GaussianNoiseAngularHitpointNode : IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<GaussianNoiseAngularHitpointNode>;

	void setParameters(float mean, float stDev, rgl_axis_t rotationAxis);

	// Node
	void validateImpl() override;
	void enqueueExecImpl(cudaStream_t stream) override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override { return {XYZ_F32}; }

	// Data getters
	VArray::ConstPtr getFieldData(rgl_field_t field, cudaStream_t stream) override;

private:
	float mean;
	float stDev;
	rgl_axis_t rotationAxis;
	std::random_device randomDevice;
	Mat3x4f lookAtOriginTransform;

	VArrayProxy<curandStatePhilox4_32_10_t>::Ptr randomizationStates = VArrayProxy<curandStatePhilox4_32_10_t>::create();
	VArrayProxy<Field<XYZ_F32>::type>::Ptr outXyz = VArrayProxy<Field<XYZ_F32>::type>::create();
	VArrayProxy<Field<DISTANCE_F32>::type>::Ptr outDistance = nullptr;
};

struct GaussianNoiseDistanceNode : IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<GaussianNoiseDistanceNode>;

	void setParameters(float mean, float stDevBase, float stDevRisePerMeter);

	// Node
	void validateImpl() override;
	void enqueueExecImpl(cudaStream_t stream) override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override { return {XYZ_F32}; };

	// Data getters
	VArray::ConstPtr getFieldData(rgl_field_t field, cudaStream_t stream) override;

private:
	float mean;
	float stDevBase;
	float stDevRisePerMeter;
	std::random_device randomDevice;
	Mat3x4f lookAtOriginTransform;

	VArrayProxy<curandStatePhilox4_32_10_t>::Ptr randomizationStates = VArrayProxy<curandStatePhilox4_32_10_t>::create();
	VArrayProxy<Field<XYZ_F32>::type>::Ptr outXyz = VArrayProxy<Field<XYZ_F32>::type>::create();
	VArrayProxy<Field<DISTANCE_F32>::type>::Ptr outDistance = nullptr;
};
