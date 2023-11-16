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

#include <graph/NodesCore.hpp>
#include <graph/NodesPcl.hpp>
#include <gpu/nodeKernels.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>

using PCLPoint = pcl::PointXYZL;

void DownSamplePointsNode::validateImpl()
{
	IPointsNodeSingleInput::validateImpl();

	// Needed to clear cache because fields in the pipeline may have changed
	// In fact, the cache manager is no longer useful here
	// To be kept/removed in some future refactor (when resolving comment in the `enqueueExecImpl`)
	cacheManager.clear();
}

void DownSamplePointsNode::enqueueExecImpl()
{
	cacheManager.trigger();

	if (input->getPointCount() == 0) {
		filteredIndices->resize(0, false, false);
		return;
	}

	// Get formatted input data (SSE2-aligned XYZ)
	FormatPointsNode::formatAsync(formattedInput, input, getRequiredFieldList(), gpuFieldDescBuilder);
	formattedInputHst->copyFrom(formattedInput);

	// Copy data into pcl::PointCloud
	auto pointCount = input->getPointCount();
	auto toFilter = std::make_shared<pcl::PointCloud<PCLPoint>>();
	toFilter->reserve(pointCount);
	const PCLPoint* begin = reinterpret_cast<const PCLPoint*>(formattedInputHst->getReadPtr());
	const PCLPoint* end = begin + pointCount;
	toFilter->assign(begin, end, pointCount);

	// Set indices that will help find out which points have been removed
	for (int i = 0; i < toFilter->size(); ++i) {
		toFilter->points[i].label = i;
	}
	pcl::VoxelGrid<PCLPoint> voxelGrid{};
	voxelGrid.setInputCloud(toFilter);
	voxelGrid.setLeafSize(leafDims.x(), leafDims.y(), leafDims.z());

	// Perform filtering
	auto filtered = std::make_shared<pcl::PointCloud<PCLPoint>>();
	voxelGrid.filter(*filtered);

	// Warn if nothing changed
	bool pclReduced = filtered->size() < toFilter->size();
	if (!pclReduced) {
		auto details = fmt::format("original: {}; filtered: {}; leafDims: {}", toFilter->size(), filtered->size(), leafDims);
		RGL_WARN("Down-sampling node had no effect! ({})", details);
	}
	filteredPoints->copyFromExternal(filtered->data(), filtered->size());
	filteredIndices->resize(filtered->size(), false, false);

	size_t offset = offsetof(PCLPoint, label);
	size_t stride = sizeof(PCLPoint);
	size_t size = sizeof(PCLPoint::label);
	auto&& dst = reinterpret_cast<char*>(filteredIndices->getWritePtr());
	auto&& src = reinterpret_cast<const char*>(filteredPoints->getReadPtr());
	gpuCutField(getStreamHandle(), filtered->size(), dst, src, offset, stride, size);

	// getFieldData may be called in client's thread from rgl_graph_get_result_data
	// Doing job there would be:
	// - unexpected (job was supposed to be done asynchronously)
	// - hard to implement:
	//     - to avoid blocking on yet-running graph stream, we would need do it in copy stream, which would require
	//       temporary rebinding DAAs to copy stream, which seems like nightmarish idea
	// Therefore, once we know what fields are requested, we compute them eagerly
	// This is supposed to be removed in some future refactor (e.g. when introducing LayeredSoA)
	for (auto&& field : cacheManager.getKeys()) {
		getFieldData(field);
	}
}

size_t DownSamplePointsNode::getWidth() const
{
	this->synchronize();
	return filteredIndices->getCount();
}

IAnyArray::ConstPtr DownSamplePointsNode::getFieldData(rgl_field_t field)
{
	std::lock_guard lock{getFieldDataMutex};

	if (!cacheManager.contains(field)) {
		auto fieldData = createArray<DeviceAsyncArray>(field, arrayMgr);
		fieldData->resize(filteredIndices->getCount(), false, false);
		cacheManager.insert(field, fieldData, true);
	}

	if (!cacheManager.isLatest(field)) {
		auto fieldData = cacheManager.getValue(field);
		fieldData->resize(filteredIndices->getCount(), false, false);
		char* outPtr = static_cast<char*>(fieldData->getRawWritePtr());
		auto fieldArray = input->getFieldData(field);
		if (!isDeviceAccessible(fieldArray->getMemoryKind())) {
			auto msg = fmt::format("DownSampleNode requires its input to be device-accessible, {} is not", field);
			throw InvalidPipeline(msg);
		}
		const char* inputPtr = static_cast<const char*>(fieldArray->getRawReadPtr());
		gpuFilter(getStreamHandle(), filteredIndices->getCount(), filteredIndices->getReadPtr(), outPtr, inputPtr,
		          getFieldSize(field));
		CHECK_CUDA(cudaStreamSynchronize(getStreamHandle()));
		cacheManager.setUpdated(field);
	}

	return cacheManager.getValue(field);
}

std::vector<rgl_field_t> DownSamplePointsNode::getRequiredFieldList() const
{
	// pcl::PointXYZL is aligned to 32 bytes for SSE2 ¯\_(ツ)_/¯
	return {XYZ_VEC3_F32, PADDING_32, PADDING_32, PADDING_32, PADDING_32, PADDING_32};
}
