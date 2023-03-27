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

void DownSamplePointsNode::onInputChangeImpl()
{
	IPointsNodeSingleInput::onInputChangeImpl();
	if (!input->hasField(XYZ_F32)) {
		auto msg = fmt::format("{} requires XYZ to be present", getName());
		throw InvalidPipeline(msg);
	}
}

void DownSamplePointsNode::executeImpl(cudaStream_t stream)
{
	cacheManager.trigger();

	if (input->getPointCount() == 0) {
		filteredIndices->resize(0, false, false);
		return;
	}

	// Get formatted input data
	FormatPointsNode::formatAsync(inputFmtData, input, getRequiredFieldList(), stream);

	// Downsample
	auto toFilter = std::make_shared<pcl::PointCloud<PCLPoint>>();
	auto filtered = std::make_shared<pcl::PointCloud<PCLPoint>>();
	auto pointCount = input->getPointCount();
	toFilter->reserve(pointCount);
	const PCLPoint* begin = static_cast<const PCLPoint*>(inputFmtData->getReadPtr(MemLoc::Host));
	const PCLPoint* end = begin + pointCount;
	toFilter->assign(begin, end, pointCount);
	for (int i = 0; i < toFilter->size(); ++i) {
		toFilter->points[i].label = i;
	}
	pcl::VoxelGrid<PCLPoint> voxelGrid {};
	voxelGrid.setInputCloud(toFilter);
	voxelGrid.setLeafSize(leafDims.x(), leafDims.y(), leafDims.z());
	voxelGrid.filter(*filtered);
	bool pclReduced = filtered->size() < toFilter->size();
	if (!pclReduced) {
		auto details = fmt::format("original: {}; filtered: {}; leafDims: {}",
		                           toFilter->size(), filtered->size(), leafDims);
		RGL_WARN("Down-sampling node had no effect! ({})", details);
	}
	filteredPoints->setData(filtered->data(), filtered->size());
	filteredIndices->resize(filtered->size(), false, false);

	size_t offset = offsetof(PCLPoint, label);
	size_t stride = sizeof(PCLPoint);
	size_t size = sizeof(PCLPoint::label);
	auto&& dst = (char*) filteredIndices->getDevicePtr();
	auto&& src = (const char*) filteredPoints->getReadPtr(MemLoc::Device);
	gpuCutField(stream, filtered->size(), dst, src, offset, stride, size);
	CHECK_CUDA(cudaEventRecord(finishedEvent, stream));
}

size_t DownSamplePointsNode::getWidth() const
{
	CHECK_CUDA(cudaEventSynchronize(finishedEvent));
	return filteredIndices->getCount();
}

VArray::ConstPtr DownSamplePointsNode::getFieldData(rgl_field_t field, cudaStream_t stream) const
{
	if (!cacheManager.contains(field)) {
		auto fieldData = VArray::create(field, filteredIndices->getCount());
		cacheManager.insert(field, fieldData, true);
	}

	if (!cacheManager.isLatest(field)) {
		auto fieldData = cacheManager.getValue(field);
		fieldData->resize(filteredIndices->getCount(), false, false);
		char* outPtr = static_cast<char *>(fieldData->getWritePtr(MemLoc::Device));
		const char* inputPtr = static_cast<const char *>(input->getFieldData(field, stream)->getReadPtr(MemLoc::Device));
		gpuFilter(stream, filteredIndices->getCount(), filteredIndices->getDevicePtr(), outPtr, inputPtr, getFieldSize(field));
		CHECK_CUDA(cudaStreamSynchronize(stream));
		cacheManager.setUpdated(field);
	}

	return std::const_pointer_cast<const VArray>(cacheManager.getValue(field));
}

std::vector<rgl_field_t> DownSamplePointsNode::getRequiredFieldList() const
{
	// pcl::PointXYZL is aligned to 32 bytes for SSE2 ¯\_(ツ)_/¯
	return {XYZ_F32, PADDING_32, PADDING_32, PADDING_32, PADDING_32, PADDING_32};
}
