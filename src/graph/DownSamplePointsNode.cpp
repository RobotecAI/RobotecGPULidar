#include <graph/Nodes.hpp>
#include <gpu/nodeKernels.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>

using PCLPoint = pcl::PointXYZL;

void DownSamplePointsNode::validate()
{
	if (finishedEvent == nullptr) {
		CHECK_CUDA(cudaEventCreateWithFlags(&finishedEvent, cudaEventDisableTiming));
	}
	input = getValidInput<IPointsNode>();
}

void DownSamplePointsNode::schedule(cudaStream_t stream)
{
	cacheManager.trigger();

	if (input->getPointCount() == 0) {
		filteredIndices->resize(0, false, false);
		return;
	}

	// Get formatted input data
	FormatPointsNode::formatAsync(inputFmtData, input, requiredFields, stream);

	// Downsample
	auto toFilter = std::make_shared<pcl::PointCloud<PCLPoint>>();
	auto filtered = std::make_shared<pcl::PointCloud<PCLPoint>>();
	auto pointCount = input->getPointCount();
	toFilter->reserve(pointCount);
	const PCLPoint* begin = static_cast<const PCLPoint*>(inputFmtData->getReadPtr(MemLoc::host()));
	const PCLPoint* end = begin + pointCount;
	toFilter->assign(begin, end, pointCount);
	for (int i = 0; i < toFilter->size(); ++i) {
		toFilter->points[i].label = i;
	}
	inputFmtData->hintLocation(VArray::GPU);
	pcl::VoxelGrid<PCLPoint> voxelGrid {};
	voxelGrid.setInputCloud(toFilter);
	voxelGrid.setLeafSize(leafDims.x(), leafDims.y(), leafDims.z());
	voxelGrid.filter(*filtered);
	RGL_WARN("Original: {} Filtered: {}", toFilter->size(), filtered->size());
	filteredPoints->copyFrom(filtered->data(), filtered->size());
	filteredIndices->resize(filtered->size(), false, false);

	size_t offset = offsetof(PCLPoint, label);
	size_t stride = sizeof(PCLPoint);
	size_t size = sizeof(PCLPoint::label);
	auto&& dst = (char*) filteredIndices->getDevicePtr();
	auto&& src = (const char*) filteredPoints->getReadPtr(MemLoc::device());
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
		char* outPtr = static_cast<char *>(fieldData->getWritePtr(MemLoc::device()));
		const char* inputPtr = static_cast<const char *>(input->getFieldData(field, stream)->getReadPtr(MemLoc::device()));
		gpuFilter(stream, filteredIndices->getCount(), filteredIndices->getDevicePtr(), outPtr, inputPtr, getFieldSize(field));
		CHECK_CUDA(cudaStreamSynchronize(stream));
		cacheManager.setUpdated(field);
	}

	return std::const_pointer_cast<const VArray>(cacheManager.getValue(field));
}
