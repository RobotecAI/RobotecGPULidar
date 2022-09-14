#include <graph/Nodes.hpp>
#include <gpu/nodeKernels.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>

using PCLPoint = pcl::PointXYZL;

void DownSampleNode::validate()
{
	if (finishedEvent == nullptr) {
		CHECK_CUDA(cudaEventCreateWithFlags(&finishedEvent, cudaEventDisableTiming));
	}
	input = getValidInput<IPointCloudNode>();
}

void DownSampleNode::schedule(cudaStream_t stream)
{
	// Get formatted input data
	std::size_t pointSize = getPointSize(requiredFields);
	std::size_t pointCount = input->getPointCount();
	VArray::Ptr fmtInputData = VArray::create<char>(pointCount * pointSize);
	auto gpuFields = getGPUFields(requiredFields, input, stream);
	char* fmtInputDataPtr = static_cast<char*>(fmtInputData->getDevicePtr());
	gpuFormat(stream, pointCount, pointSize, requiredFields.size(), gpuFields->getDevicePtr(), fmtInputDataPtr);
	CHECK_CUDA(cudaStreamSynchronize(stream));

	// Downsample
	fmtInputData->hintLocation(VArray::CPU);
	auto toFilter = std::make_shared<pcl::PointCloud<PCLPoint>>();
	auto filtered = std::make_shared<pcl::PointCloud<PCLPoint>>();
	toFilter->reserve(pointCount);
	PCLPoint* begin = static_cast<PCLPoint*>(fmtInputData->getHostPtr());
	PCLPoint* end = begin + pointCount;
	toFilter->assign(begin, end, pointCount);
	for (int i = 0; i < toFilter->size(); ++i) {
		toFilter->points[i].label = i;
	}
	pcl::VoxelGrid<PCLPoint> voxelGrid {};
	voxelGrid.setInputCloud(toFilter);
	voxelGrid.setLeafSize(leafDims.x(), leafDims.y(), leafDims.z());
	voxelGrid.filter(*filtered);
	RGL_WARN("Original: {} Filtered: {}", toFilter->size(), filtered->size());
	filteredPoints->copyFrom(filtered->data(), filtered->size());
	filteredIndices->resize(filtered->size());

	size_t offset = offsetof(PCLPoint, label);
	size_t stride = sizeof(PCLPoint);
	size_t size = sizeof(PCLPoint::label);
	auto&& dst = (char*) filteredIndices->getDevicePtr();
	auto&& src = (const char*) filteredPoints->getDevicePtr();
	gpuCutField(stream, filtered->size(), dst, src, offset, stride, size);
	CHECK_CUDA(cudaEventRecord(finishedEvent, stream));
}

size_t DownSampleNode::getWidth() const
{
	CHECK_CUDA(cudaEventSynchronize(finishedEvent));
	return filteredIndices->getCount();
}

VArray::ConstPtr DownSampleNode::getFieldData(rgl_field_t field, cudaStream_t stream) const
{
	auto&& inData = input->getFieldData(field, stream);
	VArray::Ptr outData = VArray::create(field, filteredIndices->getCount());
	gpuFilter(stream, filteredIndices->getCount(), filteredIndices->getDevicePtr(), (char*) outData->getDevicePtr(), (const char*) inData->getDevicePtr(), getFieldSize(field));
	CHECK_CUDA(cudaStreamSynchronize(stream));
	return outData;
}
