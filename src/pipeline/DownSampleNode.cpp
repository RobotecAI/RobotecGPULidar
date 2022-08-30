#include <pipeline/Nodes.hpp>
#include <gpu/nodeKernels.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>

using PCLPoint = pcl::PointXYZL;

void DownSampleNode::validate()
{
	static const std::vector<rgl_field_t> fmtFields = {
		// pcl::PointXYZL is SSE-aligned to 32 bytes ¯\_(ツ)_/¯
		XYZ_F32, PADDING_32,
		PADDING_32, PADDING_32, PADDING_32, PADDING_32
	};
	if (finishedEvent == nullptr) {
		CHECK_CUDA(cudaEventCreateWithFlags(&finishedEvent, cudaEventDisableTiming));
	}
	if (internalFmt == nullptr) {
		internalFmt = Node::create<FormatNode>();
		internalFmt->setParameters(fmtFields);
		prependNode(internalFmt);
	}
	internalFmt->validate();
	if (internalFmt->getPointSize() != sizeof(PCLPoint)) {
		auto msg = fmt::format("DownSampleNode: RGL / PCL point sizes mismatch: {} {}",
		                       internalFmt->getPointSize(), sizeof(PCLPoint));
		throw std::logic_error(msg);
	}
	input = getValidInputFrom<IPointCloudNode>(internalFmt->getInputs());
}

void DownSampleNode::schedule(cudaStream_t stream)
{
	internalFmt->schedule(stream);
	CHECK_CUDA(cudaStreamSynchronize(stream));
	internalFmt->getData()->hintLocation(VArray::CPU);
	auto toFilter = std::make_shared<pcl::PointCloud<PCLPoint>>();
	auto filtered = std::make_shared<pcl::PointCloud<PCLPoint>>();
	auto pointCount = input->getPointCount();
	toFilter->reserve(pointCount);
	PCLPoint* begin = static_cast<PCLPoint*>(internalFmt->getData()->getHostPtr());
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
