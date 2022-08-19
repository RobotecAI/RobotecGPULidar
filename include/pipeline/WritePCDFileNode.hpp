#pragma once

#include <macros/cuda.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class WritePCDFileNode;

static void streamCallback(cudaStream_t stream, cudaError_t status, void* data);

struct WritePCDFileNode : Node
{
	using Node::Node;
	using PCLPointType = pcl::PointXYZ;
	using RGLPointType = Vec4f;
	static constexpr rgl_field_t RGL_PCD_FIELD = RGL_FIELD_XYZP_F32;
	static_assert(sizeof(PCLPointType) == sizeof(RGLPointType));

	void setParameters(const char* filePath)
	{
		this->filePath = filePath;
	}

	void validate() override
	{
		pclNode = getValidInput<IPointcloudNode>();
		if (!pclNode->hasField(RGL_PCD_FIELD)) {
			auto msg = fmt::format("{} requires XYZP format which was not provided by {}",
			                       name(typeid(*this)), name(typeid(*pclNode)));
			throw InvalidPipeline(msg);
		}
	}

	void schedule(cudaStream_t stream) override
	{
		pclNode->getFieldData(RGL_PCD_FIELD)->getTypedProxy<RGLPointType>()->hintLocation(VArray::CPU);
		CHECK_CUDA(cudaStreamAddCallback(stream, streamCallback, this, 0));
	}

	void execute(cudaStream_t stream)
	{
		VArrayProxy<RGLPointType>::ConstPtr data = pclNode->getFieldData(RGL_PCD_FIELD)->getTypedProxy<RGLPointType>();
		const auto* dataPtr = reinterpret_cast<const PCLPointType*>(data->getHostPtr());
		pcl::PointCloud<PCLPointType> cloud;
		cloud.resize(pclNode->getWidth(), pclNode->getHeight());
		cloud.assign(dataPtr, dataPtr + data->getCount(), pclNode->getWidth());
		cloud.is_dense = pclNode->isDense();
		cachedPCLs += cloud;
	}

	virtual ~WritePCDFileNode()
	{
		pcl::io::savePCDFileASCII(filePath.string(), cachedPCLs);
	}

private:
	IPointcloudNode::Ptr pclNode;
	std::filesystem::path filePath{};

	pcl::PointCloud<PCLPointType> cachedPCLs;
};

static void streamCallback(cudaStream_t stream, cudaError_t error, void* data)
{
	CHECK_CUDA(error);
	reinterpret_cast<WritePCDFileNode*>(data)->execute(stream);
}
