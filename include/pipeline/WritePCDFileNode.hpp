#pragma once

#include <macros/cuda.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class WritePCDFileNode;

static void streamCallback(cudaStream_t stream, cudaError_t status, void* data);

struct WritePCDFileNode : Node
{
	using Node::Node;

	void setParameters(const char* filePath)
	{
		this->filePath = filePath;
	}

	void validate() override
	{
		pclNode = getValidInput<IPointcloudNode>();
		if (!pclNode->hasField(RGL_FIELD_XYZP_F32)) {
			auto msg = fmt::format("{} requires XYZP format which was not provided by {}",
			                       name(typeid(*this)), name(typeid(*pclNode)));
			throw InvalidPipeline(msg);
		}
	}

	void schedule(cudaStream_t stream) override
	{
		CHECK_CUDA(cudaStreamAddCallback(stream, streamCallback, this, 0));
	}

	void execute()
	{
		// for (int i = 0; i < pclNode->getWidth(); ++i) {
		// 	auto vec = (*pclNode->getFieldData(RGL_FIELD_XYZP_F32)->getTypedProxy<Vec4f>())[i];
		// 	if (vec.length() > 0) {
		// 		RGL_INFO("Hello! [{}] = {}", i, vec);
		// 	}
		// }
		pcl::PointCloud<pcl::PointXYZ> cloud;
		cloud.height   = 1;
		cloud.is_dense = true;
		auto proxy = (*pclNode->getFieldData(RGL_FIELD_XYZP_F32)->getTypedProxy<Vec4f>());
		for (int i = 0; i < pclNode->getWidth(); ++i) {
			auto v = proxy[i];
			cloud.push_back({v.x(), v.y(), v.z()});
		}
		pcl::io::savePCDFileASCII(filePath.string(), cloud);
	}

private:
	IPointcloudNode::Ptr pclNode;
	std::filesystem::path filePath{};
};

static void streamCallback(cudaStream_t stream, cudaError_t error, void* data)
{
	CHECK_CUDA(error);
	reinterpret_cast<WritePCDFileNode*>(data)->execute();
}
