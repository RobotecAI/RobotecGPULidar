#pragma once

#include <vector>
#include <set>
#include <memory>

#include <pipeline/Node.hpp>
#include <pipeline/Interfaces.hpp>
#include <gpu/RaytraceRequestContext.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <gpu/GPUFieldDesc.hpp>
#include <typeinfo>

#include <VArray.hpp>
#include <VArrayProxy.hpp>
#include <gpu/nodeKernels.hpp>

/**
 * Notes for maintainers:
 *
 * Some nodes define extra methods such as get*Count() to obtain the number of elements in their output buffers.
 * This is purposeful: interface-level methods are guaranteed to return correct number of elements or throw,
 * while buffers sizes are not reliable, since they can be resized in execute().
 *
 * For methods taking cudaStream as an argument, it is legal to return VArray that becomes valid only after stream
 * operations prior to the return are finished.
 */


struct CompactNode : public Node, IPointCloudNode
{
	using Ptr = std::shared_ptr<CompactNode>;

	inline void setParameters() {}
	inline bool hasField(rgl_field_t field) const override	{ return input->hasField(field); }
	inline bool isDense() const override { return true; }
	inline size_t getHeight() const override { return 1; }

	size_t getWidth() const override;
	VArray::ConstPtr getFieldData(rgl_field_t field, cudaStream_t stream) const override;
	void validate(cudaStream_t stream) override;
	void schedule(cudaStream_t stream) override;
private:
	size_t width;
	cudaStream_t stream;
	IPointCloudNode::Ptr input;
	VArrayProxy<CompactionIndexType>::Ptr inclusivePrefixSum = VArrayProxy<CompactionIndexType>::create();
};

struct FormatNode : Node, IFormatNode
{
	using Ptr = std::shared_ptr<FormatNode>;

	void validate(cudaStream_t stream) override;
	void schedule(cudaStream_t stream) override;

	inline void setParameters(const std::vector<rgl_field_t>& fields) { this->fields = fields;}
	inline std::vector<rgl_field_t> getFieldList() const { return fields; }
	inline bool hasField(rgl_field_t field) const override { return std::find(fields.begin(), fields.end(), field) != fields.end(); }
	inline bool isDense() const override { return input->isDense(); }
	inline size_t getWidth() const override { return input->getWidth(); }
	inline size_t getHeight() const override { return input->getHeight(); }

	std::size_t getPointSize() const override;
	inline VArray::ConstPtr getData() const override { return output; }

private:
	std::vector<rgl_field_t> fields;
	IPointCloudNode::Ptr input;
	VArray::Ptr output = output = VArray::create<char>();
};

struct RaytraceNode : IPointCloudNode, public Node
{
	using Ptr = std::shared_ptr<RaytraceNode>;

	void validate(cudaStream_t stream) override;
	void schedule(cudaStream_t stream) override;

	inline void setParameters(std::shared_ptr<Scene> scene, float range) { this->scene = scene; this->range = range; }
	void setFields(const std::set<rgl_field_t>& fields) { this->fields = std::move(fields); }

	inline bool hasField(rgl_field_t field) const override	{ return fields.contains(field); }
	inline bool isDense() const override { return false; }
	inline size_t getWidth() const override { return raysNode->getRays()->getCount(); }
	inline size_t getHeight() const override { return 1; }  // TODO: implement height in use_rays
	inline VArray::ConstPtr getFieldData(rgl_field_t field, cudaStream_t stream) const override
	{ return std::const_pointer_cast<const VArray>(fieldData.at(field)); }


private:
	float range;
	std::shared_ptr<Scene> scene;
	std::set<rgl_field_t> fields;
	IRaysNode::Ptr raysNode;
	VArrayProxy<RaytraceRequestContext>::Ptr requestCtx = VArrayProxy<RaytraceRequestContext>::create(1);
	std::unordered_map<rgl_field_t, VArray::Ptr> fieldData;

	template<rgl_field_t>
	auto getPtrTo();
};

struct TransformRaysNode : Node, IRaysNode
{
	inline void setParameters(Mat3x4f transform) { this->transform = transform; }
	inline VArrayProxy<Mat3x4f>::ConstPtr getRays() const override { return rays; }
	inline size_t getRayCount() const override { return input->getRayCount(); }

	void validate(cudaStream_t stream) override;
	void schedule(cudaStream_t stream) override;

private:
	Mat3x4f transform;
	IRaysNode::Ptr input;
	VArrayProxy<Mat3x4f>::Ptr rays = VArrayProxy<Mat3x4f>::create();
};

struct UseRaysMat3x4fNode : Node, IRaysNode
{
	using Node::Node;

	void setParameters(const Mat3x4f* raysRaw, size_t rayCount);

	void validate(cudaStream_t stream) override;
	void schedule(cudaStream_t stream) override {}

	inline VArrayProxy<Mat3x4f>::ConstPtr getRays() const override { return rays; }
	inline size_t getRayCount() const override { return rays->getCount(); }

private:
	VArrayProxy<Mat3x4f>::Ptr rays = VArrayProxy<Mat3x4f>::create();
};

struct WritePCDFileNode : Node
{
	using Node::Node;
	using PCLPointType = pcl::PointXYZ;

	inline void setParameters(const char* filePath) { this->filePath = filePath; }
	void validate(cudaStream_t stream) override;
	void schedule(cudaStream_t stream) override;
	virtual ~WritePCDFileNode();

private:
	IFormatNode::Ptr input;
	std::filesystem::path filePath{};
	pcl::PointCloud<PCLPointType> cachedPCLs;
};
