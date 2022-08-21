#pragma once

#include <vector>
#include <set>
#include <memory>

#include <pipeline/Node.hpp>
#include <pipeline/Interfaces.hpp>
#include <gpu/RaytraceRequestParams.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <gpu/GPUFieldDesc.hpp>
#include <typeinfo>

#include <VArray.hpp>
#include <VArrayProxy.hpp>

/**
 * Note: some nodes define extra methods such as get*Count() to obtain the number of elements in their output buffers.
 * This is purposeful: interface-level methods are guaranteed to return correct number of elements or throw,
 * while buffers sizes are not reliable, since they can be resized in execute().
 */

struct FormatNode : Node, IFormatNode
{
	using Ptr = std::shared_ptr<FormatNode>;

	void validate(cudaStream_t stream) override;
	void schedule(cudaStream_t stream);

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
	using Node::Node;
	using Ptr = std::shared_ptr<RaytraceNode>;

	void validate(cudaStream_t stream) override;
	void schedule(cudaStream_t stream) override;

	inline void setParameters(std::shared_ptr<Scene> scene, float range) { this->scene = scene; this->range = range; }
	void setFields(std::set<rgl_field_t> fields) { fields.insert(RGL_FIELD_XYZ_F32); this->fields = std::move(fields); }

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
};

struct TransformRaysNode : Node, IRaysNode
{
	VArrayProxy<Mat3x4f>::ConstPtr getRays() const override
	{
		return VArrayProxy<Mat3x4f>::ConstPtr();
	}

	size_t getRayCount() const override { return 0; }

	void validate(cudaStream_t stream) override;
	void schedule(cudaStream_t stream) override;

private:
	IRaysNode::Ptr input;
	VArrayProxy<Mat3x4f>::ConstPtr rays = VArrayProxy<Mat3x4f>::create();
};

struct UseRaysMat3x4fNode : Node, IRaysNode
{
	using Node::Node;

	void setParameters(const Mat3x4f* raysRaw, size_t rayCount);

	inline VArrayProxy<Mat3x4f>::ConstPtr getRays() const override { return rays; }
	inline size_t getRayCount() const override { return rays->getCount(); }

	void validate(cudaStream_t stream) override;
	void schedule(cudaStream_t stream) override {}

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
	void execute(cudaStream_t stream);

	friend void streamCallback(cudaStream_t, cudaError_t, void*);
};



struct CompactNode : public Node, IPointCloudNode
{
	// TODO: this should eagerly compute permutation array, but lazily yield varrays

};