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


struct CompactNode : Node, IPointCloudNode
{
	using Ptr = std::shared_ptr<CompactNode>;

	void validate() override;
	void schedule(cudaStream_t stream) override;
	size_t getWidth() const override;
	VArray::ConstPtr getFieldData(rgl_field_t field, cudaStream_t stream) const override;

	inline void setParameters() {}
	inline bool hasField(rgl_field_t field) const override { return input->hasField(field); }
	inline bool isDense() const override { return true; }
	inline size_t getHeight() const override { return 1; }

private:
	size_t width;
	IPointCloudNode::Ptr input;
	cudaEvent_t finishedEvent = nullptr;
	VArrayProxy<CompactionIndexType>::Ptr inclusivePrefixSum = VArrayProxy<CompactionIndexType>::create();
};

struct DownSampleNode : Node, IPointCloudNode
{
	using Ptr = std::shared_ptr<DownSampleNode>;

	void validate() override;
	void schedule(cudaStream_t stream) override;
	size_t getWidth() const override;
	VArray::ConstPtr getFieldData(rgl_field_t field, cudaStream_t stream) const override;

	inline void setParameters(Vec3f leafDims) { this->leafDims = leafDims; }
	inline bool hasField(rgl_field_t field) const override	{ return input->hasField(field); }
	inline bool isDense() const override { return false; }
	inline size_t getHeight() const override { return 1; }

private:
	Vec3f leafDims;
	IPointCloudNode::Ptr input;
};

struct FormatNode : Node, IFormatNode
{
	using Ptr = std::shared_ptr<FormatNode>;

	void validate() override;
	void schedule(cudaStream_t stream) override;
	std::size_t getPointSize() const override;

	inline void setParameters(const std::vector<rgl_field_t>& fields) { this->fields = fields;}
	inline std::vector<rgl_field_t> getFieldList() const { return fields; }
	inline bool hasField(rgl_field_t field) const override { return std::find(fields.begin(), fields.end(), field) != fields.end(); }
	inline bool isDense() const override { return input->isDense(); }
	inline size_t getWidth() const override { return input->getWidth(); }
	inline size_t getHeight() const override { return input->getHeight(); }
	inline VArray::ConstPtr getData() const override { return output; }


private:
	std::vector<rgl_field_t> fields;
	IPointCloudNode::Ptr input;
	VArray::Ptr output = output = VArray::create<char>();
};

struct RaytraceNode : Node, IPointCloudNode
{
	using Ptr = std::shared_ptr<RaytraceNode>;

	void validate() override;
	void schedule(cudaStream_t stream) override;
	void setFields(const std::set<rgl_field_t>& fields);

	inline void setParameters(std::shared_ptr<Scene> scene, float range) { this->scene = scene; this->range = range; }
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

struct TransformPointsNode : Node, IPointCloudNode
{
	using Ptr = std::shared_ptr<TransformPointsNode>;

	void validate() override;
	void schedule(cudaStream_t stream) override;
	VArray::ConstPtr getFieldData(rgl_field_t field, cudaStream_t stream) const override;

	inline void setParameters(Mat3x4f transform) { this->transform = transform; }
	inline bool hasField(rgl_field_t field) const override { return input->hasField(field); }
	inline bool isDense() const override { return input->isDense(); }
	inline size_t getWidth() const override { return input->getWidth(); }
	inline size_t getHeight() const override { return input->getHeight(); }


private:
	Mat3x4f transform;
	IPointCloudNode::Ptr input;
	VArrayProxy<Field<XYZ_F32>::type>::Ptr output = VArrayProxy<Field<XYZ_F32>::type>::create();
};

struct TransformRaysNode : Node, IRaysNode
{
	using Ptr = std::shared_ptr<TransformRaysNode>;

	void validate() override;
	void schedule(cudaStream_t stream) override;

	inline void setParameters(Mat3x4f transform) { this->transform = transform; }
	inline VArrayProxy<Mat3x4f>::ConstPtr getRays() const override { return rays; }
	inline size_t getRayCount() const override { return input->getRayCount(); }

private:
	Mat3x4f transform;
	IRaysNode::Ptr input;
	VArrayProxy<Mat3x4f>::Ptr rays = VArrayProxy<Mat3x4f>::create();
};

struct UseRaysMat3x4fNode : Node, IRaysNode
{
	using Ptr = std::shared_ptr<UseRaysMat3x4fNode>;

	void setParameters(const Mat3x4f* raysRaw, size_t rayCount);
	void validate() override;

	inline void schedule(cudaStream_t stream) override {}
	inline VArrayProxy<Mat3x4f>::ConstPtr getRays() const override { return rays; }
	inline size_t getRayCount() const override { return rays->getCount(); }

private:
	VArrayProxy<Mat3x4f>::Ptr rays = VArrayProxy<Mat3x4f>::create();
};

struct WritePCDFileNode : Node
{
	using Ptr = std::shared_ptr<WritePCDFileNode>;
	using PCLPointType = pcl::PointXYZ;

	void validate() override;
	void schedule(cudaStream_t stream) override;
	virtual ~WritePCDFileNode();

	inline void setParameters(const char* filePath) { this->filePath = filePath; }

private:
	FormatNode::Ptr internalFmt {};
	std::filesystem::path filePath{};
	pcl::PointCloud<PCLPointType> cachedPCLs;
};
