#pragma once

#include <vector>
#include <set>
#include <memory>

#include <pipeline/Node.hpp>
#include <pipeline/Interfaces.hpp>
#include <gpu/RaytraceRequestParams.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <typeinfo>

#include <VArray.hpp>
#include <VArrayProxy.hpp>

struct FormatNode : Node, IFormatNode
{
	using Ptr = std::shared_ptr<FormatNode>;

	void validate() override;
	void schedule(cudaStream_t stream);

	inline void setParameters(std::vector<rgl_field_t> fields) { this->fields = std::move(fields); }
	inline std::vector<rgl_field_t> getFieldList() const { return fields; }
	inline bool hasField(rgl_field_t field) const override { return std::find(fields.begin(), fields.end(), field) != fields.end(); }
	inline bool isDense() const override { return input->isDense(); }
	inline size_t getWidth() const override { return input->getWidth(); }
	inline size_t getHeight() const override { return input->getHeight(); }

	std::shared_ptr<const VArray> getData() const override;
	std::size_t getElemSize() const override;

private:
	std::vector<rgl_field_t> fields;
	std::shared_ptr<IPointCloudNode> input;
};

struct RaytraceNode : IPointCloudNode, public Node
{
	using Node::Node;
	using Ptr = std::shared_ptr<RaytraceNode>;

	void validate() override;
	void schedule(cudaStream_t stream) override;

	inline void setParameters(std::shared_ptr<Scene> scene, float range) { this->scene = scene; this->range = range; }
	void setFields(std::set<rgl_field_t> fields) { fields.insert(RGL_FIELD_XYZ_F32); this->fields = std::move(fields); }

	inline bool hasField(rgl_field_t field) const override	{ return fields.contains(field); }
	inline bool isDense() const override { return false; }
	inline size_t getWidth() const override { return raysNode->getRays()->getCount(); }
	inline size_t getHeight() const override { return 1; }  // TODO: implement height in use_rays
	inline std::shared_ptr<const VArray> getFieldData(rgl_field_t field, cudaStream_t stream) const override
	{ return std::const_pointer_cast<const VArray>(fieldData.at(field)); }


private:
	float range;
	std::shared_ptr<Scene> scene;
	std::set<rgl_field_t> fields;
	std::shared_ptr<IRaysNode> raysNode;
	std::shared_ptr<VArrayProxy<RaytraceRequestContext>> requestCtx = VArrayProxy<RaytraceRequestContext>::create(1);
	std::unordered_map<rgl_field_t, std::shared_ptr<VArray>> fieldData;
};

struct UseRaysMat3x4fNode : Node, IRaysNode
{
	using Node::Node;

	void setParameters(const Mat3x4f* raysRaw, size_t rayCount);

	inline std::shared_ptr<const VArrayProxy<Mat3x4f>> getRays() const override { return rays; }

	void validate() override {}
	void schedule(cudaStream_t stream) override {}

private:
	std::shared_ptr<VArrayProxy<Mat3x4f>> rays;
};

struct WritePCDFileNode : Node
{
	using Node::Node;
	using PCLPointType = pcl::PointXYZ;

	inline void setParameters(const char* filePath) { this->filePath = filePath; }
	void validate() override;
	void schedule(cudaStream_t stream) override;
	virtual ~WritePCDFileNode();

private:
	IPointCloudNode::Ptr pclNode;
	FormatNode::Ptr internalFormatNode;
	std::filesystem::path filePath{};
	pcl::PointCloud<PCLPointType> cachedPCLs;
	void execute(cudaStream_t stream);

	friend void streamCallback(cudaStream_t, cudaError_t, void*);
};



struct CompactNode : public Node, IPointCloudNode
{
	// TODO: this should eagerly compute permutation array, but lazily yield varrays

};