#pragma once

#include <vector>
#include <set>
#include <memory>
#include <thread>

#include <graph/Node.hpp>
#include <graph/Interfaces.hpp>
#include <gpu/RaytraceRequestContext.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <graph/PCLVisualizerFix.hpp>
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

// TODO(prybicki): Consider templatizing IPointCloudNode with its InputInterface type.
// TODO(prybicki): This would implement automatic getValidInput() and method forwarding.

struct FormatPointsNode : Node, IPointcloudDescription
{
	using Ptr = std::shared_ptr<FormatPointsNode>;

	void validate() override;
	void schedule(cudaStream_t stream) override;
	void setParameters(const std::vector<rgl_field_t>& fields);

	template<typename T>
	static VArray::Ptr formatAsync(IPointsNode::Ptr input, const std::vector<rgl_field_t>& fields, cudaStream_t stream);

	inline VArray::ConstPtr getData() const { return output; }
	inline size_t getFormattedPointSize() const { return getPointSize(fields); }

	inline std::vector<rgl_field_t> getRequiredFieldList() const override { return fields; }
	inline bool hasField(rgl_field_t field) const override { return std::find(fields.begin(), fields.end(), field) != fields.end(); }
	inline bool isDense() const override { return input->isDense(); }
	inline size_t getWidth() const override { return input->getWidth(); }
	inline size_t getHeight() const override { return input->getHeight(); }


private:
	std::vector<rgl_field_t> fields;
	IPointsNode::Ptr input;
	VArray::Ptr output = VArray::create<char>();
};

struct CompactPointsNode : Node, IPointsNode
{
	using Ptr = std::shared_ptr<CompactPointsNode>;

	void validate() override;
	void schedule(cudaStream_t stream) override;
	size_t getWidth() const override;
	VArray::ConstPtr getFieldData(rgl_field_t field, cudaStream_t stream) const override;

	inline void setParameters() {}
	inline std::vector<rgl_field_t> getRequiredFieldList() const override { return {}; }
	inline bool hasField(rgl_field_t field) const override { return input->hasField(field); }
	inline bool isDense() const override { return true; }
	inline size_t getHeight() const override { return 1; }

private:
	size_t width;
	IPointsNode::Ptr input;
	cudaEvent_t finishedEvent = nullptr;
	VArrayProxy<CompactionIndexType>::Ptr inclusivePrefixSum = VArrayProxy<CompactionIndexType>::create();
};

struct DownSamplePointsNode : Node, IPointsNode
{
	using Ptr = std::shared_ptr<DownSamplePointsNode>;

	void validate() override;
	void schedule(cudaStream_t stream) override;
	size_t getWidth() const override;
	VArray::ConstPtr getFieldData(rgl_field_t field, cudaStream_t stream) const override;

	inline void setParameters(Vec3f leafDims) { this->leafDims = leafDims; }
	inline std::vector<rgl_field_t> getRequiredFieldList() const override { return requiredFields; }
	inline bool hasField(rgl_field_t field) const override { return input->hasField(field); }
	inline bool isDense() const override { return false; }
	inline size_t getHeight() const override { return 1; }

private:
	Vec3f leafDims;
	std::vector<rgl_field_t> requiredFields
	{XYZ_F32, PADDING_32, PADDING_32, PADDING_32, PADDING_32, PADDING_32}; // pcl::PointXYZL is SSE-aligned to 32 bytes ¯\_(ツ)_/¯
	IPointsNode::Ptr input;
	cudaEvent_t finishedEvent = nullptr;
	VArrayProxy<Field<RAY_IDX_U32>::type>::Ptr filteredIndices = VArrayProxy<Field<RAY_IDX_U32>::type>::create();
	VArray::Ptr filteredPoints = VArray::create<pcl::PointXYZL>();
};


struct RaytraceNode : Node, IPointsNode
{
	using Ptr = std::shared_ptr<RaytraceNode>;

	void validate() override;
	void schedule(cudaStream_t stream) override;
	void setFields(const std::set<rgl_field_t>& fields);

	inline void setParameters(std::shared_ptr<Scene> scene, float range) { this->scene = scene; this->range = range; }
	inline bool hasField(rgl_field_t field) const override	{ return fields.contains(field); }
	inline std::vector<rgl_field_t> getRequiredFieldList() const override { return {}; }
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

struct TransformPointsNode : Node, IPointsNode
{
	using Ptr = std::shared_ptr<TransformPointsNode>;

	void validate() override;
	void schedule(cudaStream_t stream) override;
	VArray::ConstPtr getFieldData(rgl_field_t field, cudaStream_t stream) const override;

	inline void setParameters(Mat3x4f transform) { this->transform = transform; }
	inline std::vector<rgl_field_t> getRequiredFieldList() const override { return requiredFields; }
	inline bool hasField(rgl_field_t field) const override { return input->hasField(field); }
	inline bool isDense() const override { return input->isDense(); }
	inline size_t getWidth() const override { return input->getWidth(); }
	inline size_t getHeight() const override { return input->getHeight(); }


private:
	Mat3x4f transform;
	std::vector<rgl_field_t> requiredFields{XYZ_F32};
	IPointsNode::Ptr input;
	VArrayProxy<Field<XYZ_F32>::type>::Ptr output = VArrayProxy<Field<XYZ_F32>::type>::create();
};

struct TransformRaysNode : Node, IRaysNode
{
	using Ptr = std::shared_ptr<TransformRaysNode>;

	void validate() override;
	void schedule(cudaStream_t stream) override;

	inline std::vector<rgl_field_t> getRequiredFieldList() const override { return {}; }
	inline void setParameters(Mat3x4f transform) { this->transform = transform; }
	inline VArrayProxy<Mat3x4f>::ConstPtr getRays() const override { return rays; }
	inline size_t getRayCount() const override { return input->getRayCount(); }
	inline std::optional<VArrayProxy<int>::ConstPtr> getRingIds() const override { return input->getRingIds(); }
	inline std::optional<size_t> getRingIdsCount() const override { return input->getRingIdsCount(); }

private:
	Mat3x4f transform;
	IRaysNode::Ptr input;
	VArrayProxy<Mat3x4f>::Ptr rays = VArrayProxy<Mat3x4f>::create();
};

struct FromMat3x4fRaysNode : Node, IRaysNode
{
	using Ptr = std::shared_ptr<FromMat3x4fRaysNode>;

	void setParameters(const Mat3x4f* raysRaw, size_t rayCount);
	void validate() override;

	inline std::vector<rgl_field_t> getRequiredFieldList() const override { return {}; }
	inline void schedule(cudaStream_t stream) override {}
	inline VArrayProxy<Mat3x4f>::ConstPtr getRays() const override { return rays; }
	inline size_t getRayCount() const override { return rays->getCount(); }
	inline std::optional<VArrayProxy<int>::ConstPtr> getRingIds() const override { return std::nullopt; }
	inline std::optional<size_t> getRingIdsCount() const override { return std::nullopt; }

private:
	VArrayProxy<Mat3x4f>::Ptr rays = VArrayProxy<Mat3x4f>::create();
};

struct SetRingIdsRaysNode : Node, IRaysNode
{
	using Ptr = std::shared_ptr<SetRingIdsRaysNode>;

	void setParameters(const int* ringIdsRaw, size_t ringIdsCount);
	void validate() override;

	inline std::vector<rgl_field_t> getRequiredFieldList() const override { return {}; }
	inline void schedule(cudaStream_t stream) override {}
	inline VArrayProxy<Mat3x4f>::ConstPtr getRays() const override { return input->getRays(); }
	inline size_t getRayCount() const override { return input->getRayCount(); }
	inline std::optional<VArrayProxy<int>::ConstPtr> getRingIds() const override { return ringIds; }
	inline std::optional<size_t> getRingIdsCount() const override { return ringIds->getCount(); }

private:
	IRaysNode::Ptr input;
	VArrayProxy<int>::Ptr ringIds = VArrayProxy<int>::create();
};

struct WritePCDFilePointsNode : Node
{
	using Ptr = std::shared_ptr<WritePCDFilePointsNode>;
	using PCLPointType = pcl::PointXYZ;

	void validate() override;
	void schedule(cudaStream_t stream) override;
	virtual ~WritePCDFilePointsNode();

	inline std::vector<rgl_field_t> getRequiredFieldList() const override { return requiredFields; }

	inline void setParameters(const char* filePath) { this->filePath = filePath; }

private:
	std::vector<rgl_field_t> requiredFields{XYZ_F32, PADDING_32};
	IPointsNode::Ptr input;
	std::filesystem::path filePath{};
	pcl::PointCloud<PCLPointType> cachedPCLs;
};

struct YieldPointsNode : Node
{
	void schedule(cudaStream_t stream) override;
	void setParameters(const std::vector<rgl_field_t>& fields);

	inline std::vector<rgl_field_t> getRequiredFieldList() const override { return fields; }
	inline void validate() override { input = getValidInput<IPointsNode>(); }

private:
	IPointsNode::Ptr input;
	std::vector<rgl_field_t> fields;
	std::unordered_map<rgl_field_t, VArray::ConstPtr> results;
};

struct VisualizeNode : Node
{
	using Ptr = std::shared_ptr<VisualizeNode>;
	using PCLPointType = pcl::PointXYZRGB;

	static const int FRAME_RATE = 60;

	void validate() override;
	void schedule(cudaStream_t stream) override;
	void runVisualize();
	virtual ~VisualizeNode();

	void setParameters(const char* windowName, int windowWidth, int windowHeight, bool fullscreen);
	inline std::vector<rgl_field_t> getRequiredFieldList() const override { return requiredFields; }


private:
	std::vector<rgl_field_t> requiredFields
	{XYZ_F32, PADDING_32, PADDING_32, PADDING_32, PADDING_32, PADDING_32};
	IPointsNode::Ptr input;

	PCLVisualizerFix::Ptr viewer;
	std::thread visThread;
	std::mutex updateCloudMutex;
	bool isNewCloud{false};

	std::string windowName{};
	int windowWidth;
	int windowHeight;
	bool fullscreen;
	pcl::PointCloud<PCLPointType>::Ptr cloudPCL{new pcl::PointCloud<PCLPointType>};
};
