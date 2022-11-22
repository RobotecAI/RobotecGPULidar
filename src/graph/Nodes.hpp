// Copyright 2022 Robotec.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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

#include <CacheManager.hpp>
#include <VArray.hpp>
#include <VArrayProxy.hpp>
#include <gpu/nodeKernels.hpp>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>

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

struct FormatPointsNode : Node, IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<FormatPointsNode>;
	void setParameters(const std::vector<rgl_field_t>& fields);

	// Node
	void validate() override;
	void schedule(cudaStream_t stream) override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override { return fields; }

	// Point cloud description
	bool hasField(rgl_field_t field) const override { return std::find(fields.begin(), fields.end(), field) != fields.end(); }

	// Data getters
	VArray::ConstPtr getFieldData(rgl_field_t field, cudaStream_t stream) const override;
	std::size_t getFieldPointSize(rgl_field_t field) const override;

	// Actual implementation of formatting made public for other nodes
	static void formatAsync(const VArray::Ptr& output, const IPointsNode::Ptr& input,
	                        const std::vector<rgl_field_t>& fields, cudaStream_t stream);

private:
	std::vector<rgl_field_t> fields;
	VArray::Ptr output = VArray::create<char>();
};

struct CompactPointsNode : Node, IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<CompactPointsNode>;
	void setParameters() {}

	// Node
	void validate() override;
	void schedule(cudaStream_t stream) override;

	// Point cloud description
	bool isDense() const override { return true; }
	size_t getWidth() const override;
	size_t getHeight() const override { return 1; }

	// Data getters
	VArray::ConstPtr getFieldData(rgl_field_t field, cudaStream_t stream) const override;

private:
	size_t width;
	cudaEvent_t finishedEvent = nullptr;
	VArrayProxy<CompactionIndexType>::Ptr inclusivePrefixSum = VArrayProxy<CompactionIndexType>::create();
	mutable CacheManager<rgl_field_t, VArray::Ptr> cacheManager;
};

struct DownSamplePointsNode : Node, IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<DownSamplePointsNode>;
	void setParameters(Vec3f leafDims) { this->leafDims = leafDims; }

	// Node
	void validate() override;
	void schedule(cudaStream_t stream) override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override;

	// Point cloud description
	bool isDense() const override { return false; }
	size_t getWidth() const override;
	size_t getHeight() const override { return 1; }

	// Data getters
	VArray::ConstPtr getFieldData(rgl_field_t field, cudaStream_t stream) const override;

private:
	Vec3f leafDims;
	VArray::Ptr inputFmtData = VArray::create<char>();
	cudaEvent_t finishedEvent = nullptr;
	VArrayProxy<Field<RAY_IDX_U32>::type>::Ptr filteredIndices = VArrayProxy<Field<RAY_IDX_U32>::type>::create();
	VArray::Ptr filteredPoints = VArray::create<pcl::PointXYZL>();
	mutable CacheManager<rgl_field_t, VArray::Ptr> cacheManager;
};

struct RaytraceNode : Node, IPointsNode
{
	using Ptr = std::shared_ptr<RaytraceNode>;
	void setParameters(std::shared_ptr<Scene> scene, float range) { this->scene = scene; this->range = range; }

	// Node
	void validate() override;
	void schedule(cudaStream_t stream) override;

	// Point cloud description
	bool isDense() const override { return false; }
	bool hasField(rgl_field_t field) const override { return fields.contains(field); }
	size_t getWidth() const override { return raysNode->getRays()->getCount(); }
	size_t getHeight() const override { return 1; }  // TODO: implement height in use_rays

	// Data getters
	VArray::ConstPtr getFieldData(rgl_field_t field, cudaStream_t stream) const override
	{ return std::const_pointer_cast<const VArray>(fieldData.at(field)); }


	void setFields(const std::set<rgl_field_t>& fields);
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

struct TransformPointsNode : Node, IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<TransformPointsNode>;
	void setParameters(Mat3x4f transform) { this->transform = transform; }

	// Node
	void validate() override;
	void schedule(cudaStream_t stream) override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override;

	// Data getters
	VArray::ConstPtr getFieldData(rgl_field_t field, cudaStream_t stream) const override;

private:
	Mat3x4f transform;
	VArrayProxy<Field<XYZ_F32>::type>::Ptr output = VArrayProxy<Field<XYZ_F32>::type>::create();
};

struct TransformRaysNode : Node, IRaysNodeSingleInput
{
	using Ptr = std::shared_ptr<TransformRaysNode>;
	void setParameters(Mat3x4f transform) { this->transform = transform; }

	// Node
	void validate() override;
	void schedule(cudaStream_t stream) override;

	// Data getters
	VArrayProxy<Mat3x4f>::ConstPtr getRays() const override { return rays; }

private:
	Mat3x4f transform;
	VArrayProxy<Mat3x4f>::Ptr rays = VArrayProxy<Mat3x4f>::create();
};

struct FromMat3x4fRaysNode : Node, IRaysNode
{
	using Ptr = std::shared_ptr<FromMat3x4fRaysNode>;
	void setParameters(const Mat3x4f* raysRaw, size_t rayCount);

	// Node
	void validate() override;
	void schedule(cudaStream_t stream) override {}

	// Rays description
	size_t getRayCount() const override { return rays->getCount(); }
	std::optional<size_t> getRingIdsCount() const override { return std::nullopt; }

	// Data getters
	VArrayProxy<Mat3x4f>::ConstPtr getRays() const override { return rays; }
	std::optional<VArrayProxy<int>::ConstPtr> getRingIds() const override { return std::nullopt; }

private:
	VArrayProxy<Mat3x4f>::Ptr rays = VArrayProxy<Mat3x4f>::create();
};

struct SetRingIdsRaysNode : Node, IRaysNodeSingleInput
{
	using Ptr = std::shared_ptr<SetRingIdsRaysNode>;
	void setParameters(const int* ringIdsRaw, size_t ringIdsCount);

	// Node
	void validate() override;
	void schedule(cudaStream_t stream) override {}

	// Rays description
	std::optional<size_t> getRingIdsCount() const override { return ringIds->getCount(); }

	// Data getters
	std::optional<VArrayProxy<int>::ConstPtr> getRingIds() const override { return ringIds; }

private:
	VArrayProxy<int>::Ptr ringIds = VArrayProxy<int>::create();
};

struct WritePCDFilePointsNode : Node, IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<WritePCDFilePointsNode>;
	using PCLPointType = pcl::PointXYZ;
	void setParameters(const char* filePath) { this->filePath = filePath; }

	// Node
	void validate() override;
	void schedule(cudaStream_t stream) override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override;

	virtual ~WritePCDFilePointsNode();

private:
	VArray::Ptr inputFmtData = VArray::create<char>();
	std::filesystem::path filePath{};
	pcl::PointCloud<PCLPointType> cachedPCLs;
};

struct YieldPointsNode : Node, IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<YieldPointsNode>;
	void setParameters(const std::vector<rgl_field_t>& fields);

	// Node
	void validate() override;
	void schedule(cudaStream_t stream) override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override { return fields; }

	// Data getters
	VArray::ConstPtr getFieldData(rgl_field_t field, cudaStream_t stream) const override
	{ return results.at(field); }

private:
	std::vector<rgl_field_t> fields;
	std::unordered_map<rgl_field_t, VArray::ConstPtr> results;
};

struct VisualizePointsNode : Node, IPointsNodeSingleInput
{
	static const int FRAME_RATE = 60;
	using Ptr = std::shared_ptr<VisualizePointsNode>;
	using PCLPointType = pcl::PointXYZRGB;
	void setParameters(const char* windowName, int windowWidth, int windowHeight, bool fullscreen);

	// Node
	void validate() override;
	void schedule(cudaStream_t stream) override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override;

	void runVisualize();
	virtual ~VisualizePointsNode();

private:
	VArray::Ptr inputFmtData = VArray::create<char>();

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

class RglRos2Node : public rclcpp::Node
{
public:
    using SharedPtr = std::shared_ptr<RglRos2Node>;

    RglRos2Node(std::string topicName);
	void constructRos2Msg(std::vector<rgl_field_t> fields);
    void publish(const void* rawData, int count);

  private:
	sensor_msgs::msg::PointField createPointFieldMsg(std::string name, int offset, int datatype, int count);

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    sensor_msgs::msg::PointCloud2 msg_;
};

struct Ros2PublishPointsNode : Node, IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<Ros2PublishPointsNode>;

	void setParameters(const char* topicName, const char* frameId);

	// Node
	void validate() override;
	void schedule(cudaStream_t stream) override;

private:
	VArray::Ptr inputFmtData = VArray::create<char>();

	std::string topicName{};
	std::string frameId{};

	RglRos2Node::SharedPtr ros2Node;
};
