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

#include <thread>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <graph/Node.hpp>
#include <graph/Interfaces.hpp>
#include <graph/PCLVisualizerFix.hpp>
#include <CacheManager.hpp>

struct DownSamplePointsNode : IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<DownSamplePointsNode>;
	void setParameters(Vec3f leafDims) { this->leafDims = leafDims; }

	// Node
	void validateImpl() override;
	void enqueueExecImpl() override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override;

	// Point cloud description
	bool isDense() const override { return false; }
	size_t getWidth() const override;
	size_t getHeight() const override { return 1; }

	// Data getters
	IAnyArray::ConstPtr getFieldData(rgl_field_t field) override;

private:
	Vec3f leafDims;
	DeviceAsyncArray<char>::Ptr formattedInput = DeviceAsyncArray<char>::create(arrayMgr);
	HostPinnedArray<char>::Ptr formattedInputHst = HostPinnedArray<char>::create();
	DeviceAsyncArray<Field<RAY_IDX_U32>::type>::Ptr filteredIndices = DeviceAsyncArray<Field<RAY_IDX_U32>::type>::create(
	    arrayMgr);
	DeviceAsyncArray<pcl::PointXYZL>::Ptr filteredPoints = DeviceAsyncArray<pcl::PointXYZL>::create(arrayMgr);
	mutable CacheManager<rgl_field_t, IAnyArray::Ptr> cacheManager;
	std::mutex getFieldDataMutex;
	GPUFieldDescBuilder gpuFieldDescBuilder;
};

struct RemoveGroundPointsNode : IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<RemoveGroundPointsNode>;
	void setParameters(rgl_axis_t sensorUpAxis, float groundAngleThreshold, float groundDistanceThreshold);

	// Node
	void validateImpl() override;
	void enqueueExecImpl() override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override;

	// Point cloud description
	size_t getWidth() const override;
	size_t getHeight() const override { return 1; }

	// Data getters
	IAnyArray::ConstPtr getFieldData(rgl_field_t field) override;

private:
	// Data containers
	std::vector<Field<RAY_IDX_U32>::type> filteredIndicesHost;
	DeviceAsyncArray<Field<RAY_IDX_U32>::type>::Ptr filteredIndices = DeviceAsyncArray<Field<RAY_IDX_U32>::type>::create(
	    arrayMgr);
	DeviceAsyncArray<char>::Ptr formattedInput = DeviceAsyncArray<char>::create(arrayMgr);
	HostPinnedArray<char>::Ptr formattedInputHost = HostPinnedArray<char>::create();

	// PCL members
	pcl::PointCloud<pcl::PointXYZ>::Ptr toFilterPointCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	pcl::PointIndices::Ptr groundIndices = std::make_shared<pcl::PointIndices>();
	pcl::ModelCoefficients planeCoefficients;
	pcl::SACSegmentation<pcl::PointXYZ> segmentation;

	// RGL related members
	GPUFieldDescBuilder gpuFieldDescBuilder;
	std::mutex getFieldDataMutex;
	mutable CacheManager<rgl_field_t, IAnyArray::Ptr> cacheManager;
};

struct VisualizePointsNode : IPointsNodeSingleInput
{
	static const int FRAME_RATE = 60;
	using Ptr = std::shared_ptr<VisualizePointsNode>;
	using PCLPointType = pcl::PointXYZRGB;
	void setParameters(const char* windowName, int windowWidth, int windowHeight, bool fullscreen);

	// Node
	void enqueueExecImpl() override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override;

	virtual ~VisualizePointsNode();

private:
	DeviceAsyncArray<char>::Ptr formattedInputDev = DeviceAsyncArray<char>::create(arrayMgr);
	HostPinnedArray<char>::Ptr formattedInputHst = HostPinnedArray<char>::create();
	GPUFieldDescBuilder gpuFieldDescBuilder;

	PCLVisualizerFix::Ptr viewer;
	std::string windowName{};
	int windowWidth;
	int windowHeight;
	bool fullscreen;

	std::atomic<bool> eraseRequested{false};
	std::atomic<bool> isClosed{false};

	std::atomic<bool> hasNewPointCloud{false};
	std::mutex updateCloudMutex;
	pcl::PointCloud<PCLPointType>::Ptr cloudPCL{new pcl::PointCloud<PCLPointType>};

	struct VisualizeThread
	{
		void runVisualize();
		VisualizeThread() { thread = std::thread(&VisualizeThread::runVisualize, this); }
		~VisualizeThread()
		{
			// This might be called when the main thread is doing exit
			{
				// Request removing all nodes
				std::lock_guard lock{visualizeNodesMutex};
				for (auto&& node : visualizeNodes) {
					node->eraseRequested = true;
				}
			}
			shouldQuit = true;
			thread.join();
		}

		std::thread thread;
		std::atomic<bool> shouldQuit{false};
		std::mutex visualizeNodesMutex;

		// Adding: client thread
		// Removing: visualize thread
		std::list<VisualizePointsNode::Ptr> visualizeNodes;
	};

	inline static std::optional<VisualizeThread> visualizeThread;
};
