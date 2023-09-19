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
	DeviceAsyncArray<Field<RAY_IDX_U32>::type>::Ptr filteredIndices = DeviceAsyncArray<Field<RAY_IDX_U32>::type>::create(arrayMgr);
	DeviceAsyncArray<pcl::PointXYZL>::Ptr filteredPoints = DeviceAsyncArray<pcl::PointXYZL>::create(arrayMgr);
	mutable CacheManager<rgl_field_t, IAnyArray::Ptr> cacheManager;
	GPUFieldDescBuilder gpuFieldDescBuilder;
};

struct VisualizePointsNode : IPointsNodeSingleInput
{
	static const int FRAME_RATE = 60;
	using Ptr = std::shared_ptr<VisualizePointsNode>;
	using PCLPointType = pcl::PointXYZRGB;
	void setParameters(const char* windowName, int windowWidth, int windowHeight, bool fullscreen);

	// Node
	void validateImpl() override;
	void enqueueExecImpl() override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override;

	void runVisualize();
	virtual ~VisualizePointsNode();

private:
	DeviceAsyncArray<char>::Ptr formattedInputDev = DeviceAsyncArray<char>::create(arrayMgr);
	HostPinnedArray<char>::Ptr formattedInputHst = HostPinnedArray<char>::create();

	PCLVisualizerFix::Ptr viewer;
	std::thread visThread;
	std::mutex updateCloudMutex;
	bool isNewCloud{false};

	std::string windowName{};
	int windowWidth;
	int windowHeight;
	bool fullscreen;
	pcl::PointCloud<PCLPointType>::Ptr cloudPCL{new pcl::PointCloud<PCLPointType>};
	GPUFieldDescBuilder gpuFieldDescBuilder;
};
