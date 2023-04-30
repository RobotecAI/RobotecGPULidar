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

	DownSamplePointsNode() { CHECK_CUDA(cudaEventCreateWithFlags(&finishedEvent, cudaEventDisableTiming)); }
	virtual ~DownSamplePointsNode() { CHECK_CUDA_NO_THROW(cudaEventDestroy(finishedEvent)); }

	// Node
	void validateImpl() override;
	void enqueueExecImpl(cudaStream_t stream) override;

	// Node requirements
	std::vector<rgl_field_t> getRequiredFieldList() const override;

	// Point cloud description
	bool isDense() const override { return false; }
	size_t getWidth() const override;
	size_t getHeight() const override { return 1; }

	// Data getters
	VArray::ConstPtr getFieldData(rgl_field_t field, cudaStream_t stream) override;



private:
	Vec3f leafDims;
	VArray::Ptr inputFmtData = VArray::create<char>();
	cudaEvent_t finishedEvent = nullptr;
	VArrayProxy<Field<RAY_IDX_U32>::type>::Ptr filteredIndices = VArrayProxy<Field<RAY_IDX_U32>::type>::create();
	VArray::Ptr filteredPoints = VArray::create<pcl::PointXYZL>();
	mutable CacheManager<rgl_field_t, VArray::Ptr> cacheManager;
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
	void enqueueExecImpl(cudaStream_t stream) override;

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
	GPUFieldDescBuilder gpuFieldDescBuilder;
};
