// Copyright 2023 Robotec.AI
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

#include <graph/NodesCore.hpp>
#include <graph/NodesPcl.hpp>
#include <gpu/nodeKernels.hpp>

#include <pcl/impl/point_types.hpp>

void RemoveGroundPointsNode::setParameters(rgl_axis_t sensorUpAxis, float groundAngleThreshold, float groundDistanceThreshold,
                                           float groundFilterDistance)
{
	this->groundFilterDistance = groundFilterDistance;

	planeCoefficients = pcl::ModelCoefficients(); // Reset coefficients (they are optimized every run)

	// Setup segmentation options
	segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
	segmentation.setAxis(Eigen::Vector3f(sensorUpAxis == RGL_AXIS_X ? 1.0 : 0.0, sensorUpAxis == RGL_AXIS_Y ? 1.0 : 0.0,
	                                     sensorUpAxis == RGL_AXIS_Z ? 1.0 : 0.0));
	segmentation.setEpsAngle(groundAngleThreshold);
	segmentation.setOptimizeCoefficients(true);
	segmentation.setMethodType(pcl::SAC_RANSAC);
	segmentation.setDistanceThreshold(groundDistanceThreshold);
	segmentation.setInputCloud(toFilterPointCloud);
	constexpr int maxIterations = 500;
	segmentation.setMaxIterations(maxIterations);
}

void RemoveGroundPointsNode::validateImpl()
{
	IPointsNodeSingleInput::validateImpl();

	// Needed to clear cache because fields in the pipeline may have changed
	// In fact, the cache manager is no longer useful here
	// To be kept/removed in some future refactor (when resolving comment in the `enqueueExecImpl`)
	cacheManager.clear();
}

void RemoveGroundPointsNode::enqueueExecImpl()
{
	cacheManager.trigger();

	if (input->getPointCount() == 0) {
		filteredIndices->resize(0, false, false);
		return;
	}

	// Get formatted input data (SSE2-aligned XYZ)
	FormatPointsNode::formatAsync(formattedInput, input, getRequiredFieldList(), gpuFieldDescBuilder);
	formattedInputHost->copyFrom(formattedInput);

	// Copy data into pcl::PointCloud
	auto pointCount = input->getPointCount();
	toFilterPointCloud->resize(pointCount);
	const pcl::PointXYZ* begin = reinterpret_cast<const pcl::PointXYZ*>(formattedInputHost->getReadPtr());
	const pcl::PointXYZ* end = begin + pointCount;
	toFilterPointCloud->assign(begin, end, pointCount);

	// Segment ground and approximate plane coefficients
	segmentation.segment(*groundIndices, planeCoefficients);

	// Select ground indices (points within given distance to approximate plane model). Can be optimized (GPU?)
	auto planeCoefficientsEigen = Eigen::Vector4f(planeCoefficients.values[0], planeCoefficients.values[1],
	                                              planeCoefficients.values[2], planeCoefficients.values[3]);
	auto planeModel = pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZ>(toFilterPointCloud);
	planeModel.selectWithinDistance(planeCoefficientsEigen, groundFilterDistance, groundIndices->indices);

	// Compute non-ground indices. Can be optimized (GPU?)
	if (groundIndices->indices.empty()) {
		filteredIndicesHost.resize(pointCount);
		std::iota(filteredIndicesHost.begin(), filteredIndicesHost.end(), 0);
		filteredIndices->resize(0, false, false);
	} else {
		filteredIndicesHost.resize(pointCount - groundIndices->indices.size());
		int currentGroundIdx = 0;
		int currentNonGroundIdx = 0;
		for (int i = 0; i < pointCount; ++i) {
			if (i == groundIndices->indices[currentGroundIdx]) {
				++currentGroundIdx;
				continue;
			}
			filteredIndicesHost[currentNonGroundIdx] = i;
			++currentNonGroundIdx;
		}
	}

	filteredIndices->copyFromExternal(filteredIndicesHost.data(), filteredIndicesHost.size());

	// getFieldData may be called in client's thread from rgl_graph_get_result_data
	// Doing job there would be:
	// - unexpected (job was supposed to be done asynchronously)
	// - hard to implement:
	//     - to avoid blocking on yet-running graph stream, we would need do it in copy stream, which would require
	//       temporary rebinding DAAs to copy stream, which seems like nightmarish idea
	// Therefore, once we know what fields are requested, we compute them eagerly
	// This is supposed to be removed in some future refactor (e.g. when introducing LayeredSoA)
	for (auto&& field : cacheManager.getKeys()) {
		getFieldData(field);
	}
}

size_t RemoveGroundPointsNode::getWidth() const
{
	this->synchronize();
	return filteredIndices->getCount();
}

IAnyArray::ConstPtr RemoveGroundPointsNode::getFieldData(rgl_field_t field)
{
	std::lock_guard lock{getFieldDataMutex};

	if (!cacheManager.contains(field)) {
		auto fieldData = createArray<DeviceAsyncArray>(field, arrayMgr);
		fieldData->resize(filteredIndices->getCount(), false, false);
		cacheManager.insert(field, fieldData, true);
	}

	if (!cacheManager.isLatest(field)) {
		auto fieldData = cacheManager.getValue(field);
		fieldData->resize(filteredIndices->getCount(), false, false);
		char* outPtr = static_cast<char*>(fieldData->getRawWritePtr());
		auto fieldArray = input->getFieldData(field);
		if (!isDeviceAccessible(fieldArray->getMemoryKind())) {
			auto msg = fmt::format("RemoveGround requires its input to be device-accessible, {} is not", field);
			throw InvalidPipeline(msg);
		}
		const char* inputPtr = static_cast<const char*>(fieldArray->getRawReadPtr());
		gpuFilter(getStreamHandle(), filteredIndices->getCount(), filteredIndices->getReadPtr(), outPtr, inputPtr,
		          getFieldSize(field));
		CHECK_CUDA(cudaStreamSynchronize(getStreamHandle()));
		cacheManager.setUpdated(field);
	}

	return cacheManager.getValue(field);
}

std::vector<rgl_field_t> RemoveGroundPointsNode::getRequiredFieldList() const
{
	// SSE2-aligned XYZ
	return {XYZ_VEC3_F32, PADDING_32};
}
