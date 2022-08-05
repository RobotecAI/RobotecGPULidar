#include <Lidar.hpp>

#include <macros/cuda.hpp>
#include <macros/optix.hpp>

API_OBJECT_INSTANCE(Lidar);

Lidar::Lidar(TransformMatrix *rayPoses, int rayPosesCount) : range(std::numeric_limits<float>::max())
{
	CHECK_CUDA(cudaStreamCreate(&stream));

	if (rayPosesCount <= 0) {
		auto msg = fmt::format("LidarContext::LidarContext: rayPosesCount ({}) must be > 0", rayPosesCount);
		throw std::logic_error(msg);
	}

	dRayPoses.copyFromHost(rayPoses, rayPosesCount);

	// It should be possible to have no ring-ids,
	// because user may be not interested in having them  in the output format, however,
	// current formatting hardcodes their usage; therefore, a default dummy value is set
	// TODO: remove this once formatting code is improved
	std::vector<int> ringIds = {0};
	setRingIds(ringIds.data(), ringIds.size());

	dUnityVisualisationPoints.resizeToFit(rayPosesCount);
	dRosXYZ.resizeToFit(rayPosesCount);
	dDensePoint3f.resizeToFit(rayPosesCount);
	dDensePCL12.resizeToFit(rayPosesCount);
	dDensePCL24.resizeToFit(rayPosesCount);
	dDensePCL48.resizeToFit(rayPosesCount);
	dWasHit.resizeToFit(rayPosesCount);
	dHitsBeforeIndex.resizeToFit(rayPosesCount);

	dRandomizationStates.resizeToFit(rayPosesCount);
	setupGaussianNoiseGenerator(randomDevice(), stream, dRandomizationStates);
}

void Lidar::setRingIds(int *ringIds, size_t ringIdsCount)
{
	if (ringIdsCount <= 0) {
		auto msg = fmt::format("LidarContext::LidarContext: lidarArrayRingCount ({}) must be > 0", ringIdsCount);
		throw std::logic_error(msg);
	}
	dLidarArrayRingIds.copyFromHost(ringIds, ringIdsCount);
}

void Lidar::scheduleRaycast(std::shared_ptr<Scene> scene)
{
	densePointCount.reset();
	if (scene->getObjectCount() == 0) {
            RGL_WARN("Requested raytracing on an empty scene");
		densePointCount = 0;
		return;
	}
	auto sceneAS = scene->getAS();
	auto sceneSBT = scene->getSBT();
	currentJob = LaunchLidarParams{
		.range = range,
		.rayCount = dRayPoses.getElemCount(),
		.lidarPose = lidarPose,
		.rosTransform = rosTransform,
		.traversable = sceneAS,
		.lidarNoiseParams = lidarNoiseParams,
		.dRayPoses = dRayPoses.readDevice(),
		.dUnityVisualisationPoints = dUnityVisualisationPoints.writeDevice(),
		.dRosXYZ = dRosXYZ.writeDevice(),
		.dWasHit = dWasHit.writeDevice(),
	};
	dCurrentJob.copyFromHostAsync(&currentJob.value(), 1, stream);

	CUdeviceptr pipelineArgsPtr = dCurrentJob.readDeviceRaw();
	size_t pipelineArgsSize = dCurrentJob.getByteSize();
	dim3 launchDims = {static_cast<unsigned int>(currentJob->rayCount), 1, 1};

	addGaussianNoise(stream, dRayPoses, lidarNoiseParams, dRandomizationStates, dRayPoses);
	CHECK_OPTIX(optixLaunch(Optix::instance().pipeline, stream, pipelineArgsPtr, pipelineArgsSize, &sceneSBT, launchDims.x, launchDims.y, launchDims.y));

	Point3f lidar_origin_position = {lidarPose[3], lidarPose[7], lidarPose[11]};
	addGaussianNoise(stream, dRosXYZ, dUnityVisualisationPoints, lidar_origin_position, lidarNoiseParams,
	                 dRandomizationStates, dRosXYZ, dUnityVisualisationPoints);

	formatPCLsAsync(stream, dWasHit, dHitsBeforeIndex, dRosXYZ, dUnityVisualisationPoints, dLidarArrayRingIds,
	                dDensePoint3f, dDensePCL12, dDensePCL24, dDensePCL48);

	hDensePoint3f.copyFromDeviceAsync(dDensePoint3f, stream);
	hDensePCL12.copyFromDeviceAsync(dDensePCL12, stream);
	hDensePCL24.copyFromDeviceAsync(dDensePCL24, stream);
	hDensePCL48.copyFromDeviceAsync(dDensePCL48, stream);
}

int Lidar::getResultsSize()
{
	if (!densePointCount.has_value()) {
		densePointCount = -1;
		// TODO: move this to scheduleRaycast
		CHECK_CUDA(cudaMemcpyAsync(&densePointCount.value(),
		                       dHitsBeforeIndex.readDevice() + dHitsBeforeIndex.getElemCount() - 1,
		                       sizeof(densePointCount.value()),
		                       cudaMemcpyDeviceToHost,
		                       stream));
		CHECK_CUDA(cudaStreamSynchronize(stream));
	}
	return densePointCount.value();
}

void Lidar::getResults(int format, void *data)
{
	CHECK_CUDA(cudaStreamSynchronize(stream));
	// densePointCount.has_value() guarantees that the stream has been synchronized
	if (!densePointCount.has_value()) {
		throw std::logic_error("getResults() has been called without a prior call to getResultsSize()");
	}

	if (densePointCount == 0) {
		RGL_WARN("Returning an empty pointcloud");
		return;
	}

	if (format == RGL_FORMAT_XYZ) {
		memcpy(data, hDensePoint3f.readHost(), hDensePoint3f.getByteSize());
		return;
	}
	if (format == RGL_FORMAT_E2E_PCL12) {
		memcpy(data, hDensePCL12.readHost(), hDensePCL12.getByteSize());
		return;
	}
	if (format == RGL_FORMAT_E2E_PCL24) {
		memcpy(data, hDensePCL24.readHost(), hDensePCL24.getByteSize());
		return;
	}
	if (format == RGL_FORMAT_E2E_PCL48) {
		memcpy(data, hDensePCL48.readHost(), hDensePCL48.getByteSize());
		return;
	}
}
