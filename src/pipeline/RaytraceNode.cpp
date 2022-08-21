#include <pipeline/Nodes.hpp>
#include <scene/Scene.hpp>
#include <macros/optix.hpp>

void RaytraceNode::validate(cudaStream_t stream)
{
	raysNode = getValidInput<IRaysNode>();
	for (auto&& field : fields) {
		if (!fieldData.contains(field)) {
			fieldData.insert({field, VArray::create(field)});
		}
		fieldData[field]->resize(raysNode->getRayCount(), false, false);
		fieldData[field]->hintLocation(VArray::GPU, stream);
	}
}


void RaytraceNode::schedule(cudaStream_t stream)
{
	auto rays = raysNode->getRays();
	auto sceneAS = scene->getAS();
	auto sceneSBT = scene->getSBT();
	dim3 launchDims = {static_cast<unsigned int>(rays->getCount()), 1, 1};

	(*requestCtx)[0] = {
	.rays = rays->getDevicePtr(),
	.rayCount = rays->getCount(),
	.rayOriginToWorld = Mat3x4f::identity(),
	.rayRange = range,
	.scene = sceneAS,
	.xyz = fields.contains(RGL_FIELD_XYZ_F32) ? fieldData.at(RGL_FIELD_XYZ_F32)->getTypedProxy<Vec3f>()->getDevicePtr() : nullptr,
	};

	// TODO(prybicki): VArray may use CUDA managed memory, which hasn't been proven to work with OptiX.
	// TODO(prybicki): Based on my intuition, it should work fine, but beware.
	CUdeviceptr pipelineArgsPtr = requestCtx->getCUdeviceptr();
	std::size_t pipelineArgsSize = requestCtx->getBytesInUse();
	CHECK_OPTIX(optixLaunch(Optix::instance().pipeline, stream, pipelineArgsPtr, pipelineArgsSize, &sceneSBT, launchDims.x, launchDims.y, launchDims.y));
	CHECK_CUDA(cudaStreamSynchronize(stream));
}
