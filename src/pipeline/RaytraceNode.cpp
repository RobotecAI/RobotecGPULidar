#include <pipeline/Nodes.hpp>
#include <scene/Scene.hpp>
#include <macros/optix.hpp>
#include <RGLFields.hpp>

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

template<rgl_field_t field>
auto RaytraceNode::getPtrTo()
{
	return fields.contains(field) ? fieldData.at(field)->getTypedProxy<typename RGLField<field>::Type>()->getDevicePtr() : nullptr;
}

void RaytraceNode::schedule(cudaStream_t stream)
{
	auto rays = raysNode->getRays();
	auto sceneAS = scene->getAS();
	auto sceneSBT = scene->getSBT();
	dim3 launchDims = {static_cast<unsigned int>(rays->getCount()), 1, 1};

	(*requestCtx)[0] = RaytraceRequestContext{
		.rays = rays->getDevicePtr(),
		.rayCount = rays->getCount(),
		.rayRange = range,
		.scene = sceneAS,
		.xyz = getPtrTo<RGL_FIELD_XYZ_F32>(),
		.isHit = getPtrTo<RGL_FIELD_IS_HIT_I32>(),
	};

	// TODO(prybicki): VArray may use CUDA managed memory, which hasn't been proven to work with OptiX.
	// TODO(prybicki): Based on my intuition, it should work fine, but beware.
	CUdeviceptr pipelineArgsPtr = requestCtx->getCUdeviceptr();
	std::size_t pipelineArgsSize = requestCtx->getBytesInUse();
	CHECK_OPTIX(optixLaunch(Optix::instance().pipeline, stream, pipelineArgsPtr, pipelineArgsSize, &sceneSBT, launchDims.x, launchDims.y, launchDims.y));
	CHECK_CUDA(cudaStreamSynchronize(stream));
}
