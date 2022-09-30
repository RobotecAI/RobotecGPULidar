#include <graph/Nodes.hpp>
#include <scene/Scene.hpp>
#include <macros/optix.hpp>
#include <RGLFields.hpp>

void RaytraceNode::validate()
{
	raysNode = getValidInput<IRaysNode>();

	if (fields.contains(RING_ID_U16) && !raysNode->getRingIds().has_value()) {
		auto msg = fmt::format("requested for field RING_ID_U16, but RaytraceNode cannot get ring ids");
		throw InvalidPipeline(msg);
	}
}

template<rgl_field_t field>
auto RaytraceNode::getPtrTo()
{
	return fields.contains(field) ? fieldData.at(field)->getTypedProxy<typename Field<field>::type>()->getDevicePtr() : nullptr;
}

void RaytraceNode::schedule(cudaStream_t stream)
{
	for (auto&& field : fields) {
		fieldData[field]->resize(raysNode->getRayCount(), false, false);
	}
	auto rays = raysNode->getRays();
	auto sceneAS = scene->getAS();
	auto sceneSBT = scene->getSBT();
	dim3 launchDims = {static_cast<unsigned int>(rays->getCount()), 1, 1};

	// Optional
	auto ringIds = raysNode->getRingIds();

	(*requestCtx)[0] = RaytraceRequestContext{
		.rays = rays->getDevicePtr(),
		.rayCount = rays->getCount(),
		.rayRange = range,
		.ringIds = ringIds.has_value() ? (*ringIds)->getDevicePtr() : nullptr,
		.ringIdsCount = ringIds.has_value() ? (*ringIds)->getCount() : 0,
		.scene = sceneAS,
		.xyz = getPtrTo<XYZ_F32>(),
		.isHit = getPtrTo<IS_HIT_I32>(),
		.rayIdx = getPtrTo<RAY_IDX_U32>(),
		.ringIdx = getPtrTo<RING_ID_U16>(),
		.distanceIdx = getPtrTo<DISTANCE_F32>(),
		.intensityIdx = getPtrTo<INTENSITY_F32>(),
	};

	// TODO(prybicki): VArray may use CUDA managed memory, which hasn't been proven to work with OptiX.
	// TODO(prybicki): Based on my intuition, it should work fine, but beware.
	CUdeviceptr pipelineArgsPtr = requestCtx->getCUdeviceptr();
	std::size_t pipelineArgsSize = requestCtx->getBytesInUse();
	CHECK_OPTIX(optixLaunch(Optix::instance().pipeline, stream, pipelineArgsPtr, pipelineArgsSize, &sceneSBT, launchDims.x, launchDims.y, launchDims.y));
	CHECK_CUDA(cudaStreamSynchronize(stream));
}

void RaytraceNode::setFields(const std::set<rgl_field_t>& fields)
{
	this->fields = std::move(fields);
	for (auto&& field : fields) {
		if (!fieldData.contains(field)) {
			fieldData.insert({field, VArray::create(field)});
			fieldData[field]->hintLocation(VArray::GPU);
		}
	}
}
