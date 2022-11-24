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

#include <graph/Nodes.hpp>
#include <scene/Scene.hpp>
#include <macros/optix.hpp>
#include <RGLFields.hpp>

void RaytraceNode::validate()
{
	raysNode = getValidInput<IRaysNode>();

	if (fields.count(RING_ID_U16) > 0 && !raysNode->getRingIds().has_value()) {
		auto msg = fmt::format("requested for field RING_ID_U16, but RaytraceNode cannot get ring ids");
		throw InvalidPipeline(msg);
	}
}

template<rgl_field_t field>
auto RaytraceNode::getPtrTo()
{
	return fields.count(field) > 0 ? fieldData.at(field)->getTypedProxy<typename Field<field>::type>()->getDevicePtr() : nullptr;
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

	RaytraceRequestContext rrc{};
	rrc.rays = rays->getDevicePtr();
	rrc.rayCount = rays->getCount();
	rrc.rayRange = range;
	rrc.ringIds = ringIds.has_value() ? (*ringIds)->getDevicePtr() : nullptr;
	rrc.ringIdsCount = ringIds.has_value() ? (*ringIds)->getCount() : 0;
	rrc.scene = sceneAS;
	rrc.xyz = getPtrTo<XYZ_F32>();
	rrc.isHit = getPtrTo<IS_HIT_I32>();
	rrc.rayIdx = getPtrTo<RAY_IDX_U32>();
	rrc.ringIdx = getPtrTo<RING_ID_U16>();
	rrc.distance = getPtrTo<DISTANCE_F32>();
	rrc.intensity = getPtrTo<INTENSITY_F32>();
	(*requestCtx)[0] = rrc;

	CUdeviceptr pipelineArgsPtr = requestCtx->getCUdeviceptr();
	std::size_t pipelineArgsSize = requestCtx->getBytesInUse();
	CHECK_OPTIX(optixLaunch(Optix::instance().pipeline, stream, pipelineArgsPtr, pipelineArgsSize, &sceneSBT, launchDims.x, launchDims.y, launchDims.y));
	CHECK_CUDA(cudaStreamSynchronize(stream));
}

void RaytraceNode::setFields(const std::set<rgl_field_t>& fields)
{
	this->fields = std::move(fields);
	for (auto&& field : fields) {
		if (fieldData.count(field) == 0) {
			fieldData.insert({field, VArray::create(field)});
		}
	}
}
