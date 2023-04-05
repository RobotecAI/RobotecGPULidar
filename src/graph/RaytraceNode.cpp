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

#include <ranges>
#include <iterator>

#include <graph/NodesCore.hpp>
#include <scene/Scene.hpp>
#include <macros/optix.hpp>
#include <RGLFields.hpp>

void RaytraceNode::validateImpl()
{
	// It should be viewed as a temporary solution. Will change in v14.
	setFields(findFieldsToCompute());

	raysNode = getExactlyOneInputOfType<IRaysNode>();

	if (fieldData.contains(RING_ID_U16) && !raysNode->getRingIds().has_value()) {
		auto msg = fmt::format("requested for field RING_ID_U16, but RaytraceNode cannot get ring ids");
		throw InvalidPipeline(msg);
	}

	if (fieldData.contains(TIME_STAMP_F64) && !scene->getTime().has_value()) {
		auto msg = fmt::format("requested for field TIME_STAMP_F64, but RaytraceNode cannot get time from scene");
		throw InvalidPipeline(msg);
	}
}

template<rgl_field_t field>
auto RaytraceNode::getPtrTo()
{
	return fieldData.contains(field) ? fieldData.at(field)->getTypedProxy<typename Field<field>::type>()->getDevicePtr() : nullptr;
}

void RaytraceNode::enqueueExecImpl(cudaStream_t stream)
{
	for (auto const& [_, data] : fieldData) {
		data->resize(raysNode->getRayCount(), false, false);
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
		.sceneTime = scene->getTime()->asSeconds(),
		.xyz = getPtrTo<XYZ_F32>(),
		.isHit = getPtrTo<IS_HIT_I32>(),
		.rayIdx = getPtrTo<RAY_IDX_U32>(),
		.ringIdx = getPtrTo<RING_ID_U16>(),
		.distance = getPtrTo<DISTANCE_F32>(),
		.intensity = getPtrTo<INTENSITY_F32>(),
		.timestamp = getPtrTo<TIME_STAMP_F64>(),
	};

	CUdeviceptr pipelineArgsPtr = requestCtx->getCUdeviceptr();
	std::size_t pipelineArgsSize = requestCtx->getBytesInUse();
	CHECK_OPTIX(optixLaunch(Optix::getOrCreate().pipeline, stream, pipelineArgsPtr, pipelineArgsSize, &sceneSBT, launchDims.x, launchDims.y, launchDims.y));
	CHECK_CUDA(cudaStreamSynchronize(stream));
}

void RaytraceNode::setFields(const std::set<rgl_field_t>& fields)
{
	auto keyViewer = std::views::keys(fieldData);
	std::set<rgl_field_t> currentFields { keyViewer.begin(), keyViewer.end() };

	std::set<rgl_field_t> toRemove, toInsert;
	std::ranges::set_difference(currentFields, fields, std::inserter(toRemove, toRemove.end()));
	std::ranges::set_difference(fields, currentFields, std::inserter(toInsert, toInsert.end()));

	for (auto&& field : toRemove) {
		fieldData.erase(field);
	}
	for (auto&& field : toInsert) {
		fieldData.insert({field, VArray::create(field)});
	}
}

std::set<rgl_field_t> RaytraceNode::findFieldsToCompute()
{
	std::set<rgl_field_t> outFields;

	// Add primary field
	outFields.insert(XYZ_F32);

	// dfsInputs - if false dfs for outputs
	std::function<void(Node::Ptr, bool)> dfsRet = [&](const Node::Ptr & current, bool dfsInputs) {
		auto dfsNodes = dfsInputs ? current->getInputs() : current->getOutputs();
		for (auto&& node : dfsNodes) {
			if (auto pointNode = std::dynamic_pointer_cast<IPointsNode>(node)) {
				for (auto&& field : pointNode->getRequiredFieldList()) {
					if (!isDummy(field)) {
						outFields.insert(field);
					}
				}
				dfsRet(node, dfsInputs);
			}
		}
	};

	dfsRet(shared_from_this(), true);  // Search in inputs. Needed for SetRingIds only.
	dfsRet(shared_from_this(), false);  // Search in outputs

	return outFields;
}
