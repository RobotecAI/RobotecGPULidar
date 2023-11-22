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

void RaytraceNode::setParameters()
{
	const static Vec2f defaultRangeValue = Vec2f(0.0f, FLT_MAX);
	defaultRange->copyFromExternal(&defaultRangeValue, 1);
}

void RaytraceNode::validateImpl()
{
	// It should be viewed as a temporary solution. Will change in v14.
	setFields(findFieldsToCompute());

	raysNode = getExactlyOneInputOfType<IRaysNode>();

	if (fieldData.contains(RING_ID_U16) && !raysNode->getRingIds().has_value()) {
		auto msg = fmt::format("requested for field RING_ID_U16, but RaytraceNode cannot get ring ids");
		throw InvalidPipeline(msg);
	}

	if (fieldData.contains(TIME_STAMP_F64) && !Scene::instance().getTime().has_value()) {
		auto msg = fmt::format("requested for field TIME_STAMP_F64, but RaytraceNode cannot get time from scene");
		throw InvalidPipeline(msg);
	}

	if (doApplyDistortion && !raysNode->getTimeOffsets().has_value()) {
		auto msg = fmt::format("requested for raytrace with velocity distortion, but RaytraceNode cannot get time offsets");
		throw InvalidPipeline(msg);
	}
}

template<rgl_field_t field>
auto RaytraceNode::getPtrTo()
{
	return fieldData.contains(field) ? fieldData.at(field)
	                                       ->asTyped<typename Field<field>::type>()
	                                       ->template asSubclass<DeviceAsyncArray>()
	                                       ->getWritePtr() :
	                                   nullptr;
}

void RaytraceNode::enqueueExecImpl()
{
	for (auto const& [_, data] : fieldData) {
		data->resize(raysNode->getRayCount(), false, false);
	}

	// Even though we are in graph thread here, we can access Scene class (see comment there)
	const Mat3x4f* raysPtr = raysNode->getRays()->asSubclass<DeviceAsyncArray>()->getReadPtr();
	auto sceneAS = Scene::instance().getASLocked();
	auto sceneSBT = Scene::instance().getSBTLocked();
	dim3 launchDims = {static_cast<unsigned int>(raysNode->getRayCount()), 1, 1};

	// Optional
	auto rayRanges = raysNode->getRanges();
	auto ringIds = raysNode->getRingIds();
	auto timeOffsets = raysNode->getTimeOffsets();

	// Note: requestCtx is a HostPinnedArray just for convenience (Host meme accessible from GPU), may be optimized.
	requestCtxHst->resize(1, true, false);
	requestCtxHst->at(0) = RaytraceRequestContext{
	    .sensorLinearVelocityXYZ = sensorLinearVelocityXYZ,
	    .sensorAngularVelocityRPY = sensorAngularVelocityRPY,
	    .doApplyDistortion = doApplyDistortion,
	    .rays = raysPtr,
	    .rayCount = raysNode->getRayCount(),
	    .rayOriginToWorld = raysNode->getCumulativeRayTransfrom(),
	    .rayRanges = rayRanges.has_value() ? (*rayRanges)->asSubclass<DeviceAsyncArray>()->getReadPtr() :
	                                         defaultRange->getReadPtr(),
	    .rayRangesCount = rayRanges.has_value() ? (*rayRanges)->getCount() : defaultRange->getCount(),
	    .ringIds = ringIds.has_value() ? (*ringIds)->asSubclass<DeviceAsyncArray>()->getReadPtr() : nullptr,
	    .ringIdsCount = ringIds.has_value() ? (*ringIds)->getCount() : 0,
	    .rayTimeOffsets = timeOffsets.has_value() ? (*timeOffsets)->asSubclass<DeviceAsyncArray>()->getReadPtr() : nullptr,
	    .rayTimeOffsetsCount = timeOffsets.has_value() ? (*timeOffsets)->getCount() : 0,
	    .scene = sceneAS,
	    .sceneTime = Scene::instance().getTime().value_or(Time::zero()).asSeconds(),
	    .sceneDeltaTime = static_cast<float>(Scene::instance().getDeltaTime().value_or(Time::zero()).asSeconds()),
	    .xyz = getPtrTo<XYZ_VEC3_F32>(),
	    .isHit = getPtrTo<IS_HIT_I32>(),
	    .rayIdx = getPtrTo<RAY_IDX_U32>(),
	    .ringIdx = getPtrTo<RING_ID_U16>(),
	    .distance = getPtrTo<DISTANCE_F32>(),
	    .intensity = getPtrTo<INTENSITY_F32>(),
	    .timestamp = getPtrTo<TIME_STAMP_F64>(),
	    .entityId = getPtrTo<ENTITY_ID_I32>(),
	    .pointAbsVelocity = getPtrTo<ABSOLUTE_VELOCITY_VEC3_F32>()};

	requestCtxDev->copyFrom(requestCtxHst);
	CUdeviceptr pipelineArgsPtr = requestCtxDev->getDeviceReadPtr();
	std::size_t pipelineArgsSize = requestCtxDev->getSizeOf() * requestCtxDev->getCount();
	CHECK_OPTIX(optixLaunch(Optix::getOrCreate().pipeline, getStreamHandle(), pipelineArgsPtr, pipelineArgsSize, &sceneSBT,
	                        launchDims.x, launchDims.y, launchDims.y));
}

void RaytraceNode::setFields(const std::set<rgl_field_t>& fields)
{
	auto keyViewer = std::views::keys(fieldData);
	std::set<rgl_field_t> currentFields{keyViewer.begin(), keyViewer.end()};

	std::set<rgl_field_t> toRemove, toInsert;
	std::ranges::set_difference(currentFields, fields, std::inserter(toRemove, toRemove.end()));
	std::ranges::set_difference(fields, currentFields, std::inserter(toInsert, toInsert.end()));

	for (auto&& field : toRemove) {
		fieldData.erase(field);
	}
	for (auto&& field : toInsert) {
		fieldData.insert({field, createArray<DeviceAsyncArray>(field, arrayMgr)});
	}
}

std::set<rgl_field_t> RaytraceNode::findFieldsToCompute()
{
	std::set<rgl_field_t> outFields;

	// Add primary field
	outFields.insert(XYZ_VEC3_F32);

	// dfsInputs - if false dfs for outputs
	std::function<void(Node::Ptr, bool)> dfsRet = [&](const Node::Ptr& current, bool dfsInputs) {
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
	dfsRet(shared_from_this(), false); // Search in outputs

	return outFields;
}

void RaytraceNode::setVelocity(const Vec3f* linearVelocity, const Vec3f* angularVelocity)
{
	doApplyDistortion = linearVelocity != nullptr && angularVelocity != nullptr;

	if (!doApplyDistortion) {
		return;
	}

	sensorLinearVelocityXYZ = *linearVelocity;
	sensorAngularVelocityRPY = *angularVelocity;
}
