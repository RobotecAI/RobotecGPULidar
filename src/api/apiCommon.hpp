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

#include <cmath>
#include <thread>
#include <spdlog/common.h>

#if RGL_BUILD_ROS2_EXTENSION
#include <rclcpp/exceptions.hpp>
#endif

#include <rgl/api/core.h>

#include <Optix.hpp>
#include <graph/Node.hpp>
#include <graph/GraphRunCtx.hpp>

#include <Tape.hpp>
#include <RGLExceptions.hpp>

#include <repr.hpp>

#define RGL_API_LOG RGL_TRACE

#define CHECK_ARG(expr)                                                                          \
do if (!(expr)) {                                                                                \
    auto msg = fmt::format("RGL API Error: Invalid argument, condition unsatisfied: {}", #expr); \
    throw std::invalid_argument(msg);                                                            \
} while(0)

extern rgl_status_t lastStatusCode;
extern std::optional<std::string> lastStatusString;

void rglLazyInit();
const char* getLastErrorString() noexcept;
bool canContinueAfterStatus(rgl_status_t status);
rgl_status_t updateAPIState(rgl_status_t status, std::optional<std::string> auxErrStr = std::nullopt);

template<typename Fn>
rgl_status_t rglSafeCall(Fn fn)
{
	rglLazyInit(); // Trigger initialization on the first API call
	if (!canContinueAfterStatus(lastStatusCode)) {
		if (lastStatusCode != RGL_LOGGING_ERROR) {
			RGL_CRITICAL("Logging disabled due to the previous fatal error");
			try {
				Logger::getOrCreate().configure(RGL_LOG_LEVEL_OFF, std::nullopt, false);
			}
			catch (std::exception& e) {}
		}
		return updateAPIState(RGL_INVALID_STATE);
	}
	try {
		std::invoke(fn);
	}
	#if RGL_BUILD_ROS2_EXTENSION
	catch (rclcpp::exceptions::RCLErrorBase& e) {
		return updateAPIState(RGL_ROS2_ERROR, e.message.c_str());
	}
	#endif
	catch (spdlog::spdlog_ex& e) {
		return updateAPIState(RGL_LOGGING_ERROR, e.what());
	}
	catch (InvalidAPIObject& e) {
		return updateAPIState(RGL_INVALID_API_OBJECT, e.what());
	}
	catch (InvalidPipeline &e) {
		return updateAPIState(RGL_INVALID_PIPELINE, e.what());
	}
	catch (InvalidFilePath& e) {
		return updateAPIState(RGL_INVALID_FILE_PATH, e.what());
	}
	catch (std::invalid_argument &e) {
		return updateAPIState(RGL_INVALID_ARGUMENT, e.what());
	}
	catch (RecordError& e) {
		return updateAPIState(RGL_TAPE_ERROR, e.what());
	}
	catch (std::exception& e) {
		return updateAPIState(RGL_INTERNAL_EXCEPTION, e.what());
	}
	catch (...) {
		return updateAPIState(RGL_INTERNAL_EXCEPTION, "exceptional exception");
	}
	return updateAPIState(RGL_SUCCESS);
}

template<typename NodeType, typename... Args>
void createOrUpdateNode(rgl_node_t* nodeRawPtr, Args&&... args)
{
	std::shared_ptr<NodeType> node;
	if (*nodeRawPtr == nullptr) {
		node = Node::create<NodeType>();
	}
	else {
		node = Node::validatePtr<NodeType>(*nodeRawPtr);
	}
	// TODO: The magic below detects calls changing rgl_field_t* (e.g. FormatPointsNode)
	// TODO: Such changes may require recomputing required fields in RaytraceNode.
	// TODO: However, taking care of this manually is very bug prone.
	// TODO: There are other ways to automate this, however, for now this should be enough.
	bool fieldsModified = ((std::is_same_v<Args, std::vector<rgl_field_t>> || ... ));
	if (fieldsModified && node->hasGraphRunCtx()) {
		node->getGraphRunCtx()->detachAndDestroy();
	}

	// As of now, there's no guarantee that changing node parameter won't influence other nodes
	// Therefore, before changing them, we need to ensure all nodes are idle (not running in GraphRunCtx).
	if (node->hasGraphRunCtx()) {
		node->getGraphRunCtx()->synchronize();
	}
	node->setParameters(args...);
	node->dirty = true;
	*nodeRawPtr = node.get();
}

inline void handleDestructorException(std::exception_ptr e, const char* what)
{
	RGL_CRITICAL("Exception thrown from destructor!");
	updateAPIState(RGL_INTERNAL_EXCEPTION, what);
	// We got some exception thrown in some destructor
	// We may be in graph thread which should get gracefully killed:
	// TODO: Implement this in a thread-safe manner (accessing GraphRunCtx::instances)
	std::lock_guard lock { GraphRunCtx::instancesMutex };
	for (auto&& ctx : GraphRunCtx::instances) {
		if (!ctx->maybeThread.has_value()) {
			continue;
		}
		if (ctx->maybeThread.value().get_id() != std::this_thread::get_id()) {
			continue;
		}
		// We're a graph thread. Mark remaining nodes as executed and set exception.
		for (auto&& [node, state] : ctx->executionStatus) {
			if (state.enqueued.load(std::memory_order::relaxed)) {
				continue;
			}
			state.exceptionPtr = std::current_exception();
			state.enqueued.store(true, std::memory_order::release);
			throw e; // In Graph thread, rethrow to die.
		}
	}
}