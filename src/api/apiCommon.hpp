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

#include <cmath>
#include <spdlog/common.h>

#include <rgl/api/core.h>

#include <gpu/Optix.hpp>
#include <graph/Node.hpp>

#include <Tape.hpp>
#include <RGLExceptions.hpp>

#include <repr.hpp>

#define RGL_API_LOG RGL_TRACE

#define CHECK_ARG(expr)                                                                          \
do if (!(expr)) {                                                                                \
    auto msg = fmt::format("RGL API Error: Invalid argument, condition unsatisfied: {}", #expr); \
    throw std::invalid_argument(msg);                                                            \
} while(0)

static rgl_status_t lastStatusCode = RGL_SUCCESS;
static std::optional<std::string> lastStatusString = std::nullopt;

static bool isCompiledWithAutoTape() { return !std::string(RGL_AUTO_TAPE_PATH).empty(); }

static void rglLazyInit();

static const char* getLastErrorString()
{
	// By default, use lastStatusString value, which is usually a message from the caught exception
	if (lastStatusString.has_value()) {
		return lastStatusString->c_str();
	}
	// If lastStatusString is not set, provide a fallback explanation:
	switch (lastStatusCode) {
		case RGL_SUCCESS: return "operation successful";
		case RGL_INVALID_ARGUMENT: return "invalid argument";
		case RGL_INTERNAL_EXCEPTION: return "unhandled internal exception";
		case RGL_INVALID_STATE: return "invalid state - unrecoverable error occurred";
		case RGL_NOT_IMPLEMENTED: return "operation not (yet) implemented";
		case RGL_LOGGING_ERROR: return "spdlog error";
		case RGL_INVALID_FILE_PATH: return "invalid file path";
		case RGL_TAPE_ERROR: return "tape error";
		default: return "???";
	}
}

static bool canContinueAfterStatus(rgl_status_t status)
{
	// Set constructor may throw, hence lazy initialization.
	static std::set recoverableErrors = {
		RGL_INVALID_ARGUMENT,
		RGL_INVALID_API_OBJECT,
		RGL_INVALID_PIPELINE,
		RGL_INVALID_FILE_PATH,
		RGL_NOT_IMPLEMENTED,
		RGL_TAPE_ERROR
	};
	return status == RGL_SUCCESS || recoverableErrors.contains(status);
};

static rgl_status_t updateAPIState(rgl_status_t status, std::optional<std::string> auxErrStr = std::nullopt)
{
	lastStatusString = auxErrStr;
	lastStatusCode = status;
	if (status != RGL_SUCCESS) {
		// Report API error in log file
		const char* msg = getLastErrorString();
		if (canContinueAfterStatus(lastStatusCode)) {
			RGL_ERROR("Recoverable error (code={}): {}", lastStatusCode, msg);
		}
		else {
			RGL_CRITICAL("Unrecoverable error (code={}): {}", lastStatusCode, msg);
		}
		Logger::getOrCreate().flush();
	}
	return status;
}

template<typename Fn>
static rgl_status_t rglSafeCall(Fn fn)
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

static void rglLazyInit()
{
	static bool initCalled = false;
	if (initCalled) {
		return;
	}
	initCalled = true;
	rgl_status_t initStatus = rglSafeCall([&]() {
		Logger::getOrCreate();
		Optix::getOrCreate();
		if (isCompiledWithAutoTape()) {
			auto path = std::filesystem::path(RGL_AUTO_TAPE_PATH);
			RGL_INFO("Starting RGL Auto Tape on path '{}'", path.string());
			tapeRecord.emplace(path);
		}
	});
	if (initStatus != RGL_SUCCESS) {
		// If initialization fails, change error code to unrecoverable one and preserve original message
		updateAPIState(RGL_INITIALIZATION_ERROR, lastStatusString);
	}
}

template<typename NodeType, typename... Args>
void createOrUpdateNode(rgl_node_t* nodeRawPtr, Args&&... args)
{
	CHECK_ARG(nodeRawPtr != nullptr);
	std::shared_ptr<NodeType> node;
	if (*nodeRawPtr == nullptr) {
		node = Node::create<NodeType>();
	}
	else {
		node = Node::validatePtr<NodeType>(*nodeRawPtr);
	}
	node->setParameters(args...);
	*nodeRawPtr = node.get();
}