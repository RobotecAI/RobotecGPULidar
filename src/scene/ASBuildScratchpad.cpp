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

#include <scene/ASBuildScratchpad.hpp>
#include <macros/optix.hpp>
#include <macros/cuda.hpp>

void ASBuildScratchpad::resizeToFit(OptixBuildInput input, OptixAccelBuildOptions options)
{
	OptixAccelBufferSizes bufferSizes;
	CHECK_OPTIX(optixAccelComputeMemoryUsage(Optix::getOrCreate().context, &options, &input, 1, &bufferSizes));

	// Short-circuit evaluation workaround
	dTemp->resize(bufferSizes.tempSizeInBytes, false, false);
	dFull->resize(bufferSizes.outputSizeInBytes, false, false);
	dCompactedSize->resize(1, false, false);
}

void ASBuildScratchpad::doCompaction(OptixTraversableHandle &handle)
{
	throw std::runtime_error("AS compaction is disabled due to performance reasons");
	// TODO(prybicki): Too many lines for getting a number from GPU :(
	// TODO(prybicki): Some time later, it turns out that this communication (async cpy + synchronize) is killing perf
	// TODO(prybicki): This should remain disabled, until a real-world memory management subsystem is implemented

	// Commented out due to introduction of Array<M,T>
	// Should be re-implemented when needed using new interface

	// hCompactedSize->copyFromDeviceAsync(dCompactedSize, nullptr); // TODO: stream
	// CHECK_CUDA(cudaStreamSynchronize(nullptr));
	// uint64_t compactedSize = *hCompactedSize->readHost();

	// dCompact.resizeToFit(compactedSize);
	// CHECK_OPTIX(optixAccelCompact(Optix::getOrCreate().context,
	//                               nullptr, // TODO: stream
	//                               handle,
	//                               dCompact.readDeviceRaw(),
	//                               dCompact.getByteSize(),
	//                               &handle));
}
