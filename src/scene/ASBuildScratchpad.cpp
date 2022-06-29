#include <scene/ASBuildScratchpad.hpp>
#include <macros/optix.hpp>
#include <macros/cuda.hpp>
#include <HostPinnedBuffer.hpp>

bool ASBuildScratchpad::resizeToFit(OptixBuildInput input, OptixAccelBuildOptions options)
{
	OptixAccelBufferSizes bufferSizes;
	CHECK_OPTIX(optixAccelComputeMemoryUsage(Optix::instance().context, &options, &input, 1, &bufferSizes));

	return dTemp.resizeToFit(bufferSizes.tempSizeInBytes)
	    || dFull.resizeToFit(bufferSizes.outputSizeInBytes)
	    || dCompactedSize.resizeToFit(1);
}

void ASBuildScratchpad::doCompaction(OptixTraversableHandle &handle)
{
	throw std::runtime_error("AS compaction is disabled due to performance reasons");
	// TODO(prybicki): Too many lines for getting a number from GPU :(
	// TODO(prybicki): Some time later, it turns out that this communication (async cpy + sync) is killing perf
	// TODO(prybicki): This should remain disabled, until a real-world memory management subsystem is implemented
	// TODO(prybicki): Copy here should be not async...
	HostPinnedBuffer<uint64_t> hCompactedSize;
	hCompactedSize.copyFromDeviceAsync(dCompactedSize, nullptr); // TODO: stream
	CHECK_CUDA(cudaStreamSynchronize(nullptr));
	uint64_t compactedSize = *hCompactedSize.readHost();

	dCompact.resizeToFit(compactedSize);
	CHECK_OPTIX(optixAccelCompact(Optix::instance().context,
	                              nullptr, // TODO: stream
	                              handle,
	                              dCompact.readDeviceRaw(),
	                              dCompact.getByteSize(),
	                              &handle));
}
