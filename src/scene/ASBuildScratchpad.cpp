#include <scene/ASBuildScratchpad.hpp>
#include <macros/optix.hpp>
#include <macros/cuda.hpp>
#include <HostPinnedBuffer.hpp>

void ASBuildScratchpad::resizeToFit(OptixBuildInput input, OptixAccelBuildOptions options)
{
	OptixAccelBufferSizes bufferSizes;
	CHECK_OPTIX(optixAccelComputeMemoryUsage(Optix::instance().context, &options, &input, 1, &bufferSizes));

	dTemp.resizeToFit(bufferSizes.tempSizeInBytes);
	dFull.resizeToFit(bufferSizes.outputSizeInBytes);
	dCompactedSize.resizeToFit(1);
}

void ASBuildScratchpad::doCompaction(OptixTraversableHandle &handle)
{
	// TODO(prybicki): Too many lines for getting a number from GPU :(
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
