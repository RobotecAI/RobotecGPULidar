#pragma once

#include <cuda_runtime_api.h>
#include <optional>
#include <cstring>

#include <memory/MemoryKind.hpp>
#include <macros/cuda.hpp>
#include <CudaStream.hpp>

struct CopyArg
{
	void* ptr;
	MemoryKind kind;
	std::optional<CudaStream::Ptr> stream;
};
struct CopyOp
{
	CopyArg src;
	CopyArg dst;
	size_t bytes;
};

void copy(const CopyOp& copy)
{
	// Both operands are on host - either pageable or pinned.
	// Standard memcpy is faster + avoids the overhead of cudaMemcpy*
	if (isHost(copy.dst.kind) && isHost(copy.src.kind)) {
		memcpy(copy.dst.ptr, copy.src.ptr, copy.bytes);
		return;
	}

	// Ensure src is ready (one day, it can be optimized to some waiting on some cudaEvent, not entire stream)
	if (copy.src.stream.has_value()) {
		CHECK_CUDA(cudaStreamSynchronize(copy.src.stream.value()->getHandle()));
	}

	// TODO: remove null stream usage once DeviceSyncArray is removed
	CudaStream::Ptr copyStream = copy.dst.stream.has_value() ? copy.dst.stream.value() : CudaStream::getNullStream();
	CHECK_CUDA(cudaMemcpyAsync(copy.dst.ptr, copy.src.ptr, copy.bytes, cudaMemcpyDefault, copyStream->getHandle()));
	CHECK_CUDA(cudaStreamSynchronize(copyStream->getHandle()));
}
