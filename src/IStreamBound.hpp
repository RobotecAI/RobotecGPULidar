#pragma once

#include <CudaStream.hpp>

/**
 * Interface for objects that hold a stream and allow changing it in runtime.
 * It is a caller responsibility to make sure that stream can be changed.
 */
struct IStreamBound
{
	virtual void setStream(CudaStream::Ptr) = 0;
};