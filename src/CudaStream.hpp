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

#include <macros/cuda.hpp>
#include <macros/handleDestructorException.hpp>
#include <Logger.hpp>

// RAII object to (de)initialize cudaStream_t
struct CudaStream
{
	using Ptr = std::shared_ptr<CudaStream>;

	static CudaStream::Ptr create(unsigned flags = 0U) { return CudaStream::Ptr(new CudaStream(flags)); }

	static CudaStream::Ptr getNullStream()
	{
		// Copy Stream does not synchronize with the NULL stream
		// Copy Stream is always synchronized before client thread finished API call
		static CudaStream::Ptr nullStream{new CudaStream()};
		return nullStream;
	}

	static CudaStream::Ptr getCopyStream()
	{
		static CudaStream::Ptr copyStream{new CudaStream(cudaStreamNonBlocking)};
		return copyStream;
	}

	cudaStream_t getHandle() { return stream; }

	~CudaStream()
	try {
		if (stream != nullptr) {
			CHECK_CUDA(cudaStreamSynchronize(stream)); // May not be required, but it is safer
			CHECK_CUDA(cudaStreamDestroy(stream));
			stream = nullptr;
		}
	}
	HANDLE_DESTRUCTOR_EXCEPTION

private:
	// Wraps null stream
	CudaStream() {}

	// Constructs a new stream
	explicit CudaStream(unsigned flags) { CHECK_CUDA(cudaStreamCreateWithFlags(&stream, flags)); }

private:
	cudaStream_t stream{nullptr};
};
