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
#include <Logger.hpp>

// RAII object to (de)initialize cudaStream_t
struct CudaStream
{
	using Ptr = std::shared_ptr<CudaStream>;

	static CudaStream::Ptr create()
	{
		return CudaStream::Ptr(new CudaStream(0U));
	}

	static CudaStream::Ptr getNullStream()
	{
		static CudaStream::Ptr nullStream { new CudaStream() };
		return nullStream;
	}

	cudaStream_t get() { return stream; }

	~CudaStream()
	{
		if (stream != nullptr) {
			try {
				CHECK_CUDA(cudaStreamSynchronize(stream));  // May not be required, but it is safer
				CHECK_CUDA(cudaStreamDestroy(stream));
				stream = nullptr;
			}
			catch(std::runtime_error& e) {
				RGL_ERROR("Error in ~CudaStream: {}", e.what());
			}
		}
	}


private:
	// Wraps null stream
	CudaStream() {}

	// Constructs a new stream
	explicit CudaStream(unsigned flags) { CHECK_CUDA(cudaStreamCreateWithFlags(&stream, flags)); }

private:
	cudaStream_t stream {nullptr};
};
