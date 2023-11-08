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
//#include <scene/Scene.hpp>
#include <Logger.hpp>

// RAII object to (de)initialize cudaStream_t
struct CudaStream
{
	using Ptr = std::shared_ptr<CudaStream>;

	static CudaStream::Ptr create(unsigned flags = 0U) { return CudaStream::Ptr(new CudaStream(flags)); }

	static CudaStream::Ptr getNullStream();

	/**
	 * Copy stream is used to e.g. copy results from a completed node to user,
	 * while the rest of Graph Nodes are still being executed in Graph Stream.
	 */
	static CudaStream::Ptr getCopyStream();

	/**
	 * Mesh stream is used to perform mesh-related operations, such as copying/updating vertex data and building GASes.
	 * Meshes are supposed to be shared between scenes.
	 * That would require some synchronization between scene streams and mesh stream.
	 * Therefore, since for now only a single scene is supported, we use Scene stream instead.
	 */
	static CudaStream::Ptr getMeshStream();

	cudaStream_t getHandle() { return stream; }

	~CudaStream();

private:
	// Wraps null stream
	CudaStream() {}

	// Constructs a new stream
	explicit CudaStream(unsigned flags) { CHECK_CUDA(cudaStreamCreateWithFlags(&stream, flags)); }

private:
	cudaStream_t stream{nullptr};
};
