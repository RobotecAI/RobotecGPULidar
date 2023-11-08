// Copyright 2023 Robotec.AI
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

#include <CudaStream.hpp>
#include <scene/Scene.hpp>

CudaStream::Ptr CudaStream::getMeshStream() { return Scene::defaultInstance()->getStream(); }

CudaStream::Ptr CudaStream::getCopyStream()
{
	static CudaStream::Ptr copyStream{new CudaStream(cudaStreamNonBlocking)};
	return copyStream;
}

CudaStream::Ptr CudaStream::getNullStream()
{
	// Copy Stream does not synchronize with the NULL stream
	// Copy Stream is always synchronized before client thread finished API call
	static CudaStream::Ptr nullStream{new CudaStream()};
	return nullStream;
}

CudaStream::~CudaStream()
try {
	if (stream != nullptr) {
		CHECK_CUDA(cudaStreamSynchronize(stream)); // May not be required, but it is safer
		CHECK_CUDA(cudaStreamDestroy(stream));
		stream = nullptr;
	}
}
HANDLE_DESTRUCTOR_EXCEPTION
