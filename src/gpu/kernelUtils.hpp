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

#include <macros/cuda.hpp>

#define LIMIT(count)                                                                                                           \
	const int tid = (blockIdx.x * blockDim.x + threadIdx.x);                                                                   \
	do {                                                                                                                       \
		if (tid >= count) {                                                                                                    \
			return;                                                                                                            \
		}                                                                                                                      \
	} while (false)

template<typename Kernel, typename... KernelArgs>
void run(Kernel&& kernel, cudaStream_t stream, size_t threads, KernelArgs... kernelArgs)
{
	int blockDim = 256;
	int blockCount = 1 + threads / 256;
	void* args[] = { &threads, &kernelArgs... };
	CHECK_CUDA(cudaLaunchKernel(reinterpret_cast<void*>(kernel), blockCount, blockDim, args, 0, stream));
}
