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

#pragma once

#include <functional>

#include <macros/cuda.hpp>
#include <memory/MemoryKind.hpp>
#include <CudaStream.hpp>

/**
 * MemoryOperations encapsulate 4 basic memory operations needed to implement dynamic-size array.
 * It also provides factory method to create MemoryOperations corresponding to those defined in MemoryKind enum.
 * Warning: deallocate, copy and clear can work ONLY on the memory kind returned by allocate.
 */
struct MemoryOperations
{
	std::function<void*(size_t)> allocate;
	std::function<void(void*)> deallocate;
	std::function<void(void*, const void*, size_t)> copy;
	std::function<void(void*, int value, size_t)> clear;

	/** Returns MemoryOperations for given MemoryKind */
	template<MemoryKind memoryKind>
	static MemoryOperations get(std::optional<CudaStream::Ptr> maybeStream = std::nullopt)
	{
		// clang-format off
		if constexpr (memoryKind == MemoryKind::HostPageable) {
			return {
				.allocate = malloc,
				.deallocate = free,
				.copy = memcpy,
				.clear = memset };
		}
		else if constexpr (memoryKind == MemoryKind::HostPinned) {
			return {
				.allocate = [](size_t bytes) {
					void* ptr = nullptr;
					CHECK_CUDA(cudaMallocHost(&ptr, bytes));
					return ptr;
				},
				.deallocate = [](void* ptr) {
					CHECK_CUDA(cudaFreeHost(ptr));
				},
				// Regular memcpy and memset avoid the overhead of cuda[Memcpy|Memset] and achieve higher performance.
				.copy = memcpy,
				.clear = memset };
		}
		else if constexpr (memoryKind == MemoryKind::DeviceSync || memoryKind == MemoryKind::DeviceAsync) { // Force SYNC for debugging
			return {
				.allocate = [](size_t bytes) {
					void* ptr = nullptr;
					CHECK_CUDA(cudaMalloc(&ptr, bytes));
					return ptr;
				},
				.deallocate = [](void* ptr) {
					CHECK_CUDA(cudaFree(ptr));
				},
				.copy = [](void* dst, const void* src, size_t bytes) {
					CHECK_CUDA(cudaMemcpy(dst, src, bytes, cudaMemcpyDeviceToDevice));
				},
				.clear = [=](void* dst, int value, size_t bytes) {
					CHECK_CUDA(cudaMemset(dst, value, bytes));
				}
			};
		}
		else {
			static_assert("invalid memory kind passed to MemoryOperations::get()");
		}
		// clang-format on
	}
};
