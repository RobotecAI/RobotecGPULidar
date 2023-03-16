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

/** Enumerates kinds of CUDA memory used in RGL. @see MemoryOperations */
enum struct MemoryKind
{
	/**
	 * Device global memory, but allocated asynchronously via cudaMallocAsync.
	 */
	DeviceAsync,

	/**
	 * Device global memory. Allocated with cudaMalloc.
	 * Operations on this type of memory happen in the CUDA null stream.
	 */
	DeviceSync,

	/**
	 * Regular host memory (e.g., allocated with malloc or new)
	 */
	HostPageable,

	/**
	 * Page-locked host memory. Used for full-speed transfers between Host and GPU
	 * Using large amounts of pinned memory may degrade system performance, because it cannot be swapped.
	 * Operations on this type of memory happen in the CUDA null stream.
	 */
	HostPinned,
};