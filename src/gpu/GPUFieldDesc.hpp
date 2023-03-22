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

// Used to send e.g. formatting request to GPU
struct GPUFieldDesc
{
	// Only one DataPtr will be assigned (the other will be nullptr).
	// Ideally, std::variant would simplify these two members, but CUDA doesn't support them.
	// An alternative is a union, but this is not a safe solution.
	const char* readDataPtr;
	char* writeDataPtr;

	size_t size;
	size_t dstOffset;
};
static_assert(std::is_trivially_copyable<GPUFieldDesc>::value);
