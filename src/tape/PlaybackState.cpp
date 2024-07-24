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

#include <fcntl.h>

#include <tape/PlaybackState.hpp>
#include <RGLExceptions.hpp>
#include <macros/handleDestructorException.hpp>

// Hack to complete compilation on Windows. In runtime, it is never used.
#ifdef _WIN32
#include <io.h>
#define PROT_READ 1
#define MAP_PRIVATE 1
#define MAP_FAILED nullptr
static int munmap(void* addr, size_t length) { return -1; }
static void* mmap(void* start, size_t length, int prot, int flags, int fd, size_t offset) { return nullptr; }
#else
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif // _WIN32

PlaybackState::PlaybackState(const char* binaryFilePath) { mmapInit(binaryFilePath); }

void PlaybackState::clear()
{
	meshes.clear();
	entities.clear();
	textures.clear();
	skeletons.clear();
	nodes.clear();
}

PlaybackState::~PlaybackState()
try {
	if (fileMmap == nullptr) {
		return;
	}
	if (munmap(fileMmap, mmapSize) == -1) {
		throw std::runtime_error(fmt::format("TAPE: failed to remove binary mappings due to {}", std::strerror(errno)));
	}
}
HANDLE_DESTRUCTOR_EXCEPTION

void PlaybackState::mmapInit(const char* path)
{
	int fd = open(path, O_RDONLY);
	if (fd < 0) {
		throw InvalidFilePath(fmt::format("TAPE: could not open binary file: '{}' "
		                                  " due to the error: {}",
		                                  path, std::strerror(errno)));
	}

	try {
		struct stat staticBuffer
		{};
		int err = fstat(fd, &staticBuffer);
		if (err < 0) {
			throw InvalidFilePath("TAPE: couldn't read binary file length");
		}

		mmapSize = staticBuffer.st_size;

		if (staticBuffer.st_size > 0) {
			fileMmap = (uint8_t*) mmap(nullptr, staticBuffer.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
			if (fileMmap == MAP_FAILED) {
				throw InvalidFilePath(fmt::format("TAPE: could not mmap binary file: {}", path));
			}
		}
	}
	catch (...) {
		close(fd);
		throw;
	}
}
