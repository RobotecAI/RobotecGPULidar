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

#include "rgl/api/extensions/tape.h"
#include "spdlog/fmt/fmt.h"

int main(int argc, char** argv)
{
	if (argc != 2) {
		fmt::print(stderr, "USAGE: {} <path-to-tape-without-suffix>\n", argv[0]);
		return 1;
	}

	rgl_status_t status = rgl_tape_play(argv[1]);
	fmt::print("Tape finished with status {}\n", status);
	rgl_cleanup();
	return status;
}