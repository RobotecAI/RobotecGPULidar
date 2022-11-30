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
	return status;
}