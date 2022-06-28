#pragma once

#define RGL_CHECK(call)                  \
do {                                     \
	rgl_status_t status = call;          \
	if (status != RGL_SUCCESS) {         \
		const char* msg;                 \
		rgl_get_last_error_string(&msg); \
		FAIL() << msg;                   \
	}                                    \
} while(0)
