#pragma once

#define EXPECT_RGL_SUCCESS(status) EXPECT_EQ(status, rgl_status_t::RGL_SUCCESS)
#define EXPECT_RGL_INVALID_ARGUMENT(status, error_string_msg)     \
    {                                                             \
        EXPECT_EQ(status, rgl_status_t::RGL_INVALID_ARGUMENT);    \
        const char* error_string;                                 \
        rgl_get_last_error_string(&error_string);                 \
        EXPECT_THAT(error_string, HasSubstr("Invalid argument")); \
        EXPECT_THAT(error_string, HasSubstr(error_string_msg));   \
    }
