#pragma once

#include <gtest/gtest.h>
#include <gmock/gmock-matchers.h>

#include <rgl/api/core.h>

// TODO(msz-rai): Left this namespace for cleaner tests - fix namespace for rgl (Field)
// using namespace ::testing;

static constexpr float EPSILON_F = 1e-6f;
static constexpr int maxGPUCoresTestCount = 20000;

#define EXPECT_RGL_SUCCESS(status) EXPECT_EQ(status, rgl_status_t::RGL_SUCCESS)
#define ASSERT_RGL_SUCCESS(status) ASSERT_EQ(status, rgl_status_t::RGL_SUCCESS)
#define EXPECT_RGL_STATUS(actual, expected, ...)                                                                               \
	do {                                                                                                                       \
		EXPECT_EQ(actual, expected);                                                                                           \
		const char* error_string;                                                                                              \
		rgl_get_last_error_string(&error_string);                                                                              \
		std::vector<std::string> errMsgBits = {__VA_ARGS__};                                                                   \
		for (auto&& substr : errMsgBits) {                                                                                     \
			EXPECT_THAT(error_string, testing::HasSubstr(substr));                                                             \
		}                                                                                                                      \
	} while (false)

#define EXPECT_RGL_INVALID_OBJECT(status, type) EXPECT_RGL_STATUS(status, RGL_INVALID_API_OBJECT, "Object does not exist", type)
#define EXPECT_RGL_INVALID_ARGUMENT(status, error) EXPECT_RGL_STATUS(status, RGL_INVALID_ARGUMENT, error)
#define EXPECT_RGL_INVALID_PIPELINE(status, error) EXPECT_RGL_STATUS(status, RGL_INVALID_PIPELINE, error)

struct RGLTest : public ::testing::Test
{
protected:
	RGLTest() { rgl_configure_logging(RGL_LOG_LEVEL_OFF, nullptr, false); }

	~RGLTest() override { EXPECT_RGL_SUCCESS(rgl_cleanup()); }
};

template<typename T>
struct RGLTestWithParam : public RGLTest, public ::testing::WithParamInterface<T>
{};
