#pragma once

#include <testUtils.hpp>

class RGLAutoCleanupTest : public ::testing::Test {
protected:
	RGLAutoCleanupTest()
	{
		rgl_configure_logging(RGL_LOG_LEVEL_OFF, nullptr, false);
	}
	virtual ~RGLAutoCleanupTest() override { rgl_cleanup(); }
};
