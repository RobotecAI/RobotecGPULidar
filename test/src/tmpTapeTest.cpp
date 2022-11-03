#include <gtest/gtest.h>
#include <utils.hpp>
#include <scenes.hpp>
#include <lidars.hpp>
#include <RGLFields.hpp>
#include <rgl/api/extensions/tape_api.h>

#include <math/Mat3x4f.hpp>

class TmpTapeTest : public RGLAutoCleanupTest {};

TEST_F(TmpTapeTest, record)
{
	rgl_tape_record_begin("/home/mateusz/Music/recording");

	auto mesh = makeCubeMesh();

	rgl_node_t compact=nullptr, raytrace=nullptr;

	rgl_configure_logging(RGL_LOG_LEVEL_DEBUG, "/tmp/test.log", true);

	rgl_node_raytrace(&raytrace, nullptr, 1000);
	rgl_node_points_compact(&compact);

	rgl_graph_node_add_child(raytrace, compact);

	rgl_graph_node_set_active(raytrace, true);
	rgl_graph_node_set_active(compact, false);

	rgl_tape_record_end();
}
