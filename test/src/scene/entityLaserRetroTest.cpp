#include <helpers/lidarHelpers.hpp>
#include <helpers/sceneHelpers.hpp>
#include <helpers/commonHelpers.hpp>
#include <helpers/testPointCloud.hpp>

class EntityLaserRetroTest : public RGLTestWithParam<float>
{};

INSTANTIATE_TEST_SUITE_P(EntityLaserRetroTest, EntityLaserRetroTest, testing::Range(-100.1f, 100.1f, 50.0f),
                         [](const auto& info) {
	                         std::string valueStr = std::to_string(info.param);
	                         // The test name suffix can only contain alphanumeric characters and underscores.
	                         std::replace(valueStr.begin(), valueStr.end(), '-', 'm');
	                         std::replace(valueStr.begin(), valueStr.end(), '.', 'd');
	                         std::replace(valueStr.begin(), valueStr.end(), ',', 'd');
	                         return "retro_" + valueStr;
                         });

TEST_P(EntityLaserRetroTest, SmokeTest)
{
	// Create scene and set retro value
	float inLaserRetro = GetParam();
	auto cube = spawnCubeOnScene(Mat3x4f::TRS({0, 0, 0}));
	ASSERT_RGL_SUCCESS(rgl_entity_set_laser_retro(cube, inLaserRetro));

	// Construct graph
	std::vector<rgl_mat3x4f> rays = {
	    Mat3x4f::TRS({0, 0, 0}, {0, 0, 0}).toRGL(),                  // hit point
	    Mat3x4f::TRS({CUBE_HALF_EDGE * 3, 0, 0}, {0, 0, 0}).toRGL(), // non-hit point
	};

	std::vector<rgl_field_t> outFields { IS_HIT_I32, LASER_RETRO_F32 };

	rgl_node_t rayNode = nullptr, raytraceNode = nullptr, yieldNode = nullptr;
	ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&rayNode, rays.data(), rays.size()));
	ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));
	ASSERT_RGL_SUCCESS(rgl_node_points_yield(&yieldNode, outFields.data(), outFields.size()));

	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(rayNode, raytraceNode));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, yieldNode));

	// First run - check if retro value has been assigned properly
	{
		ASSERT_RGL_SUCCESS(rgl_graph_run(raytraceNode));

		auto outPointCloud = TestPointCloud::createFromNode(yieldNode, outFields);

		ASSERT_EQ(outPointCloud.getPointCount(), 2);

		ASSERT_EQ(outPointCloud.getFieldValue<IS_HIT_I32>(0), 1); // hit point
		ASSERT_EQ(outPointCloud.getFieldValue<LASER_RETRO_F32>(0), inLaserRetro);

		ASSERT_EQ(outPointCloud.getFieldValue<IS_HIT_I32>(1), 0); // non-hit point
		ASSERT_EQ(outPointCloud.getFieldValue<LASER_RETRO_F32>(1), 0.0f); // Retro should be zero
	}

	// Second run - check if retro value has been updated properly
	{
		float newLaserRetro = inLaserRetro + 1.0f;
		ASSERT_RGL_SUCCESS(rgl_entity_set_laser_retro(cube, newLaserRetro));
		ASSERT_RGL_SUCCESS(rgl_graph_run(raytraceNode));

		auto outPointCloud = TestPointCloud::createFromNode(yieldNode, outFields);
		ASSERT_EQ(outPointCloud.getFieldValue<IS_HIT_I32>(0), 1); // hit point
		ASSERT_EQ(outPointCloud.getFieldValue<LASER_RETRO_F32>(0), newLaserRetro);

		ASSERT_EQ(outPointCloud.getFieldValue<IS_HIT_I32>(1), 0); // non-hit point
		ASSERT_EQ(outPointCloud.getFieldValue<LASER_RETRO_F32>(1), 0.0f); // Retro should be zero
	}
}
