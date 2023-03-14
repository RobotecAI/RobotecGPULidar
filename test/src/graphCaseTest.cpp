#include <gtest/gtest.h>
#include <utils.hpp>
#include <scenes.hpp>
#include <lidars.hpp>
#include <RGLFields.hpp>
#include <Time.hpp>

#include <math/Mat3x4f.hpp>

class GraphCase : public RGLTest {};

#ifdef RGL_BUILD_PCL_EXTENSION
#include <rgl/api/extensions/pcl.h>
#endif

TEST_F(GraphCase, FullLinear)
{
	setupBoxesAlongAxes(nullptr);

	rgl_node_t useRays=nullptr, raytrace=nullptr, lidarPose=nullptr, shear=nullptr, compact=nullptr, downsample=nullptr;

	std::vector<rgl_mat3x4f> rays = makeLidar3dRays(360, 180, 0.36, 0.18);
	rgl_mat3x4f lidarPoseTf = Mat3x4f::TRS({5, 5, 5}, {45, 45, 45}).toRGL();
	rgl_mat3x4f shearTf = Mat3x4f::shear({0,0}, {-1, -1}, {0, 0}).toRGL();

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&lidarPose, &lidarPoseTf));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr, 1000));
	EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compact));
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&shear, &shearTf));

#ifdef RGL_BUILD_PCL_EXTENSION
	EXPECT_RGL_SUCCESS(rgl_node_points_downsample(&downsample, 0.1f, 0.1f, 0.1f));
#endif

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, lidarPose));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(lidarPose, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, compact));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compact, shear));

#ifdef RGL_BUILD_PCL_EXTENSION
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(shear, downsample));
#endif

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));
#ifdef RGL_BUILD_PCL_EXTENSION
	EXPECT_RGL_SUCCESS(rgl_graph_write_pcd_file(downsample, "minimal.pcd"));
#else
	RGL_WARN("RGL compiled without PCL extension. Tests will not save PCD!");
#endif
}

TEST_F(GraphCase, NodeRemoval)
{
	auto mesh = makeCubeMesh();

	auto entity = makeEntity(mesh);
	rgl_mat3x4f entityPoseTf = Mat3x4f::identity().toRGL();
	ASSERT_RGL_SUCCESS(rgl_entity_set_pose(entity, &entityPoseTf));

	rgl_node_t useRays=nullptr, raytrace=nullptr, lidarPose=nullptr, transformPts=nullptr, compact=nullptr, downsample=nullptr;
	rgl_node_t temporalMerge=nullptr;
	std::vector<rgl_field_t> tMergeFields = { RGL_FIELD_XYZ_F32 };

	std::vector<rgl_mat3x4f> rays = makeLidar3dRays(360, 180, 0.36, 0.18);
	rgl_mat3x4f lidarPoseTf = Mat3x4f::TRS({0, 0, -5}).toRGL();
	rgl_mat3x4f zeroTf = Mat3x4f::TRS({0, 0, 0}).toRGL();
	rgl_mat3x4f translateXTf = Mat3x4f::TRS({3, 0, 0}).toRGL();
	rgl_mat3x4f translateYTf = Mat3x4f::TRS({0, 3, 0}).toRGL();

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&lidarPose, &lidarPoseTf));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr, 1000));
	EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compact));
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPts, &zeroTf));
	EXPECT_RGL_SUCCESS(rgl_node_points_temporal_merge(&temporalMerge, tMergeFields.data(), tMergeFields.size()));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, lidarPose));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(lidarPose, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, compact));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compact, transformPts));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transformPts, temporalMerge));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	// Remove compact<->transformPts connection
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPts, &translateXTf));
	EXPECT_RGL_SUCCESS(rgl_graph_node_remove_child(compact, transformPts));
	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	// Restore compact<->transformPts connection
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPts, &translateYTf));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compact, transformPts));
	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	// Output pointcloud should contain two boxes
#ifdef RGL_BUILD_PCL_EXTENSION
	EXPECT_RGL_SUCCESS(rgl_graph_write_pcd_file(temporalMerge, "two_boxes_removal.pcd"));
#else
	RGL_WARN("RGL compiled without PCL extension. Tests will not save PCD!");
#endif
}

TEST_F(GraphCase, SpatialMergeFromTransforms)
{
	auto mesh = makeCubeMesh();

	auto entity = makeEntity(mesh);
	rgl_mat3x4f entityPoseTf = Mat3x4f::identity().toRGL();
	ASSERT_RGL_SUCCESS(rgl_entity_set_pose(entity, &entityPoseTf));

	rgl_node_t useRays=nullptr, raytrace=nullptr, lidarPose=nullptr, compact=nullptr;
	rgl_node_t transformPtsZero=nullptr, transformPtsY=nullptr;
	rgl_node_t spatialMerge=nullptr;
	std::vector<rgl_field_t> sMergeFields = { RGL_FIELD_XYZ_F32, RGL_FIELD_DISTANCE_F32 };

	std::vector<rgl_mat3x4f> rays = makeLidar3dRays(360, 180, 0.36, 0.18);
	rgl_mat3x4f lidarPoseTf = Mat3x4f::TRS({0, 0, -5}).toRGL();
	rgl_mat3x4f zeroTf = Mat3x4f::TRS({0, 0, 0}).toRGL();
	rgl_mat3x4f translateYTf = Mat3x4f::TRS({0, 3, 0}).toRGL();

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&lidarPose, &lidarPoseTf));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr, 1000));
	EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compact));
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPtsZero, &zeroTf));
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPtsY, &translateYTf));
	EXPECT_RGL_SUCCESS(rgl_node_points_spatial_merge(&spatialMerge, sMergeFields.data(), sMergeFields.size()));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, lidarPose));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(lidarPose, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, compact));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compact, transformPtsZero));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compact, transformPtsY));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transformPtsZero, spatialMerge));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transformPtsY, spatialMerge));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));
#ifdef RGL_BUILD_PCL_EXTENSION
	EXPECT_RGL_SUCCESS(rgl_graph_write_pcd_file(spatialMerge, "two_boxes_spatial_merge.pcd"));
#else
	RGL_WARN("RGL compiled without PCL extension. Tests will not save PCD!");
#endif
}

TEST_F(GraphCase, SpatialMergeFromRaytraces)
{
	// Setup cube scene
	auto mesh = makeCubeMesh();
	auto entity = makeEntity(mesh);
	rgl_mat3x4f entityPoseTf = Mat3x4f::identity().toRGL();
	ASSERT_RGL_SUCCESS(rgl_entity_set_pose(entity, &entityPoseTf));

	constexpr int LIDAR_FOV_Y = 40;
	constexpr int LIDAR_ROTATION_STEP = LIDAR_FOV_Y / 2;  // Make laser overlaps to validate merging

	std::vector<rgl_mat3x4f> rays = makeLidar3dRays(180, LIDAR_FOV_Y, 0.18, 1);

	// Lidars will be located in the cube center with different rotations covering all the space.
	std::vector<rgl_mat3x4f> lidarTfs;
	for (int i = 0; i < 360 / LIDAR_ROTATION_STEP; ++i) {
		lidarTfs.emplace_back(Mat3x4f::TRS({0, 0, 0}, {0, LIDAR_ROTATION_STEP * i, 0}).toRGL());
	}

	rgl_node_t spatialMerge=nullptr;
	std::vector<rgl_field_t> sMergeFields = { RGL_FIELD_XYZ_F32, RGL_FIELD_DISTANCE_F32 };
	EXPECT_RGL_SUCCESS(rgl_node_points_spatial_merge(&spatialMerge, sMergeFields.data(), sMergeFields.size()));

	for (auto& lidarTf : lidarTfs) {
		rgl_node_t lidarRays = nullptr;
		rgl_node_t lidarRaysTf = nullptr;
		rgl_node_t raytrace = nullptr;

		EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&lidarRays, rays.data(), rays.size()));
		EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&lidarRaysTf, &lidarTf));
		EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr, 1000));

		EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(lidarRays, lidarRaysTf));
		EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(lidarRaysTf, raytrace));
		EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, spatialMerge));
	}

	EXPECT_RGL_SUCCESS(rgl_graph_run(spatialMerge));
#ifdef RGL_BUILD_PCL_EXTENSION
	EXPECT_RGL_SUCCESS(rgl_graph_write_pcd_file(spatialMerge, "cube_spatial_merge.pcd"));
#else
	RGL_WARN("RGL compiled without PCL extension. Tests will not save PCD!");
#endif
}

TEST_F(GraphCase, TemporalMerge)
{
	auto mesh = makeCubeMesh();

	auto entity = makeEntity(mesh);
	rgl_mat3x4f entityPoseTf = Mat3x4f::identity().toRGL();
	ASSERT_RGL_SUCCESS(rgl_entity_set_pose(entity, &entityPoseTf));

	rgl_node_t useRays=nullptr, raytrace=nullptr, lidarPose=nullptr, compact=nullptr, transformPts=nullptr;
	rgl_node_t temporalMerge=nullptr;
	std::vector<rgl_field_t> tMergeFields = { RGL_FIELD_XYZ_F32, RGL_FIELD_DISTANCE_F32 };

	std::vector<rgl_mat3x4f> rays = makeLidar3dRays(360, 180, 0.36, 0.18);
	rgl_mat3x4f lidarPoseTf = Mat3x4f::TRS({0, 0, -5}).toRGL();
	rgl_mat3x4f zeroTf = Mat3x4f::TRS({0, 0, 0}).toRGL();
	rgl_mat3x4f translateYTf = Mat3x4f::TRS({0, 3, 0}).toRGL();

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&lidarPose, &lidarPoseTf));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr, 1000));
	EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compact));
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPts, &zeroTf));
	EXPECT_RGL_SUCCESS(rgl_node_points_temporal_merge(&temporalMerge, tMergeFields.data(), tMergeFields.size()));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, lidarPose));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(lidarPose, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, compact));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compact, transformPts));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transformPts, temporalMerge));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	// Change transform for the next raytrace
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPts, &translateYTf));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));
#ifdef RGL_BUILD_PCL_EXTENSION
	EXPECT_RGL_SUCCESS(rgl_graph_write_pcd_file(temporalMerge, "two_boxes_temporal_merge.pcd"));
#else
	RGL_WARN("RGL compiled without PCL extension. Tests will not save PCD!");
#endif
}

TEST_F(GraphCase, FormatNodeResults)
{
	auto mesh = makeCubeMesh();

	auto entity = makeEntity(mesh);
	rgl_mat3x4f entityPoseTf = Mat3x4f::identity().toRGL();
	ASSERT_RGL_SUCCESS(rgl_entity_set_pose(entity, &entityPoseTf));

	rgl_node_t useRays=nullptr, raytrace=nullptr, lidarPose=nullptr, format=nullptr;

	// The cube located in 0,0,0 with width equals 1, rays shoot in perpendicular direction
	constexpr float EXPECTED_HITPOINT_Z = 1.0f;
	constexpr float EXPECTED_RAY_DISTANCE = 1.0f;
	std::vector<rgl_mat3x4f> rays = {
		Mat3x4f::TRS({0, 0, 0}).toRGL(),
		Mat3x4f::TRS({0.1, 0, 0}).toRGL(),
		Mat3x4f::TRS({0.2, 0, 0}).toRGL(),
		Mat3x4f::TRS({0.3, 0, 0}).toRGL(),
		Mat3x4f::TRS({0.4, 0, 0}).toRGL()
	};
	rgl_mat3x4f lidarPoseTf = Mat3x4f::identity().toRGL();
	std::vector<rgl_field_t> formatFields = {
		XYZ_F32,
		PADDING_32,
		TIME_STAMP_F64
	};
	struct FormatStruct
	{
		Field<XYZ_F32>::type xyz;
		Field<PADDING_32>::type padding;
		Field<TIME_STAMP_F64>::type timestamp;
	} formatStruct;

	Time timestamp = Time::seconds(1.5);
	EXPECT_RGL_SUCCESS(rgl_scene_set_time(nullptr, timestamp.asNanoseconds()));

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&lidarPose, &lidarPoseTf));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr, 1000));
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&format, formatFields.data(), formatFields.size()));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, lidarPose));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(lidarPose, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, format));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	int32_t outCount, outSizeOf;
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(format, RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));

	EXPECT_EQ(outCount, rays.size());
	EXPECT_EQ(outSizeOf, sizeof(formatStruct));

	std::vector<FormatStruct> formatData(outCount);
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(format, RGL_FIELD_DYNAMIC_FORMAT, formatData.data()));

	for (int i = 0; i < formatData.size(); ++i) {
		EXPECT_NEAR(formatData[i].xyz[0], rays[i].value[0][3], EPSILON_F);
		EXPECT_NEAR(formatData[i].xyz[1], rays[i].value[1][3], EPSILON_F);
		EXPECT_NEAR(formatData[i].xyz[2], EXPECTED_HITPOINT_Z, EPSILON_F);
		EXPECT_EQ(formatData[i].timestamp, timestamp.asSeconds());
	}

	// Test if fields update is propagated over graph properly
	formatFields.push_back(DISTANCE_F32);  // Add distance field
	formatFields.push_back(PADDING_32);  // Align to 8 bytes
	struct FormatStructExtended : public  FormatStruct
	{
		Field<DISTANCE_F32>::type distance;
		Field<PADDING_32>::type padding2;  // Align to 8 bytes
	} formatStructEx;

	EXPECT_RGL_SUCCESS(rgl_node_points_format(&format, formatFields.data(), formatFields.size()));
	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	outCount = -1;  // reset variables
	outSizeOf = -1;
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(format, RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));
	EXPECT_EQ(outCount, rays.size());
	EXPECT_EQ(outSizeOf, sizeof(formatStructEx));

	std::vector<FormatStructExtended> formatDataEx{(size_t)outCount};
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(format, RGL_FIELD_DYNAMIC_FORMAT, formatDataEx.data()));

	for (int i = 0; i < formatDataEx.size(); ++i) {
		EXPECT_NEAR(formatDataEx[i].xyz[0], rays[i].value[0][3], EPSILON_F);
		EXPECT_NEAR(formatDataEx[i].xyz[1], rays[i].value[1][3], EPSILON_F);
		EXPECT_NEAR(formatDataEx[i].xyz[2], EXPECTED_HITPOINT_Z, EPSILON_F);
		EXPECT_NEAR(formatDataEx[i].distance, EXPECTED_RAY_DISTANCE, EPSILON_F);
		EXPECT_EQ(formatDataEx[i].timestamp, timestamp.asSeconds());
	}
}

/**
 * Temporal merging can lead to exceptionally high memory consumption.
 * In majority of cases CPU memory is much larger than GPU memory,
 * therefore TemporalMergeNode should use by default CPU memory.
 *
 * This test uses CUDA API to get current total memory usage of the GPU,
 * which makes this test inevitably prone to random fluctuations.
 *
 * In the future this might fixed by wrapping all CUDA allocations and exposing API to internally track memory usage.
 *
 * In RGL API we could give user control over which memory is used by this node.
 * However, this raises question, why not allow it for all nodes, which
 * summons another jungle of complexity, which I'd prefer to avoid until it is really necessary.
 */
TEST_F(GraphCase, TemporalMergeUsesHostMemory)
{
	const int32_t STEPS = 4;
	const int32_t BYTES_PER_STEP = (32 * 1024 * 1024);
	const int32_t allowedAllocatedBytes = (32 * 1024 * 1024); // Allow fluctuations much smaller than Node's memory needs.

	// Graph variables
	rgl_node_t usePoints = nullptr;
	rgl_node_t temporalMerge = nullptr;
	rgl_field_t fields[] = {RGL_FIELD_RETURN_TYPE_U8};
	int32_t fieldCount = ARRAY_SIZE(fields);
	std::vector<char> data(BYTES_PER_STEP);

	// Setup graph
	rgl_node_points_from_array(&usePoints, data.data(), BYTES_PER_STEP, fields, fieldCount);
	rgl_node_points_temporal_merge(&temporalMerge, fields, fieldCount);
	rgl_graph_node_add_child(usePoints, temporalMerge);

	int64_t freeMemAfterPrevStep = 0;
	int64_t allocatedBytes = 0;
	CHECK_CUDA(cudaMemGetInfo(reinterpret_cast<size_t*>(&freeMemAfterPrevStep), nullptr));
	for (int i = 0; i < STEPS; ++i) {
		rgl_graph_run(usePoints);
		int64_t currentFreeMem = 0;
		CHECK_CUDA(cudaMemGetInfo(reinterpret_cast<size_t*>(&currentFreeMem), nullptr));
		allocatedBytes += (freeMemAfterPrevStep - currentFreeMem);
		freeMemAfterPrevStep = currentFreeMem;
	}

	if (allocatedBytes > allowedAllocatedBytes) {
		FAIL() << fmt::format("TemporalMergeNode seems to allocate GPU memory (allocated {} b)", allocatedBytes);
	}
}
