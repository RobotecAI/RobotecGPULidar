#include <filesystem>

#include "helpers/sceneHelpers.hpp"
#include "helpers/commonHelpers.hpp"
#include "helpers/geometryData.hpp"
#include "helpers/textureHelpers.hpp"

#include "RGLFields.hpp"
#include "rgl/api/extensions/tape.h"
#include "math/Mat3x4f.hpp"
#include "tape/tapeDefinitions.hpp"

#if RGL_BUILD_PCL_EXTENSION
#include "rgl/api/extensions/pcl.h"
#endif

#if RGL_BUILD_ROS2_EXTENSION
#include "rgl/api/extensions/ros2.h"
#endif

#include <fstream>
#include <yaml-cpp/emitter.h>

const std::string YAML_EXT = ".yaml";
const std::string BIN_EXT = ".bin";

class TapeTest : public RGLTest
{
protected:
	std::string createTempFilePath(const std::string& baseName, const std::string& extension)
	{
		return (std::filesystem::temp_directory_path() / std::filesystem::path(baseName).concat(extension)).string();
	}

	void createEmptyFile(const std::string& path)
	{
		std::ofstream fileStream(path);
		if (!fileStream) {
			throw std::runtime_error("Failed to create file: " + path);
		}
		fileStream.close();
	}

	void createYAMLFile(const std::string& path, const YAML::Emitter& out)
	{
		std::ofstream fileYAMLStream(path);
		if (!fileYAMLStream) {
			throw std::runtime_error("Failed to create YAML file: " + path);
		}
		fileYAMLStream << out.c_str();
		fileYAMLStream.close();
	}
};

TEST_F(TapeTest, EmptyYAML)
{
	std::string yamlPath = createTempFilePath("empty", YAML_EXT);
	std::string binPath = createTempFilePath("empty", BIN_EXT);
	std::string recordPath = createTempFilePath("empty", "");

	createEmptyFile(yamlPath);
	createEmptyFile(binPath);

	EXPECT_RGL_TAPE_ERROR(rgl_tape_play(recordPath.c_str()), "Invalid Tape: Empty YAML file detected");
}

TEST_F(TapeTest, WithoutVersionYAML)
{
	std::string yamlPath = createTempFilePath("withoutVersion", YAML_EXT);
	std::string binPath = createTempFilePath("withoutVersion", BIN_EXT);
	std::string recordPath = createTempFilePath("withoutVersion", "");

	YAML::Emitter out;
	// clang-format off
	out << YAML::BeginSeq
	    	<< YAML::BeginMap << YAML::Key << "rgl_mesh_create" << YAML::Flow
	    		<< YAML::BeginMap
					<< YAML::Key << "t" << YAML::Value << 0
					<< YAML::Key << "a" << YAML::Value << YAML::Flow << YAML::BeginSeq << 0 << YAML::EndSeq
				<< YAML::EndMap
			<< YAML::EndMap
	    << YAML::EndSeq;
	// clang-format on

	createYAMLFile(yamlPath, out);
	createEmptyFile(binPath);

	EXPECT_RGL_TAPE_ERROR(rgl_tape_play(recordPath.c_str()), "Unsupported Tape format: Missing version record in the Tape");
}

TEST_F(TapeTest, OutdatedVersionYAML)
{
	std::string yamlPath = createTempFilePath("outdatedVersion", YAML_EXT);
	std::string binPath = createTempFilePath("outdatedVersion", BIN_EXT);
	std::string recordPath = createTempFilePath("outdatedVersion", "");

	YAML::Emitter out;
	// clang-format off
	out << YAML::BeginSeq
			<< YAML::BeginMap << YAML::Key << "rgl_get_version_info" << YAML::Flow
				<< YAML::BeginMap
					<< YAML::Key << "t" << YAML::Value << 0
					<< YAML::Key << "a" << YAML::Value << YAML::Flow
						<< YAML::BeginSeq
							<< RGL_TAPE_FORMAT_VERSION_MAJOR
							<< RGL_TAPE_FORMAT_VERSION_MINOR
							<< RGL_TAPE_FORMAT_VERSION_PATCH - 1
						<< YAML::EndSeq
				<< YAML::EndMap
			<< YAML::EndMap
		<< YAML::EndSeq;
	// clang-format on

	createYAMLFile(yamlPath, out);
	createEmptyFile(binPath);

	EXPECT_RGL_TAPE_ERROR(
	    rgl_tape_play(recordPath.c_str()),
	    "Unsupported Tape Format: Tape version is too old. Required version: " + std::to_string(RGL_TAPE_FORMAT_VERSION_MAJOR) +
	        "." + std::to_string(RGL_TAPE_FORMAT_VERSION_MINOR) + "." + std::to_string(RGL_TAPE_FORMAT_VERSION_PATCH) + ".");
}

TEST_F(TapeTest, OutdatedStructureYAML)
{
	std::string yamlPath = createTempFilePath("outdatedStructure", YAML_EXT);
	std::string binPath = createTempFilePath("outdatedStructure", BIN_EXT);
	std::string recordPath = createTempFilePath("outdatedStructure", "");

	YAML::Emitter out;
	// clang-format off
	out << YAML::BeginMap << YAML::Key << "rgl_version" << YAML::Value
	    	<< YAML::BeginMap
	    		<< YAML::Key << "major" << YAML::Value << 0
	    		<< YAML::Key << "minor" << YAML::Value << 16
	    		<< YAML::Key << "patch" << YAML::Value << 2
	    	<< YAML::EndMap
		<< YAML::EndMap;
	// clang-format on

	createYAMLFile(yamlPath, out);
	createEmptyFile(binPath);

	EXPECT_RGL_TAPE_ERROR(rgl_tape_play(recordPath.c_str()), "Unsupported Tape format: Detected outdated format");
}

TEST_F(TapeTest, RecordPlayLoggingCall)
{
	std::string loggingRecordPath{(std::filesystem::temp_directory_path() / std::filesystem::path("loggingRecord")).string()};
	std::string logFilePath{
	    (std::filesystem::temp_directory_path() / std::filesystem::path("loggingRecord").concat(".log")).string()};

	// Logging with Tape
	ASSERT_RGL_SUCCESS(rgl_tape_record_begin(loggingRecordPath.c_str()));
	bool isTapeRecordActive = false;
	ASSERT_RGL_SUCCESS(rgl_tape_record_is_active(&isTapeRecordActive));
	ASSERT_TRUE(isTapeRecordActive);

	EXPECT_RGL_SUCCESS(rgl_configure_logging(RGL_LOG_LEVEL_DEBUG, logFilePath.c_str(), false));

	EXPECT_RGL_SUCCESS(rgl_tape_record_end());
	EXPECT_RGL_SUCCESS(rgl_tape_record_is_active(&isTapeRecordActive));
	EXPECT_FALSE(isTapeRecordActive);
	EXPECT_RGL_SUCCESS(rgl_tape_play(loggingRecordPath.c_str()));
}

TEST_F(TapeTest, RecordPlayAllCalls)
{
	std::string allCallsRecordPath{(std::filesystem::temp_directory_path() / std::filesystem::path("allCallsRecord")).string()};
	EXPECT_RGL_SUCCESS(rgl_tape_record_begin(allCallsRecordPath.c_str()));
	bool isTapeRecordActive = false;
	EXPECT_RGL_SUCCESS(rgl_tape_record_is_active(&isTapeRecordActive));
	EXPECT_TRUE(isTapeRecordActive);

	rgl_mat3x4f identityTf = Mat3x4f::identity().toRGL();

	int32_t major, minor, patch;
	EXPECT_RGL_SUCCESS(rgl_get_version_info(&major, &minor, &patch));

	// Note: the logging using tape test has been moved to another test case (RecordPlayLoggingCall) to not change the logging level of this test case.

	rgl_mesh_t mesh = nullptr;
	EXPECT_RGL_SUCCESS(rgl_mesh_create(&mesh, cubeVertices, ARRAY_SIZE(cubeVertices), cubeIndices, ARRAY_SIZE(cubeIndices)));
	EXPECT_RGL_SUCCESS(rgl_mesh_update_vertices(mesh, cubeVertices, ARRAY_SIZE(cubeVertices)));

	rgl_entity_t entity = nullptr;
	EXPECT_RGL_SUCCESS(rgl_entity_create(&entity, nullptr, mesh));
	EXPECT_RGL_SUCCESS(rgl_entity_set_pose(entity, &identityTf));
	EXPECT_RGL_SUCCESS(rgl_entity_set_id(entity, 1));

	rgl_texture_t texture = nullptr;
	int width = 1024;
	int height = 2048;
	auto textureRawData = generateCheckerboardTexture<TextureTexelFormat>(width, height);

	EXPECT_RGL_SUCCESS(rgl_texture_create(&texture, textureRawData.data(), width, height));
	EXPECT_RGL_SUCCESS(rgl_mesh_set_texture_coords(mesh, cubeUVs, 8));
	EXPECT_RGL_SUCCESS(rgl_entity_set_intensity_texture(entity, texture));

	EXPECT_RGL_SUCCESS(rgl_scene_set_time(nullptr, 1.5 * 1e9));

	rgl_node_t useRays = nullptr;
	std::vector<rgl_mat3x4f> rays = {identityTf, identityTf};
	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, rays.data(), rays.size()));

	rgl_node_t setRingIds = nullptr;
	std::vector<int> rings = {0, 1};
	EXPECT_RGL_SUCCESS(rgl_node_rays_set_ring_ids(&setRingIds, rings.data(), rings.size()));

	rgl_node_t setTimeOffsets = nullptr;
	std::vector<float> timeOffsets = {0.0f, 1.0f, 2.0f};
	EXPECT_RGL_SUCCESS(rgl_node_rays_set_time_offsets(&setTimeOffsets, timeOffsets.data(), timeOffsets.size()));

	rgl_node_t setRange = nullptr;
	rgl_vec2f range = {0.0f, 1.0f};
	EXPECT_RGL_SUCCESS(rgl_node_rays_set_range(&setRange, &range, 1));

	rgl_node_t transformRays = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&transformRays, &identityTf));

	rgl_node_t transformPoints = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPoints, &identityTf));

	rgl_node_t raytrace = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr));

	rgl_node_t raytraceWithDistortion = nullptr;
	rgl_vec3f linearVelocity{1.0f, 2.0f, 3.0f};
	rgl_vec3f angularVelocity{1.0f, 2.0f, 3.0f};
	EXPECT_RGL_SUCCESS(rgl_node_raytrace_with_distortion(&raytraceWithDistortion, nullptr, &linearVelocity, &angularVelocity));

	rgl_node_t raytraceInMotion = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_raytrace_in_motion(&raytraceInMotion, nullptr, &linearVelocity, &angularVelocity, false));

	rgl_node_t format = nullptr;
	std::vector<rgl_field_t> fields = {RGL_FIELD_XYZ_VEC3_F32, RGL_FIELD_DISTANCE_F32};
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&format, fields.data(), fields.size()));

	rgl_node_t yield = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&yield, fields.data(), fields.size()));

	rgl_node_t compact = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compact));

	rgl_node_t compactByField = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_points_compact_by_field(&compactByField, IS_HIT_I32));

	rgl_node_t spatialMerge = nullptr;
	std::vector<rgl_field_t> sMergeFields = {RGL_FIELD_XYZ_VEC3_F32, RGL_FIELD_DISTANCE_F32, RGL_FIELD_PADDING_32};
	EXPECT_RGL_SUCCESS(rgl_node_points_spatial_merge(&spatialMerge, sMergeFields.data(), sMergeFields.size()));

	rgl_node_t temporalMerge = nullptr;
	std::vector<rgl_field_t> tMergeFields = {RGL_FIELD_XYZ_VEC3_F32, RGL_FIELD_DISTANCE_F32, RGL_FIELD_PADDING_32};
	EXPECT_RGL_SUCCESS(rgl_node_points_temporal_merge(&temporalMerge, tMergeFields.data(), tMergeFields.size()));

	rgl_node_t usePoints = nullptr;
	std::vector<rgl_field_t> usePointsFields = {RGL_FIELD_XYZ_VEC3_F32};
	std::vector<::Field<XYZ_VEC3_F32>::type> usePointsData = {
	    {1, 2, 3},
        {4, 5, 6}
    };
	EXPECT_RGL_SUCCESS(rgl_node_points_from_array(&usePoints, usePointsData.data(), usePointsData.size(),
	                                              usePointsFields.data(), usePointsFields.size()));

	rgl_node_t radarPostprocess = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_points_radar_postprocess(&radarPostprocess, 1.0f, 0.5f));

	rgl_node_t filterGround = nullptr;
	rgl_vec3f sensorUpVector = {0.0f, 1.0f, 0.0f};
	EXPECT_RGL_SUCCESS(rgl_node_points_filter_ground(&filterGround, &sensorUpVector, 0.1f));

	rgl_node_t compactByFieldGround = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_points_compact_by_field(&compactByFieldGround, IS_GROUND_I32));

#if RGL_BUILD_ROS2_EXTENSION
	rgl_node_t pointcloud2Pub = nullptr, radarscanPub = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish(&pointcloud2Pub, "pointcloud", "rgl"));

	rgl_node_t ros2pubqos = nullptr;
	rgl_qos_policy_reliability_t qos_r = QOS_POLICY_RELIABILITY_BEST_EFFORT;
	rgl_qos_policy_durability_t qos_d = QOS_POLICY_DURABILITY_VOLATILE;
	rgl_qos_policy_history_t qos_h = QOS_POLICY_HISTORY_KEEP_LAST;
	EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish_with_qos(&ros2pubqos, "pointcloud_ex", "rgl", qos_r, qos_d, qos_h, 10));
	EXPECT_RGL_SUCCESS(rgl_node_publish_ros2_radarscan(&radarscanPub, "radarscan", "rgl", qos_r, qos_d, qos_h, 10));
#endif

	rgl_node_t noiseAngularRay = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_angular_ray(&noiseAngularRay, 0.1f, 0.1f, RGL_AXIS_X));

	rgl_node_t noiseAngularHitpoint = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_angular_hitpoint(&noiseAngularHitpoint, 0.1f, 0.1f, RGL_AXIS_X));

	rgl_node_t noiseDistance = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_distance(&noiseDistance, 0.1f, 0.1f, 0.01f));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, setRange));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(setRange, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, format));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, compactByField));

	EXPECT_RGL_SUCCESS(rgl_graph_node_remove_child(raytrace, compactByField));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, filterGround));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(filterGround, compactByFieldGround));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

#if RGL_BUILD_PCL_EXTENSION
	rgl_node_t downsample = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_points_downsample(&downsample, 1.0f, 1.0f, 1.0f));

	// Have to be executed after rgl_graph_run. Graph::run() must set field XYZ_VEC3_F32 in raytrace.
	EXPECT_RGL_SUCCESS(rgl_graph_write_pcd_file(raytrace, "Tape.RecordPlayAllCalls.pcd"));

	rgl_node_t removeGround = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_points_remove_ground(&removeGround, RGL_AXIS_X, 1.0f, 1.0f, 0.5f));

	// Skipping rgl_node_points_visualize (user interaction needed)
#endif

	int32_t outCount, outSizeOf;
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(format, RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));

	std::vector<char> tmpVec;
	tmpVec.reserve(outCount * outSizeOf);
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(format, RGL_FIELD_DYNAMIC_FORMAT, tmpVec.data()));

	EXPECT_RGL_SUCCESS(rgl_graph_destroy(setRingIds));
	EXPECT_RGL_SUCCESS(rgl_entity_destroy(entity));
	EXPECT_RGL_SUCCESS(rgl_mesh_destroy(mesh));
	EXPECT_RGL_SUCCESS(rgl_texture_destroy(texture));

	EXPECT_RGL_SUCCESS(rgl_cleanup());

	EXPECT_RGL_SUCCESS(rgl_tape_record_end());

	EXPECT_RGL_SUCCESS(rgl_tape_record_is_active(&isTapeRecordActive));
	EXPECT_FALSE(isTapeRecordActive);

	EXPECT_RGL_SUCCESS(rgl_tape_play(allCallsRecordPath.c_str()));
}

void testCubeSceneOnGraph()
{
	rgl_node_t useRays = nullptr, raytrace = nullptr, lidarPose = nullptr, format = nullptr;

	std::vector<rgl_mat3x4f> rays = {Mat3x4f::TRS({0, 0, 0}).toRGL(), Mat3x4f::TRS({0.1, 0, 0}).toRGL(),
	                                 Mat3x4f::TRS({0.2, 0, 0}).toRGL(), Mat3x4f::TRS({0.3, 0, 0}).toRGL(),
	                                 Mat3x4f::TRS({0.4, 0, 0}).toRGL()};
	rgl_mat3x4f lidarPoseTf = Mat3x4f::identity().toRGL();
	std::vector<rgl_field_t> formatFields = {XYZ_VEC3_F32, PADDING_32};

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&lidarPose, &lidarPoseTf));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr));
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&format, formatFields.data(), formatFields.size()));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, lidarPose));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(lidarPose, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, format));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	int32_t outCount, outSizeOf;
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(format, RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));

	struct FormatStruct
	{
		Field<XYZ_VEC3_F32>::type xyz;
		Field<PADDING_32>::type padding;
	} formatStruct;

	EXPECT_EQ(outCount, rays.size());
	EXPECT_EQ(outSizeOf, sizeof(formatStruct));

	std::vector<FormatStruct> formatData{static_cast<long unsigned int>(outCount)};
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(format, RGL_FIELD_DYNAMIC_FORMAT, formatData.data()));

	for (int i = 0; i < formatData.size(); ++i) {
		EXPECT_NEAR(formatData[i].xyz[0], rays[i].value[0][3], 1e-6);
		EXPECT_NEAR(formatData[i].xyz[1], rays[i].value[1][3], 1e-6);
		EXPECT_NEAR(formatData[i].xyz[2], 1, 1e-6);
	}
}

TEST_F(TapeTest, SceneReconstruction)
{
	std::string cubeSceneRecordPath{
	    (std::filesystem::temp_directory_path() / std::filesystem::path("cubeSceneRecord")).string()};
	rgl_tape_record_begin(cubeSceneRecordPath.c_str());
	auto mesh = makeCubeMesh();
	auto entity = makeEntity(mesh);
	rgl_mat3x4f entityPoseTf = Mat3x4f::identity().toRGL();
	ASSERT_RGL_SUCCESS(rgl_entity_set_pose(entity, &entityPoseTf));
	rgl_tape_record_end();

	testCubeSceneOnGraph();

	rgl_cleanup();

	rgl_tape_play(cubeSceneRecordPath.c_str());

	testCubeSceneOnGraph();
}