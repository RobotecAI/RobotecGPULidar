#include <RGLFields.hpp>
#include <scenes.hpp>
#include <random>

constexpr float RAYTRACE_DEPTH = 1000;
constexpr float CUBE_HALF_EDGE = 1.0;
constexpr int ANGLE_ITERATIONS = 100;
constexpr float CUBE_Z_DISTANCE = 100.0f;
constexpr auto TEST_SEED_ENV_VAR = "RGL_FORMAT_TEST_SEED";

const rgl_mat3x4f cubePoseTf = Mat3x4f::translation(0, 0, CUBE_Z_DISTANCE).toRGL();

struct FormatPointsNodeHelper {
    rgl_node_t useRaysNode = nullptr, raytraceNode = nullptr;
    std::vector<rgl_node_t> formatNodes = { nullptr, nullptr, nullptr, nullptr, nullptr };

    std::vector<rgl_mat3x4f> rayTf;

    rgl_mesh_t cubeMesh;
    rgl_entity_t cubeEntity;

    std::random_device randomDevice;
    unsigned randomSeed;
    std::mt19937 randomGenerator;

    FormatPointsNodeHelper()
    {
        char* envSeed = std::getenv(TEST_SEED_ENV_VAR);
        if (envSeed != nullptr) {
            randomSeed = std::stoul(envSeed);
        } else {
            randomSeed = randomDevice();
        }
        randomGenerator.seed(randomSeed);
    }

    void prepareSceneWithCube()
    {
        cubeMesh = makeCubeMesh();
        EXPECT_RGL_SUCCESS(rgl_entity_create(&cubeEntity, nullptr, cubeMesh));
        EXPECT_RGL_SUCCESS(rgl_entity_set_pose(cubeEntity, &cubePoseTf));
    }

    void generateAngles(float maxAngle, float deviation = 0.0f)
    {
        // Generate angle and print the seed
        std::uniform_real_distribution<> dis(-maxAngle - deviation, maxAngle + deviation);
        std::cerr << "FormatPointsNodeTests various_angle seed: " << randomSeed << std::endl;

        rayTf.reserve(ANGLE_ITERATIONS);

        for (int i = 0; i < ANGLE_ITERATIONS; ++i) {
            float randomAngle = dis(randomGenerator);
            rayTf.emplace_back(Mat3x4f::rotationRad(RGL_AXIS_X, randomAngle).toRGL());
        }
    }

    std::vector<std::vector<rgl_field_t>> formats = {
        { XYZ_F32, IS_HIT_I32, DISTANCE_F32 }, // Point
        { XYZ_F32, PADDING_32, IS_HIT_I32, DISTANCE_F32 }, // Point1
        { XYZ_F32, PADDING_32, IS_HIT_I32, PADDING_32, DISTANCE_F32 }, // Point2
        { XYZ_F32, PADDING_32, IS_HIT_I32, PADDING_32, DISTANCE_F32, PADDING_32 }, // Point3
        { PADDING_32, XYZ_F32, PADDING_32, IS_HIT_I32, PADDING_32, DISTANCE_F32, PADDING_32 } // Point4
    };

    struct Point {
        ::Field<XYZ_F32>::type xyz;
        ::Field<IS_HIT_I32>::type isHit;
        ::Field<DISTANCE_F32>::type distance;
    };

    struct Point1 {
        ::Field<XYZ_F32>::type xyz;
        ::Field<PADDING_32>::type padding;
        ::Field<IS_HIT_I32>::type isHit;
        ::Field<DISTANCE_F32>::type distance;
    };

    struct Point2 {
        ::Field<XYZ_F32>::type xyz;
        ::Field<PADDING_32>::type padding;
        ::Field<IS_HIT_I32>::type isHit;
        ::Field<PADDING_32>::type padding2;
        ::Field<DISTANCE_F32>::type distance;
    };

    struct Point3 {
        ::Field<XYZ_F32>::type xyz;
        ::Field<PADDING_32>::type padding;
        ::Field<IS_HIT_I32>::type isHit;
        ::Field<PADDING_32>::type padding2;
        ::Field<DISTANCE_F32>::type distance;
        ::Field<PADDING_32>::type padding3;
    };

    struct Point4 {
        ::Field<PADDING_32>::type padding;
        ::Field<XYZ_F32>::type xyz;
        ::Field<PADDING_32>::type padding2;
        ::Field<IS_HIT_I32>::type isHit;
        ::Field<PADDING_32>::type padding3;
        ::Field<DISTANCE_F32>::type distance;
        ::Field<PADDING_32>::type padding4;
    };
};