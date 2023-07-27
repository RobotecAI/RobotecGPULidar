#include <scenes.hpp>
#include <utils.hpp>
#include <RGLFields.hpp>
#include <random>
// Do usuniecia v
#include <iomanip>

static constexpr float CUBE_HALF_EDGE = 1.0;

//Póki co tu zakładamy że ray jest skierowany i umieszczony defaultowo
std::pair<float, float> calculateAngles(const Vec3f targetPos)
{
    float horizAng = atanf(targetPos.x() / targetPos.z());
    float vertAng = -atanf(targetPos.y() / sqrt(pow(targetPos.x(), 2) + pow(targetPos.z(), 2)));

    return std::make_pair(vertAng, horizAng);
}

class RaytraceNodeTestImproved : public RGLTest {
protected:
    RaytraceNodeTestImproved()  
    {
        raytraceNode = nullptr;
        raysFromMatNode = nullptr;

        randomSeed = randomDevice();
        randomGenerator.seed(randomSeed);

        std::uniform_real_distribution<float> floatDistr(100.0f, 310.0f);

        // cubePosition = Vec3f(floatDistr(randomGenerator), floatDistr(randomGenerator), floatDistr(randomGenerator));
        cubePosition = Vec3f(2.0f, 233.0f, 2.0f);
    }

    void prepareScene()
    {
        const Mat3x4f cubeTf = Mat3x4f::translation(cubePosition.x(), cubePosition.y(), cubePosition.z());
        spawnCubeOnScene(nullptr, cubeTf);
    }

    void prepareGraph()
    {
        ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&raysFromMatNode, &rayTf, 1)); // TODO change hardcoded size
        ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, raytraceDepth));
        ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raysFromMatNode, raytraceNode));
    }

    void getResults()
    {
        ASSERT_RGL_SUCCESS(rgl_graph_run(raytraceNode));

        ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(raytraceNode, RGL_FIELD_XYZ_F32, &pointCount, &pointSize));
        ASSERT_EQ(pointCount, 1);
        ASSERT_EQ(pointSize, sizeof(Vec3f));

        ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(raytraceNode, RGL_FIELD_XYZ_F32, &point));
    }

    Vec3f cubePosition;
    rgl_mat3x4f rayTf = Mat3x4f::identity().toRGL();

    std::random_device randomDevice;
    unsigned randomSeed;
    std::mt19937 randomGenerator;

    rgl_node_t raytraceNode, raysFromMatNode;
    float raytraceDepth;
    int32_t pointCount, pointSize;
    Vec3f point;
};

TEST_F(RaytraceNodeTestImproved, invalid_argument_node)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_raytrace(nullptr, nullptr, 0.0f), "node != nullptr");
}

TEST_F(RaytraceNodeTestImproved, invalid_argument_range)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_raytrace(&raytraceNode, nullptr, 0.0f), "range > 0.0f");
}

TEST_F(RaytraceNodeTestImproved, valid_arguments)
{
    EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, 1.0f));
    ASSERT_THAT(raytraceNode, testing::NotNull());

    // If (*raytraceNode) != nullptr - 0.01f
    EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, 1.0f));
}

// TEST_F(RaytraceNodeTestImproved, empty_scene)
// {

//     // The test fails. It is commented because failure of rgl_cleanup affects other tests
//     FAIL();

//     // Potential solutions: 
//     // warning that the scene is empty and performing a raytracing 
//     // or error with the corresponding message that the scene is empty
//     // rgl_cleanup() should also return a warning or error with a better specified comment
// }

// // Raytrace depth test cases

// TEST_F(RaytraceNodeTestImproved, raytrace_misses_object_when_depth_is_shorter_than_distance)
// {
//     // TODO Magic number 0.01f
//     raytraceDepth = cubePosition.z() - CUBE_HALF_EDGE - 0.01f;
//     prepareScene();
//     prepareGraph();
    // vert, horiz
    std::vector<std::pair<float, float>> angles;
//     getResults();

//     EXPECT_EQ(point.x(), NON_HIT_VALUE);
//     EXPECT_EQ(point.y(), NON_HIT_VALUE);
//     EXPECT_EQ(point.z(), NON_HIT_VALUE);
// }

// TEST_F(RaytraceNodeTestImproved, raytrace_hits_object_when_depth_equals_distance)
// {
//     raytraceDepth = cubePosition.z() - CUBE_HALF_EDGE;
//     prepareScene();
//     prepareGraph();
//     getResults();

//     EXPECT_EQ(point.x(), cubePosition.x());
//     EXPECT_EQ(point.y(), cubePosition.y());
//     EXPECT_EQ(point.z(), cubePosition.z() - CUBE_HALF_EDGE);
// }

// TEST_F(RaytraceNodeTestImproved, raytrace_hits_object_when_depth_is_longer_than_distance)
// {
//     // TODO Magic number 0.01f
//     raytraceDepth = cubePosition.z() - CUBE_HALF_EDGE + 0.01f;
//     prepareScene();
//     prepareGraph();
//     getResults();

//     EXPECT_EQ(point.x(), cubePosition.x());
//     EXPECT_EQ(point.y(), cubePosition.y());
//     EXPECT_EQ(point.z(), cubePosition.z() - CUBE_HALF_EDGE);
// }

// // Different ray origins test cases
// TEST_F(RaytraceNodeTestImproved, raytrace_hits_when_ray_originates_at_cube_wall)
// {
//     raytraceDepth = 100.0f;
//     prepareScene();

//     rayTf = Mat3x4f::translation(0, 0, cubePosition.z() - CUBE_HALF_EDGE).toRGL();

//     prepareGraph();
//     getResults();

//     EXPECT_EQ(point.x(), cubePosition.x());
//     EXPECT_EQ(point.y(), cubePosition.y());
//     EXPECT_EQ(point.z(), cubePosition.z() + CUBE_HALF_EDGE);
// }

// TEST_F(RaytraceNodeTestImproved, raytrace_hits_when_ray_originates_within_cube)
// {
//     raytraceDepth = 100.0f;
//     prepareScene();

//     rayTf = Mat3x4f::translation(0, 0, cubePosition.z()).toRGL();
    
//     prepareGraph();
//     getResults();

//     EXPECT_EQ(point.x(), cubePosition.x());
//     EXPECT_EQ(point.y(), cubePosition.y());
//     EXPECT_EQ(point.z(), cubePosition.z() + CUBE_HALF_EDGE);
// }

TEST_F(RaytraceNodeTestImproved, raytrace_hits_when_ray_originates_at_distance_directed_at_cube_corner)
{
    raytraceDepth = sqrt(pow(cubePosition.x(), 2) + pow(cubePosition.y(), 2) + pow(cubePosition.z(), 2)) + CUBE_HALF_EDGE;

    // Corners of the cube
    Vec3f leftTop = cubePosition + Vec3f(CUBE_HALF_EDGE, CUBE_HALF_EDGE, -CUBE_HALF_EDGE);
    Vec3f leftBottom = cubePosition + Vec3f(CUBE_HALF_EDGE, -CUBE_HALF_EDGE, -CUBE_HALF_EDGE);
    Vec3f rightTop = cubePosition + Vec3f(-CUBE_HALF_EDGE, CUBE_HALF_EDGE, -CUBE_HALF_EDGE);
    Vec3f rightBottom = cubePosition + Vec3f(-CUBE_HALF_EDGE, -CUBE_HALF_EDGE, -CUBE_HALF_EDGE);

    prepareScene();

    std::pair<float, float> ltCornerRot = calculateAngles(leftTop);
    std::pair<float, float> lbCornerRot = calculateAngles(leftBottom);
    std::pair<float, float> rtCornerRot = calculateAngles(rightTop);
    std::pair<float, float> rbCornerRot = calculateAngles(rightBottom);

    std::cerr << "ltCornerRot: " << ltCornerRot.first << ", " << ltCornerRot.second << std::endl;
    std::cerr << "lbCornerRot: " << lbCornerRot.first << ", " << lbCornerRot.second << std::endl;
    std::cerr << "rtCornerRot: " << rtCornerRot.first << ", " << rtCornerRot.second << std::endl;
    std::cerr << "rbCornerRot: " << rbCornerRot.first << ", " << rbCornerRot.second << std::endl;

    // Add float error
    ltCornerRot.first += ((lbCornerRot.first - ltCornerRot.first) / 4);
    ltCornerRot.second -= ((ltCornerRot.second - rtCornerRot.second ) / 4);

    lbCornerRot.first -= ((lbCornerRot.first - ltCornerRot.first) / 4);
    lbCornerRot.second -= ((lbCornerRot.second - rbCornerRot.second ) / 4);

    rtCornerRot.first += ((rbCornerRot.first - rtCornerRot.first) / 4);
    rtCornerRot.second += ((ltCornerRot.second - rtCornerRot.second ) / 4);

    rbCornerRot.first -= ((rbCornerRot.first - rtCornerRot.first) / 4);
    rbCornerRot.second += ((lbCornerRot.second - rbCornerRot.second ) / 4);

    std::cerr << "ltCornerRot: " << ltCornerRot.first << ", " << ltCornerRot.second << std::endl;
    std::cerr << "lbCornerRot: " << lbCornerRot.first << ", " << lbCornerRot.second << std::endl;
    std::cerr << "rtCornerRot: " << rtCornerRot.first << ", " << rtCornerRot.second << std::endl;
    std::cerr << "rbCornerRot: " << rbCornerRot.first << ", " << rbCornerRot.second << std::endl;


    std::vector<rgl_mat3x4f> rays;

    float step = 0.0005f;

    // vert, horiz
    std::vector<std::pair<float, float>> angles;

   for (float i = -M_PI/2; i <= 0.0f; i += step)
    {
        for (float j = 0.0f; j <= M_PI/2; j += step)
        {
            rays.push_back(Mat3x4f::rotationRad(i, j, 0).toRGL());

            angles.push_back(std::make_pair(i, j));
        }
    }

    rays.push_back(Mat3x4f::rotationRad(ltCornerRot.first, ltCornerRot.second, 0).toRGL());
    angles.push_back(std::make_pair(ltCornerRot.first, ltCornerRot.second));
    rays.push_back(Mat3x4f::rotationRad(lbCornerRot.first, lbCornerRot.second, 0).toRGL());
    angles.push_back(std::make_pair(lbCornerRot.first, lbCornerRot.second));
    rays.push_back(Mat3x4f::rotationRad(rtCornerRot.first, rtCornerRot.second, 0).toRGL());
    angles.push_back(std::make_pair(rtCornerRot.first, rtCornerRot.second));
    // TODO - bez linijki niżej dobrze liczy min ver i hor ???
    rays.push_back(Mat3x4f::rotationRad(rbCornerRot.first, rbCornerRot.second, 0).toRGL());
    angles.push_back(std::make_pair(rbCornerRot.first, rbCornerRot.second));

    // TODO Put it into function (change parameters of function)
    ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&raysFromMatNode, rays.data(), rays.size()));
    ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, raytraceDepth));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raysFromMatNode, raytraceNode));

    ASSERT_RGL_SUCCESS(rgl_graph_run(raytraceNode));

    // TODO Put it into function (change parameters of function)
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(raytraceNode, RGL_FIELD_XYZ_F32, &pointCount, &pointSize));
    ASSERT_EQ(pointCount, rays.size());
    ASSERT_EQ(pointSize, sizeof(Vec3f));

    std::vector<Vec3f> points(pointCount);
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(raytraceNode, RGL_FIELD_XYZ_F32, points.data()));

    float errorMax = 100.0f;
    float errorMin = 100.0f;

    float max_x, max_y, min_x, min_y;
    int max_index = -1;
    int min_index = -1;

    for (int i = 0; i < pointCount; ++i) {
        float currentErr = -1.0f;
        double epsilon = 0.0000001;
        if( (points[i].x() != NON_HIT_VALUE) || (points[i].y() != NON_HIT_VALUE) && (points[i].z() - (cubePosition.z() - CUBE_HALF_EDGE) <= epsilon) )  
        {
            currentErr += std::abs(cubePosition.x() + CUBE_HALF_EDGE - points[i].x());
            currentErr += std::abs(cubePosition.y() + CUBE_HALF_EDGE - points[i].y());
            currentErr += std::abs(cubePosition.z() - CUBE_HALF_EDGE - points[i].z());

            if(currentErr < errorMax)
            {
                errorMax = currentErr;
                max_index = i;
                max_x = points[i].x();
                max_y = points[i].y();
            }
            
            currentErr = -1.0f;
            currentErr += std::abs(cubePosition.x() - CUBE_HALF_EDGE - points[i].x());
            currentErr += std::abs(cubePosition.y() - CUBE_HALF_EDGE - points[i].y());
            currentErr += std::abs(cubePosition.z() - CUBE_HALF_EDGE - points[i].z());

            if(currentErr < errorMin)
            {
                errorMin = currentErr;
                min_index = i;
                min_x = points[i].x();
                min_y = points[i].y();
            }
        }
    }

    fmt::print(stderr, "Raytrace Node Test random seed: {}\n", randomSeed);

    std::cerr << "===============================================================" << std::endl;
    std::cerr << "Min ver: " << angles[min_index].first << " min hor: " << angles[min_index].second << std::endl;
    std::cerr << "Point: " << points[min_index].x() << " " << points[min_index].y() << " " << points[min_index].z() << std::endl;
    std::cerr << "d: " << rbCornerRot.first << " " << rbCornerRot.second << std::endl;
    std::cerr << "d - min: " << rbCornerRot.first - angles[min_index].first << " " << rbCornerRot.second - angles[min_index].second << std::endl;
    std::cerr << "===============================================================" << std::endl;
    std::cerr << "Max vert: " << std::setprecision(7) << angles[max_index].first << " max hor: " << std::setprecision(7) << angles[max_index].second << std::endl;
    std::cerr << "Point: " << points[max_index].x() << " " << points[max_index].y() << " " << points[max_index].z() << std::endl;
    std::cerr << "a: " << std::setprecision(7) << ltCornerRot.first << " " << std::setprecision(7) << ltCornerRot.second << std::endl;
    std::cerr << "a - max: " << ltCornerRot.first - angles[max_index].first << " " << ltCornerRot.second - angles[max_index].second << std::endl; 
    std::cerr << "===============================================================" << std::endl;
    std::cerr << "a,b,c,d points: " << std::endl;
    std::cerr << "a: " << points[pointCount - 4].x() << " " << points[pointCount - 4].y() << " " << points[pointCount - 4].z() << std::endl;
    std::cerr << "b: " << points[pointCount - 3].x() << " " << points[pointCount - 3].y() << " " << points[pointCount - 3].z() << std::endl;
    std::cerr << "c: " << points[pointCount - 2].x() << " " << points[pointCount - 2].y() << " " << points[pointCount - 2].z() << std::endl;
    std::cerr << "d: " << points[pointCount - 1].x() << " " << points[pointCount - 1].y() << " " << points[pointCount - 1].z() << std::endl;
    std::cerr << "a,b,c,d angles: " << std::endl;
    std::cerr << "a: " << angles[pointCount - 4].first << " " << angles[pointCount - 4].second << std::endl;
    std::cerr << "b: " << angles[pointCount - 3].first << " " << angles[pointCount - 3].second << std::endl;
    std::cerr << "c: " << angles[pointCount - 2].first << " " << angles[pointCount - 2].second << std::endl;
    std::cerr << "d: " << angles[pointCount - 1].first << " " << angles[pointCount - 1].second << std::endl;
}

// TEST_F(RaytraceNodeTestImproved, raytrace_hits_when_ray_originates_at_distance_directed_at_cube_edge)
// {
//     // To implement
// }

// TEST_F(RaytraceNodeTestImproved, raytrace_hits_when_ray_originates_at_distance_directed_at_cube_face)
// {
//     // To implement
// }

// TEST_F(RaytraceNodeTestImproved, raytrace_misses_when_ray_is_parallel_to_cube_surface)
// {
//     raytraceDepth = 100.0f;
//     prepareScene();

//     std::vector<rgl_mat3x4f> rays;
//     rays.push_back(Mat3x4f::translation(0, -CUBE_HALF_EDGE, 0).toRGL());

//     ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&raysFromMatNode, rays.data(), rays.size()));
//     ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, raytraceDepth));
//     ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raysFromMatNode, raytraceNode));
//     ASSERT_RGL_SUCCESS(rgl_graph_run(raytraceNode));

//     ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(raytraceNode, RGL_FIELD_XYZ_F32, &pointCount, &pointSize));
//     ASSERT_EQ(pointCount, rays.size());
//     ASSERT_EQ(pointSize, sizeof(Vec3f));

//     std::vector<Vec3f> points(pointCount);
//     ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(raytraceNode, RGL_FIELD_XYZ_F32, points.data()));

//     EXPECT_EQ(points[0].x(), NON_HIT_VALUE);
//     EXPECT_EQ(points[0].y(), NON_HIT_VALUE);
//     EXPECT_EQ(points[0].z(), NON_HIT_VALUE);
// }

// TEST_F(RaytraceNodeTestImproved, raytrace_misses_when_ray_originates_behind_cube)
// {
//     // To implement
// }


// To check angles:

