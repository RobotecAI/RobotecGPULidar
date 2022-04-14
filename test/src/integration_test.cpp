#include <gtest/gtest.h>
#include "LidarRenderer.h"
#include "model_utils.h"

#include <iostream>


std::ostream& operator<<(std::ostream& os, const Point3f& p)
{
    os << "[" << p.x << ", " <<  p.y << ", " << p.z << "]";
    return os;
}

std::ostream& operator<<(std::ostream& os, const PCL12& p)
{
    os << "[" << p.x << ", " <<  p.y << ", " << p.z << "]";
    return os;
}


class GPULidarIntegration : public ::testing::Test {
protected:
    void SetUp() override {
        not_translate_ = new float[12] {
            1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f
        };
        translate_ = new float[12] {
            1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 5.0f,
            0.0f, 0.0f, 1.0f, 0.0f
        };

        ringIds_ = new int[2];
        ringIds_[0] = 10;
        ringIds_[1] = 111;
        ringIdsSize_ = 2;

        ml_.load_obj("cube.obj");
        std::shared_ptr<TriangleMesh> mesh = ml_.get_triangle_mesh(0);


        lr_.addMeshRaw("cube", mesh->vertex.size(), mesh->vertex.data(),
                      mesh->index.size(), mesh->index.data(), 12, not_translate_);
        lr_.addMeshRaw("cube_translated", mesh->vertex.size(), mesh->vertex.data(),
                      mesh->index.size(), mesh->index.data(), 12, translate_);
    }

    void TearDown() override {
        delete not_translate_;
        delete translate_;
    }

    LidarRenderer lr_;
    ModelLoader ml_;

    float* not_translate_;
    float* translate_;
    int* ringIds_;
    int ringIdsSize_;
};

TEST_F(GPULidarIntegration, SimpleRaycast)
{
    TransformMatrix lidarPose = {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, -30.0f
    };
    TransformMatrix rayPoses[2] = {{
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
    },{
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
    }};
    LidarContext lidarCtx(rayPoses, 2, ringIds_, ringIdsSize_);
    lidarCtx.lidarNoiseParams = {
        .angularNoiseType = AngularNoiseType::HITPOINT_BASED,
        .angularNoiseStDev = 0.0f,
        .angularNoiseMean = 0.00f,
        .distanceNoiseStDevBase = 0.0f,
        .distanceNoiseStDevRisePerMeter = 0.0f,
        .distanceNoiseMean = 0.0f
    };

    lr_.renderCtx(&lidarCtx, lidarPose, 100.0f);

    std::vector<Point3f> results(lr_.getResultPointCloudSizeCtx(&lidarCtx));
    lr_.downloadPointsCtx(&lidarCtx, results.size(), results.data());
    ASSERT_EQ(2, results.size());
    EXPECT_NEAR(results[0].x, 0.0f, 0.01f);
    EXPECT_NEAR(results[0].y, 0.0f, 0.01f);
    EXPECT_NEAR(results[0].z, -1.0f, 0.01f);
    std::cout << results[0] << std::endl;
}
//
// TEST_F(GPULidarIntegration, MovedUpRaycast) {
//     TransformMatrix lidarPose = {
//         1.0f, 0.0f, 0.0f, 0.0f,
//         0.0f, 1.0f, 0.0f, 0.0f,
//         0.0f, 0.0f, 1.0f, -30.0f
//     };
//     TransformMatrix sourcePoses = {
//         1.0f, 0.0f, 0.0f, 0.0f,
//         0.0f, 1.0f, 0.0f, 5.0f,
//         0.0f, 0.0f, 1.0f, 0.0f
//     };
//     LidarSource lidar_source("lidar", lidarPose, &sourcePoses, 1, ringIds_, ringIdsSize_, 100.0f);
//     lr_.render({lidar_source});
//     std::vector<Point3f> results(lr_.getNextDownloadPointCount());
//     lr_.downloadPoints(results.size(), results.data());
//     ASSERT_EQ(1, results.size());
//     EXPECT_NEAR(results[0].x, 0.0f, 0.01f);
//     EXPECT_NEAR(results[0].y, 5.0f, 0.01f);
//     EXPECT_NEAR(results[0].z, -1.0f, 0.01f);
//     std::cout << results[0] << std::endl;
// }
//
// TEST_F(GPULidarIntegration, Roatated90Raycast) {
//     TransformMatrix lidarPose = {
//         0.0f, 0.0f, 1.0f, -30.0f,
//         0.0f, 1.0f, 0.0f, 0.0f,
//         -1.0f, 0.0f, 0.0f, 0.0f
//     };
//     TransformMatrix sourcePoses = {
//         1.0f, 0.0f, 0.0f, 0.0f,
//         0.0f, 1.0f, 0.0f, 0.0f,
//         0.0f, 0.0f, 1.0f, 0.0f
//     };
//     LidarSource lidar_source("lidar", lidarPose, &sourcePoses, 1, ringIds_, ringIdsSize_, 100.0f);
//     lr_.render({lidar_source});
//     std::vector<Point3f> results(lr_.getNextDownloadPointCount());
//     lr_.downloadPoints(results.size(), results.data());
//     ASSERT_EQ(1, results.size());
//     EXPECT_NEAR(results[0].x, -1.0f, 0.01f);
//     EXPECT_NEAR(results[0].y, 0.0f, 0.01f);
//     EXPECT_NEAR(results[0].z, 0.0f, 0.01f);
//     std::cout << results[0] << std::endl;
// }
//
// TEST_F(GPULidarIntegration, MultipleRuns) {
//     TransformMatrix lidarPose = {
//         0.0f, 0.0f, 1.0f, -30.0f,
//         0.0f, 1.0f, 0.0f, 0.0f,
//         -1.0f, 0.0f, 0.0f, 0.0f
//     };
//     TransformMatrix sourcePoses = {
//         1.0f, 0.0f, 0.0f, 0.0f,
//         0.0f, 1.0f, 0.0f, 0.0f,
//         0.0f, 0.0f, 1.0f, 0.0f
//     };
//     size_t test_count = 500;
//     for (size_t i = 0; i < test_count; i++) {
//         if (i % 100 == 0) {
//             std::cout << "[" << i << "]:";
//         }
//         std::cout << ".";
//         if (i % 100 == 99) {
//             std::cout << std::endl;
//         }
//         LidarSource lidar_source("lidar", lidarPose, &sourcePoses, 1, ringIds_, ringIdsSize_, 100.0f);
//         lr_.render({lidar_source});
//         std::vector<Point3f> results(lr_.getNextDownloadPointCount());
//         lr_.downloadPoints(results.size(), results.data());
//         ASSERT_EQ(1, results.size());
//         ASSERT_NEAR(results[0].x, -1.0f, 0.01f);
//         ASSERT_NEAR(results[0].y, 0.0f, 0.01f);
//         ASSERT_NEAR(results[0].z, 0.0f, 0.01f);
//     }
// }
//
// TEST_F(GPULidarIntegration, MultipleRaysAndRuns) {
//     const float start_x = -0.99f;
//     const float end_x = 0.99f;
//     const int x_steps_count = 128;
//     const float start_y = -0.99f;
//     const float end_y = 0.99f;
//     const int y_steps_count = 360 * 5;
//
//     ASSERT_GT(x_steps_count, 1);
//     ASSERT_GT(y_steps_count, 1);
//
//     const float x_step = (end_x - start_x) / (x_steps_count - 1);
//     const float y_step = (end_y - start_y) / (y_steps_count - 1);
//     const float size = x_steps_count * y_steps_count;
//     std::cout << "Testing " << size << " points" << std::endl;
//
//     TransformMatrix lidarPose = {
//         1.0f, 0.0f, 0.0f, 0.0f,
//         0.0f, 1.0f, 0.0f, 0.0f,
//         0.0f, 0.0f, 1.0f, -30.0f
//     };
//
//     TransformMatrix sourcePosesTemplate = {
//         1.0f, 0.0f, 0.0f, 0.0f,
//         0.0f, 1.0f, 0.0f, 0.0f,
//         0.0f, 0.0f, 1.0f, 0.0f
//     };
//
//     std::vector<TransformMatrix> sourcePoses;
//     std::vector<Point3f> goldenResults (size);
//
//     for (int x_step_id = 0; x_step_id < x_steps_count; x_step_id++) {
//         for (int y_step_id = 0; y_step_id < y_steps_count; y_step_id++) {
//             const float x = start_x + x_step_id * x_step;
//             const float y = start_y + y_step_id * y_step;
//
//             sourcePoses.emplace_back(TransformMatrix{
//                 1.0f, 0.0f, 0.0f, x,
//                 0.0f, 1.0f, 0.0f, y,
//                 0.0f, 0.0f, 1.0f, 0.0f
//             });
//
//             int result_idx = x_step_id * y_steps_count + y_step_id;
//             goldenResults[result_idx].x = x;
//             goldenResults[result_idx].y = y;
//             goldenResults[result_idx].z = -1.0f;
//         }
//     }
//
// //    for (int i = 0; i < sourcePoses.size(); i++) {
// //        if (i % 12 == 0) {
// //            std::cout << "[" << i/12 << "]" << std::endl;
// //        }
// //        std::cout << sourcePoses[i] << " ";
// //        if (i % 4 == 3) {
// //            std::cout << std::endl;
// //        }
// //    }
// //
// //    for (int i = 0; i < goldenResults.size(); i++) {
// //        std::cout << "[" << i << "]:" << goldenResults[i] << std::endl;
// //    }
//     int sentRays = 0;
//     bool once = true;
//     for (size_t i = 0; i < 100; i++) {
//         sentRays += size;
//         if (i % 100 == 0) {
//             std::cout << "[" << i << ":" << sentRays << "]:";
//         }
//         std::cout << ".";
//         if (i % 100 == 99) {
//             std::cout << std::endl;
//         }
//         LidarSource lidar_source("lidar", lidarPose, sourcePoses.data(), sourcePoses.size(), ringIds_, ringIdsSize_, 100.0f);
//         lr_.render({lidar_source});
//         std::vector<Point3f> results(lr_.getNextDownloadPointCount());
//         lr_.downloadPoints(results.size(), results.data());
//         ASSERT_EQ(size, results.size());
//         for (int i = 0; i < size; i++) {
//             ASSERT_NEAR(results[i].x, goldenResults[i].x, 0.01f);
//             ASSERT_NEAR(results[i].y, goldenResults[i].y, 0.01f);
//             ASSERT_NEAR(results[i].z, goldenResults[i].z, 0.01f);
//         }
//     }
// }


