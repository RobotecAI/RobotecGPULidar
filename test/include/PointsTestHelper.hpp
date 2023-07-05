#include <PointsGenerator.hpp>

template <typename PointType>
struct RGLTestUsePointsNodeHelper {
    rgl_node_t usePointsNode = nullptr;
    std::unique_ptr<RGLTestPointsGeneratorBase<PointType>> generator;
    int randomNonHitCount = 0;

    void prepareUsePointsNode(int pointsCount, HitPointDensity hitPointDensity = HitPointDensity::NONE) { }

    rgl_node_t simulateEmptyPointCloudOutputNode()
    {
        prepareUsePointsNode(1, HitPointDensity::ALL_NON_HIT);
        rgl_node_t compactNode = nullptr;
        EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compactNode));
        EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, compactNode));

        return compactNode;
    }

    std::vector<PointType> getResults(rgl_node_t dataNode)
    {
        EXPECT_RGL_SUCCESS(rgl_graph_run(dataNode));

        int32_t outCount = 0, outSizeOf = 0;

        std::vector<PointType> resultPoints;
        auto pointFields = generator->getPointFields();

        for (auto field : pointFields) {
            EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(dataNode, field, &outCount, &outSizeOf));
            EXPECT_EQ(outSizeOf, getFieldSize(field));

            switch (field) {
            case XYZ_F32: {
                std::vector<::Field<XYZ_F32>::type> outData(outCount);
                if (outCount != 0) {
                    EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(dataNode, field, outData.data()));
                    resultPoints.resize(outCount);
                    for (int i = 0; i < outCount; ++i) {
                        resultPoints[i].xyz = outData[i];
                    }
                }
                break;
            }
            case IS_HIT_I32: {
                std::vector<::Field<IS_HIT_I32>::type> outData(outCount);
                if (outCount != 0) {
                    EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(dataNode, field, outData.data()));
                    for (int i = 0; i < outCount; ++i) {
                        resultPoints[i].isHit = outData[i];
                    }
                }
                break;
            }
            case INTENSITY_F32: {
                std::vector<::Field<INTENSITY_F32>::type> outData(outCount);
                if (outCount != 0) {
                    EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(dataNode, field, outData.data()));
                    for (int i = 0; i < outCount; ++i) {
                        resultPoints[i].intensity = outData[i];
                    }
                }
                break;
            }
            }
        }

        return resultPoints;
    }

    void verifyResults(const std::vector<PointType>& resultPoints, const std::vector<PointType>& expectedPoints)
    {
        EXPECT_EQ(resultPoints.size(), expectedPoints.size());
        auto pointFields = generator->getPointFields();

        for (size_t i = 0; i < resultPoints.size(); ++i) {
            EXPECT_EQ(expectedPoints.at(i), resultPoints.at(i));
        }
    }

    void createUsePointsNode(const auto& generator)
    {
        ASSERT_RGL_SUCCESS(rgl_node_points_from_array(
            &usePointsNode,
            generator->getPointsData(),
            generator->getPointsSize(),
            generator->getPointFields().data(),
            generator->getPointFields().size()));

        ASSERT_THAT(usePointsNode, testing::NotNull());
    }
};

template <>
inline void RGLTestUsePointsNodeHelper<TestPoint>::prepareUsePointsNode(int pointsCount, HitPointDensity hitPointDensity)
{
    auto pointGenerator = std::make_unique<RGLTestPointsGenerator>();
    pointGenerator->generateTestPoints(pointsCount);
    createUsePointsNode(pointGenerator.get());
    generator = std::move(pointGenerator);
}

template <>
inline void RGLTestUsePointsNodeHelper<TestPointIsHit>::prepareUsePointsNode(int pointsCount, HitPointDensity hitPointDensity)
{
    auto pointGenerator = std::make_unique<RGLTestPointsIsHitGenerator>(hitPointDensity);
    pointGenerator->generateTestPoints(pointsCount);
    randomNonHitCount = pointGenerator->getRandomNonHitCount();
    createUsePointsNode(pointGenerator.get());
    generator = std::move(pointGenerator);
}