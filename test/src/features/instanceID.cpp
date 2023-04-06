#include "lidars.hpp"
#include "scenes.hpp"
#include "utils.hpp"

class InstanceIDTest : public RGLTest
{

};

TEST_F(InstanceIDTest, BaseTest)
{
    setupThreeBoxScene(nullptr);

    rgl_node_t useRaysNode = nullptr, raytraceNode = nullptr, compactNode = nullptr, formatNode = nullptr;
    std::vector<rgl_mat3x4f> rays = makeLidar3dRays(360, 180, 0.36, 0.18);

    std::vector<rgl_field_t> formatFields = {
        XYZ_F32,
        ENTITY_IDX_I32,
        TIME_STAMP_F64
    };
    struct FormatStruct
    {
        Field<XYZ_F32>::type xyz;
        Field<ENTITY_IDX_I32>::type entityId;
        Field<TIME_STAMP_F64>::type timestamp;
    } formatStruct;


    EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, rays.data(), rays.size()));
    EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, 1000));
    EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compactNode));
    EXPECT_RGL_SUCCESS(rgl_node_points_format(&formatNode, formatFields.data(), formatFields.size()));

    EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, raytraceNode));
    EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, compactNode));
    EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compactNode, formatNode));

    EXPECT_RGL_SUCCESS(rgl_graph_run(raytraceNode));




   // int32_t outCount, outSizeOf;
  //  EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(compactNode, XYZ_F32, &outCount, &outSizeOf));



}