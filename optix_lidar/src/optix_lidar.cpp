#include "optix_lidar.h"

#include "LidarRenderer.h"
#include "lidar_source.h"
#include "model.h"
#include "raycast_result.h"
#include <memory>
#include <vector>

class OptiXLidar::LidarImpl {
public:
    LidarImpl()
        : renderer_(std::make_unique<LidarRenderer>())
    {
    }

    void add_meshes(std::vector<std::shared_ptr<TriangleMesh>> meshes)
    {
        renderer_->addMeshes(meshes);
    }

    void update_mesh_transform(const std::string& mesh_id, const TransformMatrix& transform)
    {
        renderer_->updateMeshTransform(mesh_id, transform);
    }

    void add_textures(std::vector<std::shared_ptr<Texture>> textures)
    {
        renderer_->addTextures(textures);
    }

    void remove_mesh(const std::string& mesh_id)
    {
        renderer_->removeMesh(mesh_id);
    }

    void remove_texture(const std::string& texture_id)
    {
        renderer_->removeTexture(texture_id);
    }

    void raycast(const LidarSource& source)
    {
        std::vector<LidarSource> lidars { source };
        // TODO - support more sources when needed in the higher interface

        // TODO - resize is not usually required if lidars don't change (?)
        renderer_->resize(lidars);
        renderer_->render(lidars);
    }

    void get_all_points(RaycastResults& results)
    {
        renderer_->downloadPoints(results);
    }

private:
    std::unique_ptr<LidarRenderer> renderer_;
};

OptiXLidar::OptiXLidar()
{
    lidar_impl_ = std::make_unique<LidarImpl>();
}

// Necessary to declare in cpp for pimpl to work with unique_ptr
OptiXLidar::~OptiXLidar() = default;

void OptiXLidar::add_mesh(std::shared_ptr<TriangleMesh> mesh)
{
	std::vector<std::shared_ptr<TriangleMesh>> meshes { mesh };
    lidar_impl_->add_meshes(meshes);
}

void OptiXLidar::update_mesh_transform(const std::string& mesh_id, const TransformMatrix& transform)
{
    lidar_impl_->update_mesh_transform(mesh_id, transform);
}

void OptiXLidar::remove_mesh(const std::string& id)
{
    lidar_impl_->remove_mesh(id);
}

void OptiXLidar::get_all_points()
{
    return lidar_impl_->get_all_points(results_);
}

void OptiXLidar::raycast(const LidarSource& source)
{
    lidar_impl_->raycast(source);
}

const RaycastResults& OptiXLidar::last_results() const
{
    return results_;
}
