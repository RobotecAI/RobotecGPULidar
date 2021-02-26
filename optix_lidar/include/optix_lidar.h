#pragma once

#include <memory>
#include <vector>
#include "lidar_source.h"
#include "raycast_result.h"
#include "model.h"

/// <summary>
/// Interface class for the Optix lidar module
/// Responsible for: raycasting for given meshes and lidar sources
/// Not responsible for: moving meshes or lidars around, managing raycast frequencies, computing vector directions of lidar rays.
/// </summary>
class OptiXLidar
{
public:
  OptiXLidar();
  ~OptiXLidar();

  /// <summary>
  /// Adds a mesh if not present, or updates the mesh if present
  /// </summary>
  void add_or_update_mesh(std::shared_ptr<TriangleMesh> mesh);

  /// <summary>
  /// Remove model by IDs
  /// </summary>
  void remove_mesh(const std::string & id);

  /// <summary>
  /// Acquire points from the most recent raycast by the source.
  /// </summary>
  void get_all_points();

  /// <summary>
  /// Perform raycasting for a single lidar source (a single sensor)
  /// </summary>
  void raycast(const LidarSource & source);

  const RaycastResults & last_results() const;

private:
  OptiXLidar(const OptiXLidar &) = delete;
  OptiXLidar& operator=(OptiXLidar const&) = delete;

  RaycastResults results_;

  class LidarImpl;
  std::unique_ptr<LidarImpl> lidar_impl_;
};
