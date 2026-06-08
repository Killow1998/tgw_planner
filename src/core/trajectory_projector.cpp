#include "tgw_planner/core/trajectory_projector.hpp"

#include <cmath>

namespace tgw_planner::core
{

TrajectoryProjector::TrajectoryProjector(TrajectoryProjectorOptions options)
: options_(options)
{
  if (options_.resolution_m <= 0.0) {
    options_.resolution_m = 0.10;
  }
}

TrajectoryProjectionResult TrajectoryProjector::project(const N3NavResource & resource) const
{
  TrajectoryProjectionResult result;
  for (const auto & pose : resource.dense_trajectory) {
    Point3 point = pose.pose_world_lidar.translation;
    point.z += options_.support_z_offset_m;
    result.proven_seed_cells.insert(worldToGrid(point));
  }
  return result;
}

GridIndex TrajectoryProjector::worldToGrid(const Point3 & point) const
{
  return {
    static_cast<int>(std::floor(point.x / options_.resolution_m)),
    static_cast<int>(std::floor(point.y / options_.resolution_m)),
    static_cast<int>(std::floor(point.z / options_.resolution_m))};
}

}  // namespace tgw_planner::core
