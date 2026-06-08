#include "tgw_planner/core/experience_surface_builder.hpp"

#include <chrono>
#include <cmath>
#include <utility>

namespace tgw_planner::core
{

ExperienceSurfaceBuilder::ExperienceSurfaceBuilder(ExperienceSurfaceBuilderOptions options)
: options_(std::move(options)),
  validator_(),
  projector_([&]() {
    TrajectoryProjectorOptions projector = options_.projector;
    projector.resolution_m = options_.resolution_m;
    return projector;
  }()),
  expander_(options_.expander)
{
  if (options_.resolution_m <= 0.0) {
    options_.resolution_m = 0.10;
  }
}

ExperienceBuildResult ExperienceSurfaceBuilder::build(const N3NavResource & resource) const
{
  const auto t0 = std::chrono::steady_clock::now();
  ExperienceBuildResult result;
  const N3MapReadResult validation = validator_.validate(resource);
  if (!validation.success) {
    result.error_code = validation.error_code;
    result.message = validation.message;
    return result;
  }

  std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> geometry;
  for (const auto & keyframe : resource.keyframes) {
    addKeyframeGeometry(keyframe, geometry);
  }
  result.geometry_cell_count = geometry.size();
  if (geometry.empty()) {
    result.error_code = "pbstream_no_keyframes";
    result.message = "keyframes are present but contain no usable geometry";
    return result;
  }

  const TrajectoryProjectionResult seeds = projector_.project(resource);
  if (seeds.proven_seed_cells.empty()) {
    result.error_code = "pbstream_missing_dense_trajectory";
    result.message = "dense trajectory did not produce any proven reachable seeds";
    return result;
  }

  ReachableExpansionResult expanded = expander_.expand(seeds.proven_seed_cells, geometry);
  if (expanded.traversable_cells.empty()) {
    result.error_code = "experience_surface_empty";
    result.message = "reachable surface builder produced no traversable cells";
    return result;
  }

  result.snapshot.map_frame = resource.map_frame.empty() ? "map" : resource.map_frame;
  result.snapshot.resolution_m = options_.resolution_m;
  result.snapshot.surface.surface_cells = std::move(expanded.surface_cells);
  result.snapshot.surface.traversable_cells = std::move(expanded.traversable_cells);
  result.snapshot.reachability = std::move(expanded.reachability);
  result.proven_seed_count = expanded.proven_seed_count;
  result.inferred_cell_count = expanded.inferred_cell_count;

  for (const auto & entry : result.snapshot.reachability) {
    if (entry.second == ReachabilityLabel::Forbidden) {
      result.snapshot.surface.forbidden_cells.insert(entry.first);
    }
  }

  rebuildBoundaryLayer(result.snapshot.surface);
  result.snapshot.clearance.compute(
    result.snapshot.surface.traversable_cells, result.snapshot.surface.boundary_cells,
    result.snapshot.resolution_m);
  result.snapshot.risk.compute(result.snapshot.surface, result.snapshot.clearance);
  result.success = true;
  const auto t1 = std::chrono::steady_clock::now();
  result.build_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  return result;
}

GridIndex ExperienceSurfaceBuilder::worldToGrid(const Point3 & point) const
{
  return {
    static_cast<int>(std::floor(point.x / options_.resolution_m)),
    static_cast<int>(std::floor(point.y / options_.resolution_m)),
    static_cast<int>(std::floor(point.z / options_.resolution_m))};
}

void ExperienceSurfaceBuilder::rebuildBoundaryLayer(SurfaceMap & surface) const
{
  surface.boundary_cells.clear();
  surface.dropoff_boundary_cells.clear();
  surface.wall_boundary_cells.clear();
  surface.forbidden_boundary_cells.clear();

  for (const GridIndex & cell : surface.traversable_cells) {
    bool boundary = false;
    for (int dx = -1; dx <= 1 && !boundary; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const GridIndex neighbor{cell.x + dx, cell.y + dy, cell.z};
        if (surface.traversable_cells.find(neighbor) == surface.traversable_cells.end()) {
          boundary = true;
          break;
        }
      }
    }
    if (boundary) {
      surface.boundary_cells.insert(cell);
      surface.wall_boundary_cells.insert(cell);
    }
  }
}

void ExperienceSurfaceBuilder::addKeyframeGeometry(
  const N3KeyframeLite & keyframe,
  std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> & geometry) const
{
  for (const PointXYZI & point_body : keyframe.cloud_body) {
    const Point3 world = transformPoint(
      keyframe.pose_optimized, {point_body.x, point_body.y, point_body.z});
    const GridIndex cell = worldToGrid(world);
    SurfaceCell & surface_cell = geometry[cell];
    surface_cell.cell = cell;
    surface_cell.support = {cell.x, cell.y, cell.z - 1};
    surface_cell.label = SurfaceLabel::GeometrySupport;
    surface_cell.reachability = ReachabilityLabel::Unknown;
    surface_cell.height_m = world.z;
    surface_cell.confidence = std::max(surface_cell.confidence, 0.25);
  }
}

}  // namespace tgw_planner::core
