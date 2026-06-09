#include "tgw_planner/core/experience_surface_builder.hpp"

#include <chrono>
#include <cmath>
#include <algorithm>
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
  }())
{
  if (options_.resolution_m <= 0.0) {
    options_.resolution_m = 0.10;
  }
  if (options_.body_clearance_height_m < 0.0) {
    options_.body_clearance_height_m = 0.0;
  }
  options_.expander.resolution_m = options_.resolution_m;
  options_.expander.body_clearance_cells = std::max(
    options_.expander.body_clearance_cells,
    static_cast<int>(std::ceil(options_.body_clearance_height_m / options_.resolution_m)));
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

  std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> raw_geometry;
  for (const auto & keyframe : resource.keyframes) {
    addKeyframeGeometry(keyframe, raw_geometry);
  }
  markBodyObstructions(raw_geometry);
  result.raw_geometry_cell_count = raw_geometry.size();
  std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> geometry =
    buildSupportCandidates(raw_geometry);
  markBodyObstructions(geometry);
  result.geometry_cell_count = geometry.size();
  result.support_candidate_count = geometry.size();
  if (geometry.empty()) {
    result.error_code = "pbstream_no_keyframes";
    result.message = "keyframes are present but contain no usable geometry";
    return result;
  }

  const TrajectoryProjectionResult seeds = projector_.project(resource);
  if (seeds.proven_seed_cells.empty()) {
    result.error_code = "support_projection_failed";
    result.message = "dense trajectory did not project to any support cells";
    return result;
  }

  ReachableExpansionResult expanded =
    ReachableExpander(options_.expander).expand(
      seeds.observed_seed_cells, seeds.bridge_seed_cells, seeds.bridge_cell_metadata, geometry);
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
  result.rejected_expansion_count = expanded.rejected_expansion_count;
  result.body_obstructed_rejected_count = expanded.body_obstructed_rejected_count;
  result.anchor_envelope_rejected_count = expanded.anchor_envelope_rejected_count;
  result.hole_filled_count = expanded.hole_filled_count;
  result.bridge_seed_count = expanded.bridge_seed_count;
  result.bridge_used_as_expansion_anchor = expanded.bridge_used_as_expansion_anchor;
  result.hole_fill_from_bridge_rejected = expanded.hole_fill_from_bridge_rejected;
  result.support_component_count = expanded.support_component_count;
  result.anchored_support_component_count = expanded.anchored_support_component_count;
  result.rejected_unanchored_component_cells = expanded.rejected_unanchored_component_cells;

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

std::unordered_map<GridIndex, SurfaceCell, GridIndexHash>
ExperienceSurfaceBuilder::buildSupportCandidates(
  const std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> & raw_geometry) const
{
  std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> candidates;
  constexpr int kNeighborRadius = 1;
  constexpr int kMinLocalSupportCells = 3;
  const double max_local_height_spread = std::max(0.12, options_.resolution_m * 1.5);

  for (const auto & entry : raw_geometry) {
    const GridIndex & cell = entry.first;
    const SurfaceCell & raw_cell = entry.second;
    if (raw_cell.body_obstructed) {
      continue;
    }

    int support_neighbors = 0;
    double min_height = raw_cell.height_m;
    double max_height = raw_cell.height_m;
    for (int dx = -kNeighborRadius; dx <= kNeighborRadius; ++dx) {
      for (int dy = -kNeighborRadius; dy <= kNeighborRadius; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          const auto neighbor_it = raw_geometry.find({cell.x + dx, cell.y + dy, cell.z + dz});
          if (neighbor_it == raw_geometry.end()) {
            continue;
          }
          if (std::abs(neighbor_it->second.height_m - raw_cell.height_m) >
            max_local_height_spread)
          {
            continue;
          }
          ++support_neighbors;
          min_height = std::min(min_height, neighbor_it->second.height_m);
          max_height = std::max(max_height, neighbor_it->second.height_m);
        }
      }
    }

    if (support_neighbors < kMinLocalSupportCells) {
      continue;
    }
    if ((max_height - min_height) > max_local_height_spread) {
      continue;
    }

    SurfaceCell candidate = raw_cell;
    candidate.label = SurfaceLabel::GeometrySupport;
    candidate.reachability = ReachabilityLabel::Unknown;
    candidate.confidence = std::max(candidate.confidence, 0.35);
    candidates[cell] = candidate;
  }

  return candidates;
}

void ExperienceSurfaceBuilder::markBodyObstructions(
  std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> & geometry) const
{
  const int body_clearance_cells = std::max(
    0, static_cast<int>(std::ceil(options_.body_clearance_height_m / options_.resolution_m)));
  if (body_clearance_cells == 0) {
    return;
  }

  for (auto & entry : geometry) {
    const GridIndex cell = entry.first;
    for (int dz = 1; dz <= body_clearance_cells; ++dz) {
      if (geometry.find({cell.x, cell.y, cell.z + dz}) != geometry.end()) {
        entry.second.body_obstructed = true;
        break;
      }
    }
  }
}

}  // namespace tgw_planner::core
