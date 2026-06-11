#include "tgw_planner/core/experience_surface_builder.hpp"

#include <chrono>
#include <cmath>
#include <algorithm>
#include <utility>

namespace tgw_planner::core
{

ExperienceSurfaceBuilder::ExperienceSurfaceBuilder(ExperienceSurfaceBuilderOptions options)
: options_(std::move(options)),
  validator_()
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
  ExperienceGeometryIndex geometry;
  ExperienceGeometryIndexOptions geometry_options;
  geometry_options.raw_resolution_m = options_.projector.raw_resolution_m;
  geometry_options.nav_resolution_m = options_.resolution_m;
  geometry_options.body_clearance_height_m = options_.body_clearance_height_m;
  geometry_options.trajectory_roi_distance_m = options_.geometry_roi_distance_to_trajectory_m;
  geometry_options.max_debug_world_points = 0U;
  const ExperienceGeometryIndexBuildResult geometry_result =
    geometry.build(resource, geometry_options);
  if (!geometry_result.success) {
    ExperienceBuildResult result;
    result.error_code = geometry_result.error_code;
    result.message = geometry_result.message;
    result.raw_geometry_cell_count = geometry_result.raw_geometry_cell_count;
    result.geometry_cell_count = geometry_result.support_candidate_count;
    result.support_candidate_count = geometry_result.support_candidate_count;
    return result;
  }

  TrajectoryProjectorOptions projector_options = options_.projector;
  projector_options.resolution_m = options_.resolution_m;
  const TrajectoryProjectionResult projection =
    TrajectoryProjector(projector_options).project(resource, geometry);
  return build(resource, geometry, projection);
}

ExperienceBuildResult ExperienceSurfaceBuilder::build(
  const N3NavResource & resource,
  const ExperienceGeometryIndex & geometry,
  const TrajectoryProjectionResult & seeds) const
{
  const auto t0 = std::chrono::steady_clock::now();
  ExperienceBuildResult result;
  const N3MapReadResult validation = validator_.validate(resource);
  if (!validation.success) {
    result.error_code = validation.error_code;
    result.message = validation.message;
    return result;
  }

  result.raw_geometry_cell_count = geometry.rawGeometry().size();
  result.geometry_cell_count = geometry.supportCandidates().size();
  result.support_candidate_count = geometry.supportCandidates().size();
  if (geometry.supportCandidates().empty()) {
    result.error_code = "pbstream_no_support_candidates";
    result.message = "keyframes are present but contain no usable geometry";
    return result;
  }

  if (seeds.proven_seed_cells.empty()) {
    result.error_code = "support_projection_failed";
    result.message = "dense trajectory did not project to any support cells";
    return result;
  }

  const auto t_expansion = std::chrono::steady_clock::now();
  ReachableExpansionResult expanded =
    ReachableExpander(options_.expander).expand(
      seeds.observed_seed_cells, seeds.bridge_seed_cells, seeds.bridge_cell_metadata,
      geometry.supportCandidates());
  result.expansion_time_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t_expansion).count();
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
  result.snapshot.bridge_segments = seeds.bridge_segments;
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
  result.expansion_anchored_component_time_ms = expanded.anchored_component_time_ms;
  result.expansion_anchor_envelope_time_ms = expanded.anchor_envelope_time_ms;
  result.expansion_seed_initialization_time_ms = expanded.seed_initialization_time_ms;
  result.expansion_frontier_time_ms = expanded.expansion_frontier_time_ms;
  result.expansion_wave_time_ms = expanded.expansion_wave_time_ms;
  result.expansion_hole_fill_time_ms = expanded.hole_fill_time_ms;
  result.expansion_layer_assignment_time_ms = expanded.layer_assignment_time_ms;
  result.expansion_bridge_seed_time_ms = expanded.bridge_seed_time_ms;
  result.expansion_compact_time_ms = expanded.compact_time_ms;

  for (const auto & entry : result.snapshot.reachability) {
    if (entry.second == ReachabilityLabel::Forbidden) {
      result.snapshot.surface.forbidden_cells.insert(entry.first);
    }
  }

  const auto t_boundary = std::chrono::steady_clock::now();
  rebuildBoundaryLayer(result.snapshot.surface);
  result.boundary_time_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t_boundary).count();
  const auto t_clearance = std::chrono::steady_clock::now();
  result.snapshot.clearance.compute(
    result.snapshot.surface.traversable_cells, result.snapshot.surface.boundary_cells,
    result.snapshot.resolution_m);
  result.clearance_time_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t_clearance).count();
  const auto t_risk = std::chrono::steady_clock::now();
  result.snapshot.risk.compute(result.snapshot.surface, result.snapshot.clearance);
  result.risk_time_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t_risk).count();
  result.success = true;
  const auto t1 = std::chrono::steady_clock::now();
  result.build_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  return result;
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

}  // namespace tgw_planner::core
