#pragma once

#include <string>

#include "tgw_planner/core/experience_snapshot.hpp"
#include "tgw_planner/core/experience_geometry_index.hpp"
#include "tgw_planner/core/n3map_reader.hpp"
#include "tgw_planner/core/reachable_expander.hpp"
#include "tgw_planner/core/trajectory_projector.hpp"

namespace tgw_planner::core
{

struct ExperienceSurfaceBuilderOptions
{
  double resolution_m{0.10};
  double body_clearance_height_m{0.65};
  double geometry_roi_distance_to_trajectory_m{1.8};
  TrajectoryProjectorOptions projector;
  ReachableExpanderOptions expander;
};

struct ExperienceBuildResult
{
  bool success{false};
  std::string error_code;
  std::string message;
  ExperienceSnapshot snapshot;
  std::size_t raw_geometry_cell_count{0};
  std::size_t geometry_cell_count{0};
  std::size_t support_candidate_count{0};
  std::size_t proven_seed_count{0};
  std::size_t inferred_cell_count{0};
  std::size_t rejected_expansion_count{0};
  std::size_t body_obstructed_rejected_count{0};
  std::size_t anchor_envelope_rejected_count{0};
  std::size_t hole_filled_count{0};
  std::size_t bridge_seed_count{0};
  std::size_t bridge_used_as_expansion_anchor{0};
  std::size_t hole_fill_from_bridge_rejected{0};
  std::size_t support_component_count{0};
  std::size_t anchored_support_component_count{0};
  std::size_t rejected_unanchored_component_cells{0};
  double build_time_ms{0.0};
  double expansion_time_ms{0.0};
  double boundary_time_ms{0.0};
  double clearance_time_ms{0.0};
  double risk_time_ms{0.0};
  double expansion_anchored_component_time_ms{0.0};
  double expansion_anchor_envelope_time_ms{0.0};
  double expansion_seed_initialization_time_ms{0.0};
  double expansion_frontier_time_ms{0.0};
  double expansion_wave_time_ms{0.0};
  double expansion_hole_fill_time_ms{0.0};
  double expansion_layer_assignment_time_ms{0.0};
  double expansion_bridge_seed_time_ms{0.0};
  double expansion_compact_time_ms{0.0};
};

class ExperienceSurfaceBuilder
{
public:
  explicit ExperienceSurfaceBuilder(ExperienceSurfaceBuilderOptions options = {});

  // Mainline skeleton pipeline:
  // 1. trajectory resampling/projected support seeds
  // 2. proven reachable seed generation
  // 3. conservative reachable expansion from keyframe geometry
  // 4. boundary / clearance / risk rebuild
  //
  // TGW prefers explicit failure over uncertain recovery.
  // No silent fallback to global_map.pcd, realtime raycast reconstruction,
  // terrain semantics, or StairFlight-style rescue logic is allowed here.
  ExperienceBuildResult build(const N3NavResource & resource) const;
  ExperienceBuildResult build(
    const N3NavResource & resource,
    const ExperienceGeometryIndex & geometry,
    const TrajectoryProjectionResult & projection) const;

private:
  void rebuildBoundaryLayer(SurfaceMap & surface) const;

  ExperienceSurfaceBuilderOptions options_;
  N3MapReader validator_;
};

}  // namespace tgw_planner::core
