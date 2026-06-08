#pragma once

#include <string>

#include "tgw_planner/core/experience_snapshot.hpp"
#include "tgw_planner/core/n3map_reader.hpp"
#include "tgw_planner/core/reachable_expander.hpp"
#include "tgw_planner/core/trajectory_projector.hpp"

namespace tgw_planner::core
{

struct ExperienceSurfaceBuilderOptions
{
  double resolution_m{0.10};
  TrajectoryProjectorOptions projector;
  ReachableExpanderOptions expander;
};

struct ExperienceBuildResult
{
  bool success{false};
  std::string error_code;
  std::string message;
  ExperienceSnapshot snapshot;
  std::size_t geometry_cell_count{0};
  std::size_t proven_seed_count{0};
  std::size_t inferred_cell_count{0};
  double build_time_ms{0.0};
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

private:
  GridIndex worldToGrid(const Point3 & point) const;
  void rebuildBoundaryLayer(SurfaceMap & surface) const;
  void addKeyframeGeometry(
    const N3KeyframeLite & keyframe,
    std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> & geometry) const;

  ExperienceSurfaceBuilderOptions options_;
  N3MapReader validator_;
  TrajectoryProjector projector_;
  ReachableExpander expander_;
};

}  // namespace tgw_planner::core
