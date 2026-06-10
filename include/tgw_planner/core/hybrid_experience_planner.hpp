#pragma once

#include <cstddef>
#include <vector>

#include "tgw_planner/core/experience_backbone_graph.hpp"
#include "tgw_planner/core/surface_astar_planner.hpp"

namespace tgw_planner::core
{

struct HybridExperiencePlannerOptions
{
  double backbone_cost_scale{1.2};
  double portal_switch_cost{0.5};
  double portal_height_error_weight{0.25};
  double backbone_low_confidence_penalty{0.5};
  double max_backbone_edge_xy_gap_m{5.00};
  double max_backbone_edge_dz_m{0.85};
  double max_backbone_edge_slope{4.0};
  double max_portal_xy_distance_m{1.20};
  double max_portal_height_error_m{0.45};
  double surface_target_speed_mps{0.6};
  double backbone_target_speed_mps{0.3};
  double portal_target_speed_mps{0.15};
};

class HybridExperiencePlanner
{
public:
  explicit HybridExperiencePlanner(
    SurfacePlannerOptions surface_options = {},
    HybridExperiencePlannerOptions hybrid_options = {});

  SurfacePlanResult plan(
    const ExperienceSurfaceGraph & surface_graph,
    const ExperienceBackboneGraph & backbone_graph,
    SurfaceNodeId start,
    SurfaceNodeId goal) const;

private:
  Point3 surfacePoint(const ExperienceSurfaceGraph & graph, SurfaceNodeId node_id) const;
  void appendPoint(std::vector<Point3> & out, const Point3 & point) const;
  void appendPathPoint(SurfacePlanResult & result, const Point3 & point, PathPointKind kind) const;
  void appendGlobalPathPoint(
    SurfacePlanResult & result,
    const Point3 & point,
    PathPointKind kind,
    int surface_component_id,
    double confidence) const;
  void fillGlobalPathYaws(SurfacePlanResult & result) const;
  double pathLength(const std::vector<Point3> & path) const;
  void fillPathMetrics(SurfacePlanResult & result) const;

  SurfacePlannerOptions surface_options_;
  HybridExperiencePlannerOptions hybrid_options_;
};

}  // namespace tgw_planner::core
