#pragma once

#include <cstddef>
#include <vector>

#include "tgw_planner/core/experience_backbone_graph.hpp"
#include "tgw_planner/core/surface_astar_planner.hpp"

namespace tgw_planner::core
{

struct HybridExperiencePlannerOptions
{
  std::size_t max_portal_candidates_per_side{64};
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
  std::vector<ExperiencePortalId> sortedPortalCandidates(
    const ExperienceSurfaceGraph & surface_graph,
    const ExperienceBackboneGraph & backbone_graph,
    int surface_component_id,
    SurfaceNodeId query_node) const;
  Point3 surfacePoint(const ExperienceSurfaceGraph & graph, SurfaceNodeId node_id) const;
  void appendPath(std::vector<Point3> & out, const std::vector<Point3> & in) const;
  double pathLength(const std::vector<Point3> & path) const;
  void fillPathMetrics(SurfacePlanResult & result) const;

  SurfacePlannerOptions surface_options_;
  HybridExperiencePlannerOptions hybrid_options_;
};

}  // namespace tgw_planner::core
