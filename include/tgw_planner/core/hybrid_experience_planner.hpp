#pragma once

#include <cstddef>
#include <cstdint>
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
  double max_backbone_edge_xy_gap_m{1.00};
  double max_backbone_edge_dz_m{0.85};
  double max_backbone_edge_slope{4.0};
  double max_portal_xy_distance_m{1.20};
  double max_portal_height_error_m{0.45};
  double surface_target_speed_mps{0.6};
  double backbone_target_speed_mps{0.3};
  double portal_target_speed_mps{0.15};
};

enum class HybridNodeKind
{
  Surface,
  Backbone
};

enum class HybridEdgeKind
{
  Surface,
  Backbone,
  Portal
};

struct HybridNode
{
  HybridNodeKind kind{HybridNodeKind::Surface};
  SurfaceNodeId surface_id{0};
  BackboneNodeId backbone_id{0};
  Point3 point;
};

struct HybridEdge
{
  std::uint32_t to{0};
  HybridEdgeKind kind{HybridEdgeKind::Surface};
  double cost{0.0};
  double length_xy_m{0.0};
  double dz_m{0.0};
  ExperiencePortalId portal_id{0};
};

class HybridExperienceGraph
{
public:
  void build(
    const ExperienceSurfaceGraph & surface_graph,
    const ExperienceBackboneGraph & backbone_graph,
    const SurfacePlannerOptions & surface_options,
    const HybridExperiencePlannerOptions & hybrid_options);

  bool empty() const;
  bool isValid(std::uint32_t id) const;
  const std::vector<HybridNode> & nodes() const;
  const std::vector<std::vector<HybridEdge>> & adjacency() const;
  std::uint32_t backboneOffset() const;
  std::uint32_t surfaceEdgeCount() const;
  std::uint32_t backboneEdgeCount() const;
  std::uint32_t portalEdgeCount() const;

private:
  std::vector<HybridNode> nodes_;
  std::vector<std::vector<HybridEdge>> adjacency_;
  std::uint32_t backbone_offset_{0};
  std::uint32_t surface_edge_count_{0};
  std::uint32_t backbone_edge_count_{0};
  std::uint32_t portal_edge_count_{0};
};

struct HybridBackboneEdgeValidation
{
  bool allowed{false};
  const char * reason{""};
  double slope{0.0};
};

HybridBackboneEdgeValidation validateBackboneEdgeForHybrid(
  double length_xy_m,
  double dz_m,
  const HybridExperiencePlannerOptions & options);

HybridBackboneEdgeValidation validateBackboneEdgeForHybrid(
  const BackboneEdge & edge,
  const HybridExperiencePlannerOptions & options);

bool isBackboneEdgeAllowedForHybrid(
  const BackboneEdge & edge,
  const HybridExperiencePlannerOptions & options);

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
  SurfacePlanResult plan(
    const ExperienceSurfaceGraph & surface_graph,
    const ExperienceBackboneGraph & backbone_graph,
    const HybridExperienceGraph & hybrid_graph,
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
