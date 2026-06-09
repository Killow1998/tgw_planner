#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

#include "tgw_planner/core/experience_surface_graph.hpp"
#include "tgw_planner/core/n3map_reader.hpp"
#include "tgw_planner/core/trajectory_projector.hpp"

namespace tgw_planner::core
{

struct BackboneNodeId
{
  std::uint32_t id{0};

  bool operator==(const BackboneNodeId & other) const
  {
    return id == other.id;
  }

  bool operator!=(const BackboneNodeId & other) const
  {
    return !(*this == other);
  }
};

struct ExperiencePortalId
{
  std::uint32_t id{0};

  bool operator==(const ExperiencePortalId & other) const
  {
    return id == other.id;
  }
};

struct BackboneNode
{
  BackboneNodeId id;
  std::uint64_t seq{0};
  double timestamp{0.0};
  Point3 trajectory_position;
  Point3 path_position;
  int nearest_surface_component_id{-1};
  SurfaceNodeId nearest_surface_node{std::numeric_limits<std::uint32_t>::max()};
  double portal_distance_m{0.0};
  double portal_height_error_m{0.0};
  bool has_projected_support{false};
  bool has_surface_portal{false};
  bool low_confidence{false};
};

struct BackboneEdge
{
  BackboneNodeId from;
  BackboneNodeId to;
  double length_xy_m{0.0};
  double dz_m{0.0};
  double slope{0.0};
  double confidence{1.0};
  bool walked_edge{true};
};

struct ExperiencePortal
{
  ExperiencePortalId id;
  int surface_component_id{-1};
  SurfaceNodeId surface_node{std::numeric_limits<std::uint32_t>::max()};
  BackboneNodeId backbone_node{std::numeric_limits<std::uint32_t>::max()};
  double distance_xy_m{0.0};
  double height_error_m{0.0};
  double confidence{1.0};
};

struct ExperienceBackboneOptions
{
  double min_node_spacing_m{0.20};
  double max_portal_xy_distance_m{1.20};
  double max_portal_height_error_m{0.45};
  double min_portal_clearance_m{0.0};
};

struct ExperienceBackboneMetrics
{
  std::size_t backbone_nodes{0};
  std::size_t backbone_edges{0};
  std::size_t portals{0};
  double backbone_z_min{std::numeric_limits<double>::infinity()};
  double backbone_z_max{-std::numeric_limits<double>::infinity()};
  double max_backbone_edge_dz_m{0.0};
  double max_backbone_edge_slope{0.0};
  double inferred_body_to_support_z_m{0.0};
};

class ExperienceBackboneGraph
{
public:
  void build(
    const N3NavResource & resource,
    const TrajectoryProjectionResult & projection,
    const ExperienceSurfaceGraph & surface_graph,
    ExperienceBackboneOptions options = {});

  bool empty() const;
  bool isValid(BackboneNodeId id) const;
  double resolution() const;
  const std::vector<BackboneNode> & nodes() const;
  const std::vector<BackboneEdge> & edges() const;
  const std::vector<ExperiencePortal> & portals() const;
  const ExperiencePortal * portal(ExperiencePortalId id) const;
  const BackboneNode * node(BackboneNodeId id) const;
  const std::vector<ExperiencePortalId> & portalsForSurfaceComponent(int component_id) const;
  std::vector<Point3> pathPositionsBetween(BackboneNodeId from, BackboneNodeId to) const;
  double pathLengthBetween(BackboneNodeId from, BackboneNodeId to) const;
  const ExperienceBackboneMetrics & metrics() const;

private:
  void addPortalForNode(const ExperienceSurfaceGraph & surface_graph, BackboneNode & node);
  double inferBodyToSupportOffset(const TrajectoryProjectionResult & projection) const;
  Point3 projectedOrInferredPathPosition(
    const N3TrajectoryPose & pose,
    const std::unordered_map<std::uint64_t, ProjectedSupportSample> & projected_by_seq,
    double body_to_support_z_m,
    bool * has_projected_support) const;

  std::vector<BackboneNode> nodes_;
  std::vector<BackboneEdge> edges_;
  std::vector<ExperiencePortal> portals_;
  std::unordered_map<int, std::vector<ExperiencePortalId>> portals_by_component_;
  std::vector<ExperiencePortalId> empty_portals_;
  ExperienceBackboneOptions options_;
  ExperienceBackboneMetrics metrics_;
  double resolution_m_{0.10};
};

}  // namespace tgw_planner::core
