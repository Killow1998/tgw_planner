#include "tgw_planner/core/experience_backbone_graph.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>

namespace tgw_planner::core
{
namespace
{
constexpr std::uint32_t kInvalidSurfaceNodeId = std::numeric_limits<std::uint32_t>::max();

struct PortalCandidate
{
  SurfaceNodeId node{kInvalidSurfaceNodeId};
  int component{-1};
  double xy_distance_m{0.0};
  double height_error_m{0.0};
  double confidence{0.0};
  double score{std::numeric_limits<double>::infinity()};
};

double xyDistance(const Point3 & a, const Point3 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

Point3 surfaceNodePoint(const SurfaceNode & node, double resolution_m)
{
  return {
    (static_cast<double>(node.x) + 0.5) * resolution_m,
    (static_cast<double>(node.y) + 0.5) * resolution_m,
    node.z};
}
}  // namespace

void ExperienceBackboneGraph::build(
  const N3NavResource & resource,
  const TrajectoryProjectionResult & projection,
  const ExperienceSurfaceGraph & surface_graph,
  ExperienceBackboneOptions options)
{
  nodes_.clear();
  edges_.clear();
  cumulative_length_m_.clear();
  portals_.clear();
  portals_by_component_.clear();
  options_ = options;
  metrics_ = {};
  resolution_m_ = surface_graph.resolution();

  if (resource.dense_trajectory.empty()) {
    metrics_.backbone_z_min = 0.0;
    metrics_.backbone_z_max = 0.0;
    return;
  }

  std::unordered_map<std::uint64_t, ProjectedSupportSample> projected_by_seq;
  projected_by_seq.reserve(projection.accepted_projected_support_samples.size());
  for (const ProjectedSupportSample & sample : projection.accepted_projected_support_samples) {
    projected_by_seq[sample.seq] = sample;
  }

  const double body_to_support_z_m = inferBodyToSupportOffset(projection);
  metrics_.inferred_body_to_support_z_m = body_to_support_z_m;
  const double min_spacing = std::max(0.0, options_.min_node_spacing_m);

  bool have_last_kept = false;
  Point3 last_kept_position;
  for (std::size_t i = 0; i < resource.dense_trajectory.size(); ++i) {
    const N3TrajectoryPose & pose = resource.dense_trajectory[i];
    bool has_projected_support = false;
    const Point3 path_position = projectedOrInferredPathPosition(
      pose, projected_by_seq, body_to_support_z_m, &has_projected_support);
    const bool force_endpoint = i == 0U || i + 1U == resource.dense_trajectory.size();
    if (have_last_kept && !force_endpoint && xyDistance(path_position, last_kept_position) < min_spacing) {
      continue;
    }

    BackboneNode node;
    node.id = {static_cast<std::uint32_t>(nodes_.size())};
    node.seq = pose.seq;
    node.timestamp = pose.timestamp;
    node.trajectory_position = pose.pose_world_lidar.translation;
    node.path_position = path_position;
    node.has_projected_support = has_projected_support;
    node.low_confidence = !has_projected_support;
    addPortalForNode(surface_graph, node);

    metrics_.backbone_z_min = std::min(metrics_.backbone_z_min, node.path_position.z);
    metrics_.backbone_z_max = std::max(metrics_.backbone_z_max, node.path_position.z);
    nodes_.push_back(node);
    last_kept_position = path_position;
    have_last_kept = true;
  }

  cumulative_length_m_.assign(nodes_.size(), 0.0);
  for (std::size_t i = 1U; i < nodes_.size(); ++i) {
    const BackboneNode & from = nodes_[i - 1U];
    const BackboneNode & to = nodes_[i];
    const double length_xy = xyDistance(from.path_position, to.path_position);
    const double dz = to.path_position.z - from.path_position.z;
    const double slope = length_xy > 1.0e-9 ? std::abs(dz) / length_xy : 0.0;
    BackboneEdge edge;
    edge.from = from.id;
    edge.to = to.id;
    edge.length_xy_m = length_xy;
    edge.dz_m = dz;
    edge.slope = slope;
    edge.confidence = from.has_projected_support && to.has_projected_support ? 1.0 : 0.45;
    edges_.push_back(edge);
    cumulative_length_m_[i] = cumulative_length_m_[i - 1U] + length_xy;
    metrics_.max_backbone_edge_dz_m =
      std::max(metrics_.max_backbone_edge_dz_m, std::abs(dz));
    metrics_.max_backbone_edge_slope = std::max(metrics_.max_backbone_edge_slope, slope);
  }

  if (!std::isfinite(metrics_.backbone_z_min)) {
    metrics_.backbone_z_min = 0.0;
    metrics_.backbone_z_max = 0.0;
  }
  metrics_.backbone_nodes = nodes_.size();
  metrics_.backbone_edges = edges_.size();
  metrics_.portals = portals_.size();
}

bool ExperienceBackboneGraph::empty() const
{
  return nodes_.empty();
}

bool ExperienceBackboneGraph::isValid(BackboneNodeId id) const
{
  return id.id < nodes_.size();
}

double ExperienceBackboneGraph::resolution() const
{
  return resolution_m_;
}

const std::vector<BackboneNode> & ExperienceBackboneGraph::nodes() const
{
  return nodes_;
}

const std::vector<BackboneEdge> & ExperienceBackboneGraph::edges() const
{
  return edges_;
}

const std::vector<ExperiencePortal> & ExperienceBackboneGraph::portals() const
{
  return portals_;
}

const ExperiencePortal * ExperienceBackboneGraph::portal(ExperiencePortalId id) const
{
  if (id.id >= portals_.size()) {
    return nullptr;
  }
  return &portals_[id.id];
}

const BackboneNode * ExperienceBackboneGraph::node(BackboneNodeId id) const
{
  if (!isValid(id)) {
    return nullptr;
  }
  return &nodes_[id.id];
}

const std::vector<ExperiencePortalId> & ExperienceBackboneGraph::portalsForSurfaceComponent(
  int component_id) const
{
  const auto it = portals_by_component_.find(component_id);
  if (it == portals_by_component_.end()) {
    return empty_portals_;
  }
  return it->second;
}

std::vector<Point3> ExperienceBackboneGraph::pathPositionsBetween(
  BackboneNodeId from, BackboneNodeId to) const
{
  std::vector<Point3> path;
  if (!isValid(from) || !isValid(to)) {
    return path;
  }
  const std::uint32_t begin = std::min(from.id, to.id);
  const std::uint32_t end = std::max(from.id, to.id);
  path.reserve(static_cast<std::size_t>(end - begin + 1U));
  if (from.id <= to.id) {
    for (std::uint32_t i = begin; i <= end; ++i) {
      path.push_back(nodes_[i].path_position);
    }
  } else {
    for (std::uint32_t i = end + 1U; i > begin; --i) {
      path.push_back(nodes_[i - 1U].path_position);
    }
  }
  return path;
}

double ExperienceBackboneGraph::pathLengthBetween(BackboneNodeId from, BackboneNodeId to) const
{
  if (!isValid(from) || !isValid(to) || from == to) {
    return 0.0;
  }
  if (cumulative_length_m_.size() == nodes_.size()) {
    return std::abs(cumulative_length_m_[from.id] - cumulative_length_m_[to.id]);
  }
  const std::uint32_t begin = std::min(from.id, to.id);
  const std::uint32_t end = std::max(from.id, to.id);
  double fallback_length = 0.0;
  for (std::uint32_t i = begin + 1U; i <= end; ++i) {
    fallback_length += xyDistance(nodes_[i - 1U].path_position, nodes_[i].path_position);
  }
  return fallback_length;
}

const ExperienceBackboneMetrics & ExperienceBackboneGraph::metrics() const
{
  return metrics_;
}

void ExperienceBackboneGraph::addPortalForNode(
  const ExperienceSurfaceGraph & surface_graph, BackboneNode & node)
{
  if (surface_graph.empty()) {
    return;
  }

  const int center_x = static_cast<int>(std::floor(node.path_position.x / surface_graph.resolution()));
  const int center_y = static_cast<int>(std::floor(node.path_position.y / surface_graph.resolution()));
  const int radius_cells = std::max(
    0, static_cast<int>(std::ceil(options_.max_portal_xy_distance_m / surface_graph.resolution())));

  std::unordered_map<int, PortalCandidate> best_by_component;

  for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      const std::vector<SurfaceNodeId> * xy_nodes =
        surface_graph.nodesAtXY(center_x + dx, center_y + dy);
      if (xy_nodes == nullptr) {
        continue;
      }
      for (const SurfaceNodeId candidate_id : *xy_nodes) {
        const SurfaceNode * candidate = surface_graph.node(candidate_id);
        if (candidate == nullptr) {
          continue;
        }
        if (candidate->clearance_m < options_.min_portal_clearance_m) {
          continue;
        }
        const Point3 candidate_point = surfaceNodePoint(*candidate, surface_graph.resolution());
        const double xy_distance_m = xyDistance(node.path_position, candidate_point);
        if (xy_distance_m > options_.max_portal_xy_distance_m) {
          continue;
        }
        const double height_error_m = std::abs(node.path_position.z - candidate_point.z);
        if (height_error_m > options_.max_portal_height_error_m) {
          continue;
        }
        const int component = surface_graph.componentId(candidate_id);
        if (component < 0) {
          continue;
        }
        const double confidence = std::clamp(candidate->confidence, 0.0, 1.0);
        const double score = xy_distance_m + 0.25 * height_error_m + 0.05 * (1.0 - confidence);
        auto component_it = best_by_component.find(component);
        if (component_it == best_by_component.end() || score < component_it->second.score) {
          best_by_component[component] = {
            candidate_id,
            component,
            xy_distance_m,
            height_error_m,
            confidence,
            score};
        }
      }
    }
  }

  if (best_by_component.empty()) {
    return;
  }

  std::vector<PortalCandidate> candidates;
  candidates.reserve(best_by_component.size());
  for (const auto & entry : best_by_component) {
    candidates.push_back(entry.second);
  }
  std::sort(
    candidates.begin(), candidates.end(),
    [](const PortalCandidate & lhs, const PortalCandidate & rhs) {
      return lhs.score < rhs.score;
    });
  const std::size_t max_portals =
    std::max<std::size_t>(1U, options_.max_portals_per_node);
  if (candidates.size() > max_portals) {
    candidates.resize(max_portals);
  }

  const PortalCandidate & best = candidates.front();
  node.nearest_surface_node = best.node;
  node.nearest_surface_component_id = best.component;
  node.portal_distance_m = best.xy_distance_m;
  node.portal_height_error_m = best.height_error_m;
  node.has_surface_portal = true;

  for (const PortalCandidate & candidate : candidates) {
    ExperiencePortal portal;
    portal.id = {static_cast<std::uint32_t>(portals_.size())};
    portal.surface_component_id = candidate.component;
    portal.surface_node = candidate.node;
    portal.backbone_node = node.id;
    portal.distance_xy_m = candidate.xy_distance_m;
    portal.height_error_m = candidate.height_error_m;
    portal.confidence = candidate.confidence;
    portals_by_component_[candidate.component].push_back(portal.id);
    portals_.push_back(portal);
  }
}

double ExperienceBackboneGraph::inferBodyToSupportOffset(
  const TrajectoryProjectionResult & projection) const
{
  std::vector<double> offsets;
  offsets.reserve(projection.accepted_projected_support_samples.size());
  for (const ProjectedSupportSample & sample : projection.accepted_projected_support_samples) {
    offsets.push_back(sample.trajectory_position.z - sample.support_position.z);
  }
  if (offsets.empty()) {
    return 0.0;
  }
  std::sort(offsets.begin(), offsets.end());
  return offsets[offsets.size() / 2U];
}

Point3 ExperienceBackboneGraph::projectedOrInferredPathPosition(
  const N3TrajectoryPose & pose,
  const std::unordered_map<std::uint64_t, ProjectedSupportSample> & projected_by_seq,
  double body_to_support_z_m,
  bool * has_projected_support) const
{
  const auto projected_it = projected_by_seq.find(pose.seq);
  if (projected_it != projected_by_seq.end()) {
    if (has_projected_support != nullptr) {
      *has_projected_support = true;
    }
    return projected_it->second.support_position;
  }
  if (has_projected_support != nullptr) {
    *has_projected_support = false;
  }
  Point3 out = pose.pose_world_lidar.translation;
  out.z -= body_to_support_z_m;
  return out;
}

}  // namespace tgw_planner::core
