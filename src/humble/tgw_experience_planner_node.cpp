#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <glog/logging.h>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "tgw_planner/core/experience_planner_defaults.hpp"
#include "tgw_planner/core/experience_surface_builder.hpp"
#include "tgw_planner/core/experience_surface_graph.hpp"
#include "tgw_planner/core/grid_index.hpp"
#include "tgw_planner/core/experience_backbone_graph.hpp"
#include "tgw_planner/core/hybrid_experience_planner.hpp"
#include "tgw_planner/core/local_path_smoother.hpp"
#include "tgw_planner/core/n3map_reader.hpp"
#include "tgw_planner/core/regulated_pure_pursuit.hpp"
#include "tgw_planner/core/rolling_local_map.hpp"
#include "tgw_planner/core/route_progress_tracker.hpp"
#include "tgw_planner/core/surface_astar_planner.hpp"
#include "tgw_planner/core/trajectory_projector.hpp"
#include "tgw_planner/msg/planner_stats.hpp"
#include "tgw_planner/srv/plan_path.hpp"

namespace
{
namespace fs = std::filesystem;

using tgw_planner::core::N3MapReadResult;
using tgw_planner::core::ExperienceBackboneGraph;
using tgw_planner::core::ExperienceBackboneOptions;
using tgw_planner::core::N3MapReader;
using tgw_planner::core::N3NavResource;
using tgw_planner::core::Point3;
using tgw_planner::core::ProjectedSupportSample;
using tgw_planner::core::RejectedProjectionSample;
using tgw_planner::core::ExperienceBuildResult;
using tgw_planner::core::ExperienceSurfaceGraph;
using tgw_planner::core::ExperienceSurfaceBuilder;
using tgw_planner::core::ExperienceSurfaceBuilderOptions;
using tgw_planner::core::GridIndex;
using tgw_planner::core::SurfaceGraphBuildOptions;
using tgw_planner::core::SurfaceNode;
using tgw_planner::core::SurfaceNodeId;
using tgw_planner::core::LocalPathResult;
using tgw_planner::core::LocalPathSmoother;
using tgw_planner::core::LocalPathSmootherOptions;
using tgw_planner::core::SurfaceAstarPlanner;
using tgw_planner::core::SurfacePlanResult;
using tgw_planner::core::SurfacePlannerOptions;
using tgw_planner::core::SurfaceTransitionValidator;
using tgw_planner::core::HybridExperiencePlanner;
using tgw_planner::core::HybridExperiencePlannerOptions;
using tgw_planner::core::RegulatedPurePursuitCommand;
using tgw_planner::core::RegulatedPurePursuitController;
using tgw_planner::core::RegulatedPurePursuitOptions;
using tgw_planner::core::RollingLocalMap;
using tgw_planner::core::RollingLocalMapOptions;
using tgw_planner::core::RoutePose2D;
using tgw_planner::core::RouteProgressState;
using tgw_planner::core::RouteProgressTracker;
using tgw_planner::core::RouteProgressTrackerOptions;
using tgw_planner::core::TrajectoryProjectionResult;
using tgw_planner::core::TrajectoryProjector;
using tgw_planner::core::TrajectoryProjectorOptions;
using tgw_planner::core::defaultExperienceBackboneOptions;
using tgw_planner::core::defaultExperienceSurfaceBuilderOptions;
using tgw_planner::core::defaultHybridExperiencePlannerOptions;
using tgw_planner::core::defaultSurfaceGraphBuildOptions;
using tgw_planner::core::defaultSurfacePlannerOptions;
using tgw_planner::core::defaultTrajectoryProjectorOptions;

struct IntensityPoint
{
  Point3 point;
  double intensity{0.0};
};

void cleanupOldLogFiles(const std::string & log_dir, int retention_days)
{
  if (retention_days <= 0 || log_dir.empty()) {
    return;
  }
  std::error_code error;
  if (!fs::exists(log_dir, error)) {
    return;
  }
  const auto cutoff = fs::file_time_type::clock::now() -
    std::chrono::hours(24 * retention_days);
  for (const fs::directory_entry & entry : fs::directory_iterator(log_dir, error)) {
    if (error) {
      return;
    }
    const fs::file_status status = entry.symlink_status(error);
    if (error || !(fs::is_regular_file(status) || fs::is_symlink(status))) {
      error.clear();
      continue;
    }
    const auto modified = entry.last_write_time(error);
    if (error) {
      error.clear();
      continue;
    }
    if (modified < cutoff) {
      fs::remove(entry.path(), error);
      error.clear();
    }
  }
}

std::string resolvePlannerLogDir(const std::string & configured_log_dir)
{
  if (configured_log_dir.empty()) {
    return configured_log_dir;
  }
  const fs::path configured(configured_log_dir);
  if (configured.is_absolute()) {
    return configured.string();
  }

  std::error_code error;
  const fs::path cwd = fs::current_path(error);
  if (error) {
    return configured.string();
  }
  if (cwd.filename() == "tgw_planner") {
    return (cwd / configured).string();
  }
  const fs::path source_package = cwd / "src" / "tgw_planner";
  if (fs::is_directory(source_package, error)) {
    return (source_package / configured).string();
  }
  return (cwd / configured).string();
}

void configurePlannerFileLogging(const std::string & log_dir, int retention_days)
{
  const std::string resolved_log_dir = resolvePlannerLogDir(log_dir);
  if (resolved_log_dir.empty()) {
    return;
  }
  std::error_code error;
  fs::create_directories(resolved_log_dir, error);
  if (error) {
    return;
  }
  cleanupOldLogFiles(resolved_log_dir, retention_days);
  FLAGS_log_dir = resolved_log_dir;
  FLAGS_logtostderr = false;
  FLAGS_alsologtostderr = false;
  FLAGS_colorlogtostderr = false;
  FLAGS_stderrthreshold = google::GLOG_FATAL;
  FLAGS_log_link = "";
}

std::string jsonEscape(const std::string & text)
{
  std::ostringstream out;
  for (const char ch : text) {
    switch (ch) {
      case '\\':
        out << "\\\\";
        break;
      case '"':
        out << "\\\"";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << ch;
        break;
    }
  }
  return out.str();
}

std::string toStatsJson(
  const N3MapReadResult & result,
  const std::string & pbstream_path,
  const TrajectoryProjectionResult * projection,
  const ExperienceBuildResult * build,
  const ExperienceSurfaceGraph * surface_graph,
  const ExperienceBackboneGraph * backbone_graph,
  double planner_multifloor_z_range_m)
{
  const N3NavResource & resource = result.resource;
  std::ostringstream json;
  json << "{";
  json << "\"success\":" << (result.success ? "true" : "false");
  json << ",\"error_code\":\"" << jsonEscape(result.error_code) << "\"";
  json << ",\"message\":\"" << jsonEscape(result.message) << "\"";
  json << ",\"pbstream_path\":\"" << jsonEscape(pbstream_path) << "\"";
  json << ",\"version\":\"" << jsonEscape(resource.version) << "\"";
  json << ",\"map_frame\":\"" << jsonEscape(resource.map_frame) << "\"";
  json << ",\"body_frame\":\"" << jsonEscape(resource.body_frame) << "\"";
  json << ",\"keyframes\":" << resource.keyframes.size();
  json << ",\"dense_trajectory_count\":" << resource.dense_trajectory.size();
  json << ",\"dense_trajectory_source\":\"" <<
    jsonEscape(resource.dense_trajectory_source) << "\"";
  json << ",\"dense_trajectory_degraded\":" <<
    (resource.dense_trajectory_degraded ? "true" : "false");
  json << ",\"has_native_dense_trajectory\":" <<
    (resource.has_native_dense_trajectory ? "true" : "false");
  json << ",\"dense_trajectory_from_keyframe_fallback\":" <<
    (resource.dense_trajectory_from_keyframe_fallback ? "true" : "false");
  json << ",\"nav_cloud_filter_applied\":" <<
    (resource.nav_cloud_filter_applied ? "true" : "false");
  json << ",\"nav_cloud_filter_policy\":\"" <<
    jsonEscape(resource.nav_cloud_filter_policy) << "\"";
  json << ",\"nav_filter_raw_points\":" << resource.nav_filter_raw_points;
  json << ",\"nav_filter_kept_points\":" << resource.nav_filter_kept_points;
  json << ",\"nav_filter_removed_points\":" << resource.nav_filter_removed_points;
  json << ",\"raw_geometry_cells\":" <<
    (build && build->success ? build->raw_geometry_cell_count : 0U);
  json << ",\"support_candidate_cells\":" <<
    (build && build->success ? build->support_candidate_count : 0U);
  json << ",\"projected_support_count\":" <<
    (projection ? projection->projected_support_samples.size() : 0U);
  json << ",\"proven_reachable_count\":" <<
    (projection ? projection->observed_seed_cells.size() : 0U);
  json << ",\"observed_seed_cells\":" <<
    (projection ? projection->observed_seed_cells.size() : 0U);
  json << ",\"bridge_seed_cells\":" <<
    (projection ? projection->bridge_seed_cells.size() : 0U);
  json << ",\"bridge_segments\":" <<
    (projection ? projection->bridge_segments.size() : 0U);
  json << ",\"rejected_projection_count\":" <<
    (projection ? projection->rejected_samples.size() : 0U);
  if (projection) {
    std::unordered_map<std::string, std::size_t> rejected_counts;
    for (const RejectedProjectionSample & sample : projection->rejected_samples) {
      ++rejected_counts[sample.reason];
    }
    json << ",\"support_projection_failed_count\":" <<
      rejected_counts["support_projection_failed"];
    json << ",\"support_projection_ambiguous_multifloor_count\":" <<
      rejected_counts["support_projection_ambiguous_multifloor"];
  } else {
    json << ",\"support_projection_failed_count\":0";
    json << ",\"support_projection_ambiguous_multifloor_count\":0";
  }
  json << ",\"footprint_rejected_count\":" <<
    (projection ? projection->footprint_rejected_samples : 0U);
  json << ",\"reanchored_support_count\":" <<
    (projection ? projection->reanchored_support_samples : 0U);
  json << ",\"retry_support_count\":" <<
    (projection ? projection->retry_support_samples : 0U);
  json << ",\"trajectory_bridge_seed_count\":" <<
    (projection ? projection->trajectory_bridge_seed_count : 0U);
  json << ",\"expanded_reachable_count\":" <<
    (build && build->success ? build->snapshot.surface.traversable_cells.size() : 0U);
  json << ",\"inferred_reachable_count\":" <<
    (build && build->success ? build->inferred_cell_count : 0U);
  json << ",\"rejected_expansion_count\":" <<
    (build && build->success ? build->rejected_expansion_count : 0U);
  json << ",\"body_obstructed_rejected_count\":" <<
    (build && build->success ? build->body_obstructed_rejected_count : 0U);
  json << ",\"anchor_envelope_rejected_count\":" <<
    (build && build->success ? build->anchor_envelope_rejected_count : 0U);
  json << ",\"hole_filled_count\":" <<
    (build && build->success ? build->hole_filled_count : 0U);
  json << ",\"bridge_used_as_expansion_anchor\":" <<
    (build && build->success ? build->bridge_used_as_expansion_anchor : 0U);
  json << ",\"hole_fill_from_bridge_rejected\":" <<
    (build && build->success ? build->hole_fill_from_bridge_rejected : 0U);
  json << ",\"support_components\":" <<
    (build && build->success ? build->support_component_count : 0U);
  json << ",\"anchored_support_components\":" <<
    (build && build->success ? build->anchored_support_component_count : 0U);
  json << ",\"rejected_unanchored_component_cells\":" <<
    (build && build->success ? build->rejected_unanchored_component_cells : 0U);
  json << ",\"boundary_count\":" <<
    (build && build->success ? build->snapshot.surface.boundary_cells.size() : 0U);
  json << ",\"clearance_count\":" <<
    (build && build->success ? build->snapshot.clearance.distances().size() : 0U);
  json << ",\"risk_count\":" <<
    (build && build->success ? build->snapshot.risk.risks().size() : 0U);
  json << ",\"planner_component_count\":" <<
    (surface_graph ? surface_graph->componentCount() : 0U);
  json << ",\"largest_planner_component_cells\":" <<
    (surface_graph ? surface_graph->largestComponentSize() : 0U);
  json << ",\"planner_multifloor_component_count\":" <<
    (surface_graph ?
    surface_graph->multifloorComponentCount(planner_multifloor_z_range_m) : 0U);
  if (surface_graph) {
    const auto & graph_metrics = surface_graph->metrics();
    json << ",\"graph_edges\":" << graph_metrics.graph_edges;
    json << ",\"graph_normal_edges\":" << graph_metrics.graph_normal_edges;
    json << ",\"graph_bridge_edges\":" << graph_metrics.graph_bridge_edges;
    json << ",\"graph_rejected_cross_component_edges\":" <<
      graph_metrics.graph_rejected_cross_component_edges;
    json << ",\"graph_rejected_large_dz_edges\":" <<
      graph_metrics.graph_rejected_large_dz_edges;
    json << ",\"graph_rejected_large_slope_edges\":" <<
      graph_metrics.graph_rejected_large_slope_edges;
    json << ",\"graph_rejected_invalid_bridge_edges\":" <<
      graph_metrics.graph_rejected_invalid_bridge_edges;
    json << ",\"max_graph_edge_dz_m\":" << graph_metrics.max_graph_edge_dz_m;
    json << ",\"max_graph_edge_slope\":" << graph_metrics.max_graph_edge_slope;
  } else {
    json << ",\"graph_edges\":0";
    json << ",\"graph_normal_edges\":0";
    json << ",\"graph_bridge_edges\":0";
    json << ",\"graph_rejected_cross_component_edges\":0";
    json << ",\"graph_rejected_large_dz_edges\":0";
    json << ",\"graph_rejected_large_slope_edges\":0";
    json << ",\"graph_rejected_invalid_bridge_edges\":0";
    json << ",\"max_graph_edge_dz_m\":0";
    json << ",\"max_graph_edge_slope\":0";
  }
  if (backbone_graph) {
    const auto & backbone_metrics = backbone_graph->metrics();
    json << ",\"backbone_nodes\":" << backbone_metrics.backbone_nodes;
    json << ",\"backbone_edges\":" << backbone_metrics.backbone_edges;
    json << ",\"backbone_z_range\":" <<
      (backbone_metrics.backbone_z_max - backbone_metrics.backbone_z_min);
    json << ",\"backbone_z_min\":" << backbone_metrics.backbone_z_min;
    json << ",\"backbone_z_max\":" << backbone_metrics.backbone_z_max;
    json << ",\"backbone_portals\":" << backbone_metrics.portals;
    json << ",\"max_backbone_edge_dz_m\":" << backbone_metrics.max_backbone_edge_dz_m;
    json << ",\"max_backbone_edge_slope\":" << backbone_metrics.max_backbone_edge_slope;
    json << ",\"backbone_body_to_support_z_m\":" <<
      backbone_metrics.inferred_body_to_support_z_m;
  } else {
    json << ",\"backbone_nodes\":0";
    json << ",\"backbone_edges\":0";
    json << ",\"backbone_z_range\":0";
    json << ",\"backbone_z_min\":0";
    json << ",\"backbone_z_max\":0";
    json << ",\"backbone_portals\":0";
    json << ",\"max_backbone_edge_dz_m\":0";
    json << ",\"max_backbone_edge_slope\":0";
    json << ",\"backbone_body_to_support_z_m\":0";
  }
  json << "}";
  return json.str();
}

std::string toPlanRouteStatsJson(
  const SurfacePlanResult & plan,
  double search_time_ms,
  double start_snap_distance_m,
  double goal_snap_distance_m)
{
  std::ostringstream json;
  json << "{";
  json << "\"kind\":\"plan_route\"";
  json << ",\"success\":" << (plan.success ? "true" : "false");
  json << ",\"message\":\"" << jsonEscape(plan.message) << "\"";
  json << ",\"search_time_ms\":" << search_time_ms;
  json << ",\"start_snap_distance_m\":" << start_snap_distance_m;
  json << ",\"goal_snap_distance_m\":" << goal_snap_distance_m;
  json << ",\"path_waypoints\":" << plan.path.size();
  json << ",\"path_length_m\":" << plan.metrics.path_length_m;
  json << ",\"max_path_edge_dz_m\":" << plan.metrics.max_path_edge_dz_m;
  json << ",\"start_portal_candidates\":" << plan.metrics.start_portal_candidates;
  json << ",\"goal_portal_candidates\":" << plan.metrics.goal_portal_candidates;
  json << ",\"evaluated_portal_pairs\":" << plan.metrics.evaluated_portal_pairs;
  json << ",\"hybrid_nodes\":" << plan.metrics.hybrid_nodes;
  json << ",\"hybrid_surface_edges\":" << plan.metrics.hybrid_surface_edges;
  json << ",\"hybrid_backbone_edges\":" << plan.metrics.hybrid_backbone_edges;
  json << ",\"hybrid_portal_edges\":" << plan.metrics.hybrid_portal_edges;
  json << ",\"hybrid_expanded_nodes\":" << plan.metrics.hybrid_expanded_nodes;
  json << ",\"used_backbone_edges\":" << plan.metrics.used_backbone_edges;
  json << ",\"used_portal_edges\":" << plan.metrics.used_portal_edges;
  json << ",\"used_surface_edges\":" << plan.metrics.used_surface_edges;
  json << ",\"backbone_path_length_m\":" << plan.metrics.backbone_path_length_m;
  json << ",\"surface_path_length_m\":" << plan.metrics.surface_path_length_m;
  json << ",\"portal_switch_count\":" << plan.metrics.portal_switch_count;
  json << ",\"selected_start_portal_id\":" << plan.metrics.selected_start_portal_id;
  json << ",\"selected_goal_portal_id\":" << plan.metrics.selected_goal_portal_id;
  json << ",\"selected_start_backbone_node\":" <<
    plan.metrics.selected_start_backbone_node;
  json << ",\"selected_goal_backbone_node\":" <<
    plan.metrics.selected_goal_backbone_node;
  json << ",\"selected_backbone_index_delta\":" <<
    plan.metrics.selected_backbone_index_delta;
  json << ",\"selected_backbone_length_m\":" << plan.metrics.selected_backbone_length_m;
  json << ",\"selected_start_surface_leg_m\":" <<
    plan.metrics.selected_start_surface_leg_m;
  json << ",\"selected_goal_surface_leg_m\":" <<
    plan.metrics.selected_goal_surface_leg_m;
  json << ",\"selected_total_hybrid_cost\":" << plan.metrics.selected_total_hybrid_cost;
  json << "}";
  return json.str();
}

sensor_msgs::msg::PointCloud2 makePointCloud(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const std::vector<Point3> & points)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = frame_id;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  for (const Point3 & point : points) {
    *iter_x = static_cast<float>(point.x);
    *iter_y = static_cast<float>(point.y);
    *iter_z = static_cast<float>(point.z);
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
  return cloud;
}

sensor_msgs::msg::PointCloud2 makeIntensityCloud(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const std::vector<IntensityPoint> & points)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = frame_id;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_i(cloud, "intensity");
  for (const IntensityPoint & point : points) {
    *iter_x = static_cast<float>(point.point.x);
    *iter_y = static_cast<float>(point.point.y);
    *iter_z = static_cast<float>(point.point.z);
    *iter_i = static_cast<float>(point.intensity);
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_i;
  }
  return cloud;
}

Point3 cellCenter(const GridIndex & cell, double resolution_m)
{
  return {
    (static_cast<double>(cell.x) + 0.5) * resolution_m,
    (static_cast<double>(cell.y) + 0.5) * resolution_m,
    (static_cast<double>(cell.z) + 0.5) * resolution_m};
}

Point3 posePoint(const geometry_msgs::msg::PoseStamped & pose)
{
  return {pose.pose.position.x, pose.pose.position.y, pose.pose.position.z};
}

Point3 stampedPoint(const geometry_msgs::msg::PointStamped & point)
{
  return {point.point.x, point.point.y, point.point.z};
}

geometry_msgs::msg::Quaternion quaternionFromYaw(const double yaw_rad)
{
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(0.5 * yaw_rad);
  q.z = std::sin(0.5 * yaw_rad);
  return q;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

std::string pathKindName(const tgw_planner::core::PathPointKind kind)
{
  switch (kind) {
    case tgw_planner::core::PathPointKind::Surface:
      return "surface";
    case tgw_planner::core::PathPointKind::Backbone:
      return "backbone";
    case tgw_planner::core::PathPointKind::Portal:
      return "portal";
    case tgw_planner::core::PathPointKind::Unknown:
    default:
      return "unknown";
  }
}

double xyDistance(const Point3 & a, const Point3 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double pointPathLength(const std::vector<Point3> & path)
{
  double length_m = 0.0;
  for (std::size_t i = 1U; i < path.size(); ++i) {
    length_m += xyDistance(path[i - 1U], path[i]);
  }
  return length_m;
}

double wrapAngle(const double angle_rad)
{
  constexpr double kPi = 3.14159265358979323846;
  double wrapped = angle_rad;
  while (wrapped > kPi) {
    wrapped -= 2.0 * kPi;
  }
  while (wrapped <= -kPi) {
    wrapped += 2.0 * kPi;
  }
  return wrapped;
}

std::pair<double, double> pointPathSmoothness(const std::vector<Point3> & path)
{
  if (path.size() < 3U) {
    return {0.0, 0.0};
  }
  double total_turn_rad = 0.0;
  double max_turn_rad = 0.0;
  for (std::size_t i = 1U; i + 1U < path.size(); ++i) {
    const Point3 & a = path[i - 1U];
    const Point3 & b = path[i];
    const Point3 & c = path[i + 1U];
    if (xyDistance(a, b) <= 1.0e-9 || xyDistance(b, c) <= 1.0e-9) {
      continue;
    }
    const double heading_ab = std::atan2(b.y - a.y, b.x - a.x);
    const double heading_bc = std::atan2(c.y - b.y, c.x - b.x);
    const double turn = std::abs(wrapAngle(heading_bc - heading_ab));
    total_turn_rad += turn;
    max_turn_rad = std::max(max_turn_rad, turn);
  }
  return {total_turn_rad / std::max(1.0e-9, pointPathLength(path)), max_turn_rad};
}

std::string metersText(const double value_m)
{
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(2);
  out << value_m << "m";
  return out.str();
}

nav_msgs::msg::Path makePathMsg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const std::vector<Point3> & path)
{
  nav_msgs::msg::Path msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.poses.reserve(path.size());
  for (const Point3 & point : path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg.header;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = point.z;
    pose.pose.orientation.w = 1.0;
    msg.poses.push_back(pose);
  }
  return msg;
}

visualization_msgs::msg::Marker makeSphereMarker(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const std::string & ns,
  const int id,
  const Point3 & point,
  const double scale_m,
  const float r,
  const float g,
  const float b)
{
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = stamp;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = point.x;
  marker.pose.position.y = point.y;
  marker.pose.position.z = point.z + 0.25;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = scale_m;
  marker.scale.y = scale_m;
  marker.scale.z = scale_m;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0F;
  return marker;
}

visualization_msgs::msg::Marker makeTextMarker(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const std::string & ns,
  const int id,
  const Point3 & point,
  const std::string & text,
  const double scale_m,
  const float r,
  const float g,
  const float b)
{
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = stamp;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = point.x;
  marker.pose.position.y = point.y;
  marker.pose.position.z = point.z + 0.7;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = scale_m;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0F;
  marker.text = text;
  return marker;
}
}  // namespace

class TgwExperiencePlannerNode : public rclcpp::Node
{
public:
  TgwExperiencePlannerNode()
  : Node("tgw_experience_planner_node")
  {
    const TrajectoryProjectorOptions projector_defaults = defaultTrajectoryProjectorOptions();
    const ExperienceSurfaceBuilderOptions builder_defaults =
      defaultExperienceSurfaceBuilderOptions();
    const SurfacePlannerOptions planner_defaults = defaultSurfacePlannerOptions();
    const SurfaceGraphBuildOptions graph_defaults = defaultSurfaceGraphBuildOptions();
    const ExperienceBackboneOptions backbone_defaults = defaultExperienceBackboneOptions();
    const HybridExperiencePlannerOptions hybrid_defaults = defaultHybridExperiencePlannerOptions();

    declare_parameter<std::string>("pbstream_path", "");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<double>("nav_resolution_m", projector_defaults.resolution_m);
    declare_parameter<double>("raw_resolution_m", projector_defaults.raw_resolution_m);
    declare_parameter<double>("lidar_to_footprint_x_m", projector_defaults.lidar_to_footprint_x_m);
    declare_parameter<double>("lidar_to_footprint_y_m", projector_defaults.lidar_to_footprint_y_m);
    declare_parameter<double>("support_search_below_min_m", projector_defaults.search_below_min_m);
    declare_parameter<double>("support_search_below_max_m", projector_defaults.search_below_max_m);
    declare_parameter<double>("support_max_jump_m", projector_defaults.max_support_jump_m);
    declare_parameter<bool>(
      "allow_support_reanchor_on_jump", projector_defaults.allow_support_reanchor_on_jump);
    declare_parameter<int>(
      "support_xy_search_radius_cells", projector_defaults.support_xy_search_radius_cells);
    declare_parameter<int>(
      "support_xy_retry_radius_cells", projector_defaults.support_xy_retry_radius_cells);
    declare_parameter<double>(
      "max_trajectory_bridge_gap_m", projector_defaults.max_trajectory_bridge_gap_m);
    declare_parameter<double>(
      "max_trajectory_bridge_height_delta_m",
      projector_defaults.max_trajectory_bridge_height_delta_m);
    declare_parameter<double>(
      "trajectory_bridge_sample_step_m", projector_defaults.trajectory_bridge_sample_step_m);
    declare_parameter<double>("robot_length_m", projector_defaults.footprint_length_m);
    declare_parameter<double>("robot_width_m", projector_defaults.footprint_width_m);
    declare_parameter<double>("body_clearance_height_m", builder_defaults.body_clearance_height_m);
    declare_parameter<double>("base_to_front_m", projector_defaults.footprint_base_to_front_m);
    declare_parameter<double>(
      "min_footprint_support_ratio", projector_defaults.min_footprint_support_ratio);
    declare_parameter<double>(
      "footprint_support_height_tolerance_m",
      projector_defaults.footprint_support_height_tolerance_m);
    declare_parameter<int>(
      "expansion_radius_cells", builder_defaults.expander.expansion_radius_cells);
    declare_parameter<int>("max_expansion_steps", builder_defaults.expander.max_expansion_steps);
    declare_parameter<int>(
      "vertical_tolerance_cells", builder_defaults.expander.vertical_tolerance_cells);
    declare_parameter<double>(
      "max_expansion_step_height_m", builder_defaults.expander.max_expansion_step_height_m);
    declare_parameter<int>(
      "experience_anchor_radius_cells",
      builder_defaults.expander.experience_anchor_radius_cells);
    declare_parameter<double>(
      "experience_anchor_height_tolerance_m",
      builder_defaults.expander.experience_anchor_height_tolerance_m);
    declare_parameter<int>(
      "experience_anchor_vertical_tolerance_cells",
      builder_defaults.expander.experience_anchor_vertical_tolerance_cells);
    declare_parameter<bool>("enable_hole_filling", builder_defaults.expander.enable_hole_filling);
    declare_parameter<int>("hole_fill_iterations", builder_defaults.expander.hole_fill_iterations);
    declare_parameter<int>(
      "min_hole_fill_neighbors", builder_defaults.expander.min_hole_fill_neighbors);
    declare_parameter<double>(
      "max_hole_fill_height_spread_m",
      builder_defaults.expander.max_hole_fill_height_spread_m);
    declare_parameter<double>("plan_max_step_height_m", planner_defaults.max_step_height_m);
    declare_parameter<int>("plan_max_iterations", static_cast<int>(planner_defaults.max_iterations));
    declare_parameter<double>("plan_bridge_cost", planner_defaults.w_bridge);
    declare_parameter<double>("max_start_snap_distance_m", 1.50);
    declare_parameter<double>("max_goal_snap_distance_m", 1.50);
    declare_parameter<double>("graph_max_normal_edge_slope", graph_defaults.max_edge_slope);
    declare_parameter<double>("graph_max_bridge_edge_slope", graph_defaults.max_bridge_edge_slope);
    declare_parameter<double>(
      "graph_bridge_attach_max_dz_m", graph_defaults.max_bridge_attach_height_delta_m);
    declare_parameter<double>(
      "planner_footprint_min_support_ratio", planner_defaults.footprint.min_support_ratio);
    declare_parameter<double>(
      "backbone_min_node_spacing_m", backbone_defaults.min_node_spacing_m);
    declare_parameter<double>(
      "backbone_max_portal_xy_distance_m", backbone_defaults.max_portal_xy_distance_m);
    declare_parameter<double>(
      "backbone_max_portal_height_error_m", backbone_defaults.max_portal_height_error_m);
    declare_parameter<double>(
      "backbone_min_portal_clearance_m", backbone_defaults.min_portal_clearance_m);
    declare_parameter<int>(
      "backbone_max_portals_per_node",
      static_cast<int>(backbone_defaults.max_portals_per_node));
    declare_parameter<double>("hybrid_backbone_cost_scale", hybrid_defaults.backbone_cost_scale);
    declare_parameter<double>("hybrid_portal_switch_cost", hybrid_defaults.portal_switch_cost);
    declare_parameter<double>(
      "hybrid_portal_height_error_weight", hybrid_defaults.portal_height_error_weight);
    declare_parameter<double>(
      "hybrid_backbone_low_confidence_penalty",
      hybrid_defaults.backbone_low_confidence_penalty);
    declare_parameter<double>(
      "hybrid_max_backbone_edge_xy_gap_m", hybrid_defaults.max_backbone_edge_xy_gap_m);
    declare_parameter<double>(
      "hybrid_max_backbone_edge_dz_m", hybrid_defaults.max_backbone_edge_dz_m);
    declare_parameter<double>(
      "hybrid_max_backbone_edge_slope", hybrid_defaults.max_backbone_edge_slope);
    declare_parameter<double>(
      "hybrid_max_portal_xy_distance_m", hybrid_defaults.max_portal_xy_distance_m);
    declare_parameter<double>(
      "hybrid_max_portal_height_error_m", hybrid_defaults.max_portal_height_error_m);
    declare_parameter<double>(
      "hybrid_surface_target_speed_mps", hybrid_defaults.surface_target_speed_mps);
    declare_parameter<double>(
      "hybrid_backbone_target_speed_mps", hybrid_defaults.backbone_target_speed_mps);
    declare_parameter<double>(
      "hybrid_portal_target_speed_mps", hybrid_defaults.portal_target_speed_mps);
    declare_parameter<double>("planner_multifloor_z_range_m", 1.50);
    declare_parameter<int>("max_trajectory_points", 200000);
    declare_parameter<int>("max_geometry_debug_points", 400000);
    declare_parameter<std::string>("planner_log_dir", "logs");
    declare_parameter<int>("planner_log_retention_days", 7);
    declare_parameter<bool>("enable_path_tracking", false);
    declare_parameter<std::string>("tracking_odom_topic", "/odom");
    declare_parameter<std::string>("tracking_cmd_vel_topic", "/tgw_experience/cmd_vel");
    declare_parameter<bool>("enable_kinematic_replay", false);
    declare_parameter<std::string>(
      "kinematic_replay_odom_topic", "/tgw_experience/fake_odom");
    declare_parameter<bool>("kinematic_replay_reset_on_plan", true);
    declare_parameter<int>("kinematic_replay_trace_max_points", 5000);
    declare_parameter<double>("tracking_frequency_hz", 20.0);
    declare_parameter<double>("tracking_odom_timeout_s", 0.50);
    declare_parameter<double>("tracking_lookahead_m", 0.80);
    declare_parameter<double>("tracking_projection_window_m", 5.0);
    declare_parameter<double>("tracking_local_route_length_m", 4.0);
    declare_parameter<double>("tracking_max_yaw_rate_radps", 1.20);
    declare_parameter<double>("tracking_goal_tolerance_m", 0.35);
    declare_parameter<double>("tracking_desired_linear_speed_mps", 0.60);
    declare_parameter<double>("tracking_max_linear_speed_mps", 0.80);
    declare_parameter<double>("tracking_min_linear_speed_mps", 0.02);
    declare_parameter<double>("tracking_max_lateral_accel_mps2", 0.80);
    declare_parameter<double>("tracking_goal_slowdown_distance_m", 1.00);
    declare_parameter<double>("tracking_max_linear_accel_mps2", 0.80);
    declare_parameter<double>("local_path_min_point_spacing_m", 0.20);
    declare_parameter<double>("local_path_max_point_spacing_m", 0.20);
    declare_parameter<bool>("local_path_enable_collision_check", true);
    declare_parameter<double>("local_path_bezier_handle_ratio", 0.35);
    declare_parameter<double>("local_path_max_smoothness_rad_per_m", 2.50);
    declare_parameter<double>("local_path_max_turn_angle_rad", 1.20);
    declare_parameter<double>("local_path_max_route_deviation_m", 0.45);
    declare_parameter<double>("local_path_corner_cut_turn_angle_rad", 0.75);
    declare_parameter<double>("local_path_min_corner_target_distance_m", 0.60);
    declare_parameter<double>("local_map_robot_radius_m", 0.35);
    declare_parameter<double>("local_map_inflation_radius_m", 0.10);
    declare_parameter<bool>("enable_local_obstacle_cloud", false);
    declare_parameter<std::string>("local_obstacle_cloud_topic", "/tgw_experience/local_cloud");
    declare_parameter<double>("local_map_time_window_s", 1.50);
    declare_parameter<int>("local_map_max_obstacle_points", 50000);
    declare_parameter<int>("local_obstacle_max_points_per_cloud", 20000);
    declare_parameter<double>("local_obstacle_max_range_m", 5.0);
    declare_parameter<double>("local_obstacle_min_height_above_surface_m", 0.15);
    declare_parameter<double>("local_obstacle_max_height_above_surface_m", 1.20);
    declare_parameter<bool>("local_obstacle_requires_map_frame", true);
    declare_parameter<int>("local_obstacle_debug_max_points", 50000);
    declare_parameter<double>("local_collision_time_horizon_s", 1.50);
    declare_parameter<double>("local_collision_sample_time_s", 0.10);

    configurePlannerFileLogging(
      get_parameter("planner_log_dir").as_string(),
      static_cast<int>(get_parameter("planner_log_retention_days").as_int()));

    const auto latched_qos = rclcpp::QoS(1).transient_local().reliable();
    trajectory_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tgw_experience/trajectory_cloud", latched_qos);
    keyframe_geometry_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tgw_experience/keyframe_geometry_cloud", latched_qos);
    support_candidate_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tgw_experience/support_candidate_cloud", latched_qos);
    projected_support_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tgw_experience/projected_support_cloud", latched_qos);
    trajectory_bridge_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tgw_experience/trajectory_bridge_cloud", latched_qos);
    proven_reachable_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tgw_experience/proven_reachable_cloud", latched_qos);
    expanded_reachable_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tgw_experience/expanded_reachable_cloud", latched_qos);
    rejected_projection_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tgw_experience/rejected_projection_cloud", latched_qos);
    rejected_no_support_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tgw_experience/rejected_no_support_cloud", latched_qos);
    rejected_ambiguous_multifloor_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tgw_experience/rejected_ambiguous_multifloor_cloud", latched_qos);
    boundary_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tgw_experience/boundary_cloud", latched_qos);
    clearance_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tgw_experience/clearance_cloud", latched_qos);
    risk_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tgw_experience/risk_cloud", latched_qos);
    planner_component_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tgw_experience/planner_component_cloud", latched_qos);
    backbone_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tgw_experience/backbone_cloud", latched_qos);
    portal_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tgw_experience/portal_cloud", latched_qos);
    start_portal_candidates_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tgw_experience/start_portal_candidates", latched_qos);
    goal_portal_candidates_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tgw_experience/goal_portal_candidates", latched_qos);
    selected_start_portal_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tgw_experience/selected_start_portal", latched_qos);
    selected_goal_portal_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tgw_experience/selected_goal_portal", latched_qos);
    selected_backbone_segment_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tgw_experience/selected_backbone_segment", latched_qos);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/tgw_experience/path", latched_qos);
    hybrid_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/tgw_experience/hybrid_path", latched_qos);
    local_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/tgw_experience/local_path", rclcpp::QoS(10));
    used_backbone_segment_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tgw_experience/used_backbone_segment", latched_qos);
    used_portals_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tgw_experience/used_portals", latched_qos);
    raw_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/tgw_experience/raw_path", latched_qos);
    start_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/tgw_experience/start_marker", latched_qos);
    goal_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/tgw_experience/goal_marker", latched_qos);
    plan_status_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/tgw_experience/plan_status_marker", latched_qos);
    tracker_lookahead_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/tgw_experience/tracker_lookahead_marker", rclcpp::QoS(10));
    tracker_projected_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/tgw_experience/tracker_projected_marker", rclcpp::QoS(10));
    tracking_status_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/tgw_experience/tracking_status_marker", rclcpp::QoS(10));
    local_obstacle_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tgw_experience/local_obstacle_cloud", rclcpp::QoS(10));
    stats_json_pub_ = create_publisher<std_msgs::msg::String>(
      "/tgw_experience/stats_json", latched_qos);
    tracking_status_json_pub_ = create_publisher<std_msgs::msg::String>(
      "/tgw_experience/tracking_status_json", rclcpp::QoS(10));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      get_parameter("tracking_cmd_vel_topic").as_string(), rclcpp::QoS(10));
    fake_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      get_parameter("kinematic_replay_odom_topic").as_string(), rclcpp::QoS(10));
    sim_robot_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/tgw_experience/sim_robot_path", rclcpp::QoS(10));
    sim_robot_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/tgw_experience/sim_robot_marker", rclcpp::QoS(10));
    plan_path_srv_ = create_service<tgw_planner::srv::PlanPath>(
      "/tgw_experience/plan_path",
      std::bind(
        &TgwExperiencePlannerNode::handlePlanPath, this, std::placeholders::_1,
        std::placeholders::_2));
    clear_plan_srv_ = create_service<std_srvs::srv::Trigger>(
      "/tgw_experience/clear_plan",
      std::bind(
        &TgwExperiencePlannerNode::handleClearPlan, this, std::placeholders::_1,
        std::placeholders::_2));
    start_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/start_point", rclcpp::QoS(10),
      std::bind(&TgwExperiencePlannerNode::onStartPoint, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", rclcpp::QoS(10),
      std::bind(&TgwExperiencePlannerNode::onGoalPoint, this, std::placeholders::_1));
    start_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/start_pose", rclcpp::QoS(10),
      std::bind(&TgwExperiencePlannerNode::onStartPose, this, std::placeholders::_1));
    goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", rclcpp::QoS(10),
      std::bind(&TgwExperiencePlannerNode::onGoalPose, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("tracking_odom_topic").as_string(), rclcpp::QoS(20),
      std::bind(&TgwExperiencePlannerNode::onOdom, this, std::placeholders::_1));
    local_obstacle_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      get_parameter("local_obstacle_cloud_topic").as_string(), rclcpp::QoS(10),
      std::bind(&TgwExperiencePlannerNode::onLocalObstacleCloud, this, std::placeholders::_1));

    route_tracker_ = RouteProgressTracker(routeTrackerOptions());
    local_path_smoother_ = LocalPathSmoother(localPathSmootherOptions());
    rolling_local_map_ = RollingLocalMap(rollingLocalMapOptions());
    rpp_controller_ = RegulatedPurePursuitController(rppOptions());
    const double tracking_hz = std::max(1.0, get_parameter("tracking_frequency_hz").as_double());
    tracking_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / tracking_hz)),
      std::bind(&TgwExperiencePlannerNode::onTrackingTimer, this));

    loadResource();
  }

private:
  void loadResource()
  {
    const std::string pbstream_path = get_parameter("pbstream_path").as_string();
    if (pbstream_path.empty()) {
      N3MapReadResult result;
      result.error_code = "pbstream_open_failed";
      result.message = "pbstream_path parameter is required";
      publishStats(result, pbstream_path, nullptr, nullptr);
      LOG(ERROR) << result.message;
      return;
    }

    const N3MapReadResult result = reader_.readPbstream(pbstream_path);
    if (!result.success) {
      publishStats(result, pbstream_path, nullptr, nullptr);
      LOG(ERROR) << "pbstream load failed: [" << result.error_code << "] " << result.message;
      return;
    }

    const TrajectoryProjectionResult projection = TrajectoryProjector(projectorOptions()).project(
      result.resource);
    const ExperienceBuildResult build = ExperienceSurfaceBuilder(builderOptions()).build(
      result.resource);
    if (build.success) {
      snapshot_ = build.snapshot;
      has_snapshot_ = true;
      const SurfacePlannerOptions planner_options = plannerOptions();
      SurfaceGraphBuildOptions graph_options = defaultSurfaceGraphBuildOptions();
      graph_options.max_edge_height_delta_m = planner_options.max_step_height_m;
      graph_options.max_bridge_edge_height_delta_m =
        get_parameter("max_trajectory_bridge_height_delta_m").as_double();
      graph_options.max_bridge_attach_height_delta_m =
        get_parameter("graph_bridge_attach_max_dz_m").as_double();
      graph_options.max_edge_slope = get_parameter("graph_max_normal_edge_slope").as_double();
      graph_options.max_bridge_edge_slope =
        get_parameter("graph_max_bridge_edge_slope").as_double();
      surface_graph_.build(
        snapshot_, SurfaceTransitionValidator(planner_options), planner_options, graph_options);
      has_surface_graph_ = true;
      backbone_graph_.build(result.resource, projection, surface_graph_, backboneOptions());
      has_backbone_graph_ = true;
    }
    publishKeyframeGeometry(result.resource);
    publishTrajectory(result.resource);
    publishProjectionDebug(result.resource, projection);
    publishExpansionDebug(result.resource, build);
    publishPlannerConnectivityDebug(result.resource);
    publishBackboneDebug(result.resource);
    publishStats(result, pbstream_path, &projection, &build);
    std::unordered_map<std::string, std::size_t> rejected_counts;
    for (const RejectedProjectionSample & sample : projection.rejected_samples) {
      ++rejected_counts[sample.reason];
    }
    LOG(INFO)
      << "loaded n3map experience resource"
      << " keyframes=" << result.resource.keyframes.size()
      << " dense_trajectory=" << result.resource.dense_trajectory.size()
      << " raw_geometry=" << (build.success ? build.raw_geometry_cell_count : 0U)
      << " support_candidates=" << (build.success ? build.support_candidate_count : 0U)
      << " projected_support=" << projection.projected_support_samples.size()
      << " observed_seed=" << projection.observed_seed_cells.size()
      << " bridge_seed=" << projection.bridge_seed_cells.size()
      << " expanded_reachable=" <<
      (build.success ? build.snapshot.surface.traversable_cells.size() : 0U)
      << " planner_components=" << (has_surface_graph_ ? surface_graph_.componentCount() : 0U)
      << " largest_planner_component=" <<
      (has_surface_graph_ ? surface_graph_.largestComponentSize() : 0U)
      << " planner_multifloor_components=" <<
      (has_surface_graph_ ?
      surface_graph_.multifloorComponentCount(
        get_parameter("planner_multifloor_z_range_m").as_double()) : 0U)
      << " backbone_nodes=" << (has_backbone_graph_ ? backbone_graph_.nodes().size() : 0U)
      << " backbone_edges=" << (has_backbone_graph_ ? backbone_graph_.edges().size() : 0U)
      << " backbone_portals=" << (has_backbone_graph_ ? backbone_graph_.portals().size() : 0U)
      << " rejected_projection=" << projection.rejected_samples.size()
      << " no_support=" << rejected_counts["support_projection_failed"]
      << " ambiguous_multifloor=" <<
      rejected_counts["support_projection_ambiguous_multifloor"]
      << " reanchored_support=" << projection.reanchored_support_samples
      << " retry_support=" << projection.retry_support_samples
      << " bridge_used_as_expansion_anchor=" <<
      (build.success ? build.bridge_used_as_expansion_anchor : 0U)
      << " support_components=" << (build.success ? build.support_component_count : 0U)
      << " anchored_components=" <<
      (build.success ? build.anchored_support_component_count : 0U)
      << " rejected_unanchored_component=" <<
      (build.success ? build.rejected_unanchored_component_cells : 0U)
      << " footprint_rejected=" << projection.footprint_rejected_samples
      << " body_obstructed_rejected=" <<
      (build.success ? build.body_obstructed_rejected_count : 0U)
      << " anchor_envelope_rejected=" <<
      (build.success ? build.anchor_envelope_rejected_count : 0U)
      << " hole_filled=" << (build.success ? build.hole_filled_count : 0U)
      << " frame=" << result.resource.map_frame
      << " body=" << result.resource.body_frame;
    if (!build.success) {
      LOG(WARNING) << "experience expansion failed: [" << build.error_code << "] " <<
        build.message;
    }
  }

  ExperienceBackboneOptions backboneOptions() const
  {
    ExperienceBackboneOptions options = defaultExperienceBackboneOptions();
    options.min_node_spacing_m = get_parameter("backbone_min_node_spacing_m").as_double();
    options.max_portal_xy_distance_m =
      get_parameter("backbone_max_portal_xy_distance_m").as_double();
    options.max_portal_height_error_m =
      get_parameter("backbone_max_portal_height_error_m").as_double();
    options.min_portal_clearance_m =
      get_parameter("backbone_min_portal_clearance_m").as_double();
    options.max_portals_per_node = static_cast<std::size_t>(
      std::max<std::int64_t>(1, get_parameter("backbone_max_portals_per_node").as_int()));
    return options;
  }

  TrajectoryProjectorOptions projectorOptions() const
  {
    TrajectoryProjectorOptions options = defaultTrajectoryProjectorOptions();
    options.resolution_m = get_parameter("nav_resolution_m").as_double();
    options.raw_resolution_m = get_parameter("raw_resolution_m").as_double();
    options.lidar_to_footprint_x_m = get_parameter("lidar_to_footprint_x_m").as_double();
    options.lidar_to_footprint_y_m = get_parameter("lidar_to_footprint_y_m").as_double();
    options.search_below_min_m = get_parameter("support_search_below_min_m").as_double();
    options.search_below_max_m = get_parameter("support_search_below_max_m").as_double();
    options.max_support_jump_m = get_parameter("support_max_jump_m").as_double();
    options.allow_support_reanchor_on_jump =
      get_parameter("allow_support_reanchor_on_jump").as_bool();
    options.support_xy_search_radius_cells = static_cast<int>(
      get_parameter("support_xy_search_radius_cells").as_int());
    options.support_xy_retry_radius_cells = static_cast<int>(
      get_parameter("support_xy_retry_radius_cells").as_int());
    options.max_trajectory_bridge_gap_m =
      get_parameter("max_trajectory_bridge_gap_m").as_double();
    options.max_trajectory_bridge_height_delta_m =
      get_parameter("max_trajectory_bridge_height_delta_m").as_double();
    options.trajectory_bridge_sample_step_m =
      get_parameter("trajectory_bridge_sample_step_m").as_double();
    options.footprint_length_m = get_parameter("robot_length_m").as_double();
    options.footprint_width_m = get_parameter("robot_width_m").as_double();
    options.footprint_base_to_front_m = get_parameter("base_to_front_m").as_double();
    options.min_footprint_support_ratio =
      get_parameter("min_footprint_support_ratio").as_double();
    options.footprint_support_height_tolerance_m =
      get_parameter("footprint_support_height_tolerance_m").as_double();
    return options;
  }

  void publishKeyframeGeometry(const N3NavResource & resource)
  {
    std::size_t total_points = 0U;
    for (const auto & keyframe : resource.keyframes) {
      total_points += keyframe.cloud_body.size();
    }
    if (total_points == 0U) {
      return;
    }

    const auto max_points = static_cast<std::size_t>(
      std::max<std::int64_t>(1, get_parameter("max_geometry_debug_points").as_int()));
    const std::size_t stride = total_points > max_points ?
      ((total_points + max_points - 1U) / max_points) : 1U;
    std::vector<Point3> points;
    points.reserve((total_points + stride - 1U) / stride);

    std::size_t index = 0U;
    for (const auto & keyframe : resource.keyframes) {
      for (const auto & point_body : keyframe.cloud_body) {
        if ((index % stride) == 0U) {
          points.push_back(tgw_planner::core::transformPoint(
            keyframe.pose_optimized, {point_body.x, point_body.y, point_body.z}));
        }
        ++index;
      }
    }

    const std::string frame_id = resource.map_frame.empty() ?
      get_parameter("map_frame").as_string() : resource.map_frame;
    keyframe_geometry_pub_->publish(makePointCloud(now(), frame_id, points));
  }

  ExperienceSurfaceBuilderOptions builderOptions() const
  {
    ExperienceSurfaceBuilderOptions options = defaultExperienceSurfaceBuilderOptions();
    options.resolution_m = get_parameter("nav_resolution_m").as_double();
    options.body_clearance_height_m = get_parameter("body_clearance_height_m").as_double();
    options.projector = projectorOptions();
    options.expander.expansion_radius_cells = static_cast<int>(
      get_parameter("expansion_radius_cells").as_int());
    options.expander.max_expansion_steps = static_cast<int>(
      get_parameter("max_expansion_steps").as_int());
    options.expander.vertical_tolerance_cells = static_cast<int>(
      get_parameter("vertical_tolerance_cells").as_int());
    options.expander.max_expansion_step_height_m =
      get_parameter("max_expansion_step_height_m").as_double();
    options.expander.experience_anchor_radius_cells = static_cast<int>(
      get_parameter("experience_anchor_radius_cells").as_int());
    options.expander.experience_anchor_height_tolerance_m =
      get_parameter("experience_anchor_height_tolerance_m").as_double();
    options.expander.experience_anchor_vertical_tolerance_cells = static_cast<int>(
      get_parameter("experience_anchor_vertical_tolerance_cells").as_int());
    options.expander.enable_hole_filling = get_parameter("enable_hole_filling").as_bool();
    options.expander.hole_fill_iterations = static_cast<int>(
      get_parameter("hole_fill_iterations").as_int());
    options.expander.min_hole_fill_neighbors = static_cast<int>(
      get_parameter("min_hole_fill_neighbors").as_int());
    options.expander.max_hole_fill_height_spread_m =
      get_parameter("max_hole_fill_height_spread_m").as_double();
    return options;
  }

  void publishStats(
    const N3MapReadResult & result,
    const std::string & pbstream_path,
    const TrajectoryProjectionResult * projection,
    const ExperienceBuildResult * build)
  {
    std_msgs::msg::String msg;
    msg.data = toStatsJson(
      result, pbstream_path, projection, build,
      has_surface_graph_ ? &surface_graph_ : nullptr,
      has_backbone_graph_ ? &backbone_graph_ : nullptr,
      get_parameter("planner_multifloor_z_range_m").as_double());
    stats_json_pub_->publish(msg);
  }

  SurfacePlannerOptions plannerOptions() const
  {
    SurfacePlannerOptions options = defaultSurfacePlannerOptions();
    options.max_step_height_m = get_parameter("plan_max_step_height_m").as_double();
    options.max_iterations = static_cast<std::uint32_t>(
      std::max<std::int64_t>(1, get_parameter("plan_max_iterations").as_int()));
    options.w_bridge = get_parameter("plan_bridge_cost").as_double();
    options.footprint.length_m = get_parameter("robot_length_m").as_double();
    options.footprint.width_m = get_parameter("robot_width_m").as_double();
    options.footprint.base_to_front_m = get_parameter("base_to_front_m").as_double();
    options.footprint.height_m = get_parameter("body_clearance_height_m").as_double();
    options.footprint.support_height_tolerance_m =
      get_parameter("footprint_support_height_tolerance_m").as_double();
    options.footprint.min_support_ratio =
      get_parameter("planner_footprint_min_support_ratio").as_double();
    options.require_footprint_support = true;
    options.enable_shortcut = true;
    return options;
  }

  bool nearestSurfaceGraphNode(
    const Point3 & point, SurfaceNodeId * nearest_node, double * nearest_distance_m,
    int * component_id = nullptr) const
  {
    if (!has_snapshot_ || !has_surface_graph_) {
      return false;
    }
    const double layer_tolerance_m = 0.75 * snapshot_.resolution_m;
    const double z_ceiling = point.z + 0.5 * snapshot_.resolution_m;
    double nearest_xy_distance = std::numeric_limits<double>::infinity();
    double target_layer_z = -std::numeric_limits<double>::infinity();
    bool found_layer = false;

    for (const SurfaceNode & node : surface_graph_.nodes()) {
      const Point3 center{
        (static_cast<double>(node.x) + 0.5) * surface_graph_.resolution(),
        (static_cast<double>(node.y) + 0.5) * surface_graph_.resolution(),
        node.z};
      const double dx = point.x - center.x;
      const double dy = point.y - center.y;
      const double xy_distance = std::sqrt(dx * dx + dy * dy);
      if (center.z > z_ceiling) {
        continue;
      }
      if (xy_distance > nearest_xy_distance + snapshot_.resolution_m) {
        continue;
      }
      if (xy_distance < nearest_xy_distance - snapshot_.resolution_m) {
        nearest_xy_distance = xy_distance;
        target_layer_z = center.z;
        found_layer = true;
        continue;
      }
      if (!found_layer || center.z > target_layer_z) {
        target_layer_z = center.z;
        found_layer = true;
      }
    }

    if (!found_layer) {
      *nearest_distance_m = 0.0;
      return false;
    }

    const double required_clearance_m = std::max(
      get_parameter("robot_width_m").as_double(), snapshot_.resolution_m);
    double best_safe_score = std::numeric_limits<double>::infinity();
    double best_fallback_score = -std::numeric_limits<double>::infinity();
    SurfaceNodeId best_safe_node;
    SurfaceNodeId best_fallback_node;
    double best_distance = 0.0;
    double best_fallback_distance = 0.0;
    bool found_safe = false;
    bool found_fallback = false;

    int best_safe_component = -1;
    int best_fallback_component = -1;
    for (const SurfaceNode & node : surface_graph_.nodes()) {
      const Point3 center{
        (static_cast<double>(node.x) + 0.5) * surface_graph_.resolution(),
        (static_cast<double>(node.y) + 0.5) * surface_graph_.resolution(),
        node.z};
      if (std::abs(center.z - target_layer_z) > layer_tolerance_m) {
        continue;
      }
      const double dx = point.x - center.x;
      const double dy = point.y - center.y;
      const double xy_distance = std::sqrt(dx * dx + dy * dy);

      const double clearance = node.clearance_m;
      if (clearance >= required_clearance_m) {
        const double score = xy_distance + 0.05 / std::max(clearance, 1.0e-3);
        if (!found_safe || score < best_safe_score) {
          best_safe_score = score;
          best_distance = xy_distance;
          best_safe_node = node.id;
          best_safe_component = surface_graph_.componentId(node.id);
          found_safe = true;
        }
      }

      const double fallback_score = clearance - 0.05 * xy_distance;
      if (!found_fallback || fallback_score > best_fallback_score) {
        best_fallback_score = fallback_score;
        best_fallback_distance = xy_distance;
        best_fallback_node = node.id;
        best_fallback_component = surface_graph_.componentId(node.id);
        found_fallback = true;
      }
    }

    if (found_safe) {
      *nearest_node = best_safe_node;
      *nearest_distance_m = best_distance;
      if (component_id != nullptr) {
        *component_id = best_safe_component;
      }
      return true;
    }
    if (found_fallback) {
      *nearest_node = best_fallback_node;
      *nearest_distance_m = best_fallback_distance;
      if (component_id != nullptr) {
        *component_id = best_fallback_component;
      }
      return true;
    }
    *nearest_distance_m = 0.0;
    return false;
  }

  Point3 surfaceNodePoint(const SurfaceNodeId node_id) const
  {
    const SurfaceNode * node = surface_graph_.node(node_id);
    if (node == nullptr) {
      return {};
    }
    return {
      (static_cast<double>(node->x) + 0.5) * surface_graph_.resolution(),
      (static_cast<double>(node->y) + 0.5) * surface_graph_.resolution(),
      node->z};
  }

  tgw_planner::msg::PlannerStats makePlannerStats(
    const SurfacePlanResult & plan,
    double search_time_ms,
    double start_snap_distance_m,
    double goal_snap_distance_m) const
  {
    tgw_planner::msg::PlannerStats stats;
    stats.stamp = now();
    stats.map_id = "n3map_experience";
    stats.success = plan.success;
    stats.failure_reason = plan.success ? "" : plan.message;
    stats.search_time_ms = search_time_ms;
    stats.total_plan_time_ms = search_time_ms;
    stats.traversable_cells = static_cast<std::uint32_t>(
      std::min<std::size_t>(
        snapshot_.surface.traversable_cells.size(), std::numeric_limits<std::uint32_t>::max()));
    stats.blocked_cells = static_cast<std::uint32_t>(
      std::min<std::size_t>(
        snapshot_.surface.blocked_cells.size(), std::numeric_limits<std::uint32_t>::max()));
    stats.risk_cells = static_cast<std::uint32_t>(
      std::min<std::size_t>(
        snapshot_.risk.risks().size(), std::numeric_limits<std::uint32_t>::max()));
    stats.expanded_nodes = plan.metrics.expanded_nodes;
    stats.generated_nodes = plan.metrics.generated_nodes;
    stats.raw_path_waypoints = plan.metrics.raw_path_waypoints;
    stats.raw_path_length_m = plan.metrics.raw_path_length_m;
    stats.postprocess_floor_shortcuts = plan.metrics.shortcut_count;
    stats.path_waypoints = static_cast<std::uint32_t>(plan.path.size());
    stats.path_length_m = plan.metrics.path_length_m;
    stats.max_path_edge_dz_m = plan.metrics.max_path_edge_dz_m;
    stats.path_layer_jump_edges = plan.metrics.path_layer_jump_edges;
    stats.final_path_validated = plan.metrics.final_path_validated;
    stats.final_path_fallback_to_raw = plan.metrics.final_path_fallback_to_raw;
    stats.final_path_validation_failure = plan.metrics.final_path_validation_failure;
    stats.min_path_clearance_m = plan.metrics.min_path_clearance_m;
    stats.mean_path_clearance_m = plan.metrics.mean_path_clearance_m;
    stats.clearance_cost_sum = plan.metrics.clearance_cost_sum;
    stats.unknown_cost_sum = plan.metrics.unknown_cost_sum;
    stats.risk_cost_sum = plan.metrics.risk_cost_sum;
    stats.max_path_risk = plan.metrics.max_path_risk;
    stats.low_clearance_samples = plan.metrics.low_clearance_samples;
    stats.start_snap_distance_m = start_snap_distance_m;
    stats.goal_snap_distance_m = goal_snap_distance_m;
    stats.start_portal_candidates = plan.metrics.start_portal_candidates;
    stats.goal_portal_candidates = plan.metrics.goal_portal_candidates;
    stats.evaluated_portal_pairs = plan.metrics.evaluated_portal_pairs;
    stats.hybrid_nodes = plan.metrics.hybrid_nodes;
    stats.hybrid_surface_edges = plan.metrics.hybrid_surface_edges;
    stats.hybrid_backbone_edges = plan.metrics.hybrid_backbone_edges;
    stats.hybrid_portal_edges = plan.metrics.hybrid_portal_edges;
    stats.hybrid_expanded_nodes = plan.metrics.hybrid_expanded_nodes;
    stats.used_backbone_edges = plan.metrics.used_backbone_edges;
    stats.used_portal_edges = plan.metrics.used_portal_edges;
    stats.used_surface_edges = plan.metrics.used_surface_edges;
    stats.backbone_path_length_m = plan.metrics.backbone_path_length_m;
    stats.surface_path_length_m = plan.metrics.surface_path_length_m;
    stats.portal_switch_count = plan.metrics.portal_switch_count;
    stats.selected_start_portal_id = plan.metrics.selected_start_portal_id;
    stats.selected_goal_portal_id = plan.metrics.selected_goal_portal_id;
    stats.selected_start_backbone_node = plan.metrics.selected_start_backbone_node;
    stats.selected_goal_backbone_node = plan.metrics.selected_goal_backbone_node;
    stats.selected_backbone_index_delta = plan.metrics.selected_backbone_index_delta;
    stats.selected_backbone_length_m = plan.metrics.selected_backbone_length_m;
    stats.selected_start_surface_leg_m = plan.metrics.selected_start_surface_leg_m;
    stats.selected_goal_surface_leg_m = plan.metrics.selected_goal_surface_leg_m;
    stats.selected_total_hybrid_cost = plan.metrics.selected_total_hybrid_cost;
    stats.map_resolution_m = snapshot_.resolution_m;
    stats.robot_radius_m = 0.5 * get_parameter("robot_width_m").as_double();
    return stats;
  }

  SurfacePlanResult planBetween(
    const Point3 & start,
    const Point3 & goal,
    tgw_planner::msg::PlannerStats * stats_out,
    double * search_time_ms_out)
  {
    SurfacePlanResult plan;
    if (!has_snapshot_) {
      plan.message = "experience snapshot is not available";
      if (stats_out != nullptr) {
        *stats_out = tgw_planner::msg::PlannerStats();
        stats_out->success = false;
        stats_out->failure_reason = plan.message;
      }
      if (search_time_ms_out != nullptr) {
        *search_time_ms_out = 0.0;
      }
      return plan;
    }

    SurfaceNodeId start_node;
    SurfaceNodeId goal_node;
    double start_snap_distance = 0.0;
    double goal_snap_distance = 0.0;
    int start_component_id = -1;
    int goal_component_id = -1;
    if (!nearestSurfaceGraphNode(start, &start_node, &start_snap_distance, &start_component_id)) {
      plan.message = "start_not_on_reachable_surface";
      LOG(WARNING)
        << "PlanPath rejected before search"
        << " reason=" << plan.message
        << " start=(" << start.x << ", " << start.y << ", " << start.z << ")"
        << " no_graph_node_below_click=true";
      if (stats_out != nullptr) {
        *stats_out = makePlannerStats(plan, 0.0, start_snap_distance, goal_snap_distance);
      }
      if (search_time_ms_out != nullptr) {
        *search_time_ms_out = 0.0;
      }
      return plan;
    }
    const double max_start_snap_distance_m =
      get_parameter("max_start_snap_distance_m").as_double();
    if (start_snap_distance > max_start_snap_distance_m) {
      plan.message =
        "start_snap_distance_too_far " + metersText(start_snap_distance) +
        " > " + metersText(max_start_snap_distance_m);
      const Point3 snapped_start = surfaceNodePoint(start_node);
      LOG(WARNING)
        << "PlanPath rejected before search"
        << " reason=" << plan.message
        << " clicked_start=(" << start.x << ", " << start.y << ", " << start.z << ")"
        << " snapped_start=(" <<
        snapped_start.x << ", " << snapped_start.y << ", " << snapped_start.z << ")"
        << " start_node=" << start_node.id
        << " start_snap_xy_m=" << start_snap_distance
        << " max_start_snap_xy_m=" << max_start_snap_distance_m
        << " start_component=" << start_component_id;
      if (stats_out != nullptr) {
        *stats_out = makePlannerStats(plan, 0.0, start_snap_distance, goal_snap_distance);
      }
      if (search_time_ms_out != nullptr) {
        *search_time_ms_out = 0.0;
      }
      return plan;
    }
    if (!nearestSurfaceGraphNode(goal, &goal_node, &goal_snap_distance, &goal_component_id)) {
      plan.message = "goal_unreachable_outside_experience_surface";
      const Point3 snapped_start = surfaceNodePoint(start_node);
      LOG(WARNING)
        << "PlanPath rejected before search"
        << " reason=" << plan.message
        << " clicked_start=(" << start.x << ", " << start.y << ", " << start.z << ")"
        << " snapped_start=(" <<
        snapped_start.x << ", " << snapped_start.y << ", " << snapped_start.z << ")"
        << " start_node=" << start_node.id
        << " goal=(" << goal.x << ", " << goal.y << ", " << goal.z << ")"
        << " no_graph_node_below_click=true"
        << " start_snap_xy_m=" << start_snap_distance
        << " start_component=" << start_component_id;
      if (stats_out != nullptr) {
        *stats_out = makePlannerStats(plan, 0.0, start_snap_distance, goal_snap_distance);
      }
      if (search_time_ms_out != nullptr) {
        *search_time_ms_out = 0.0;
      }
      return plan;
    }
    const double max_goal_snap_distance_m =
      get_parameter("max_goal_snap_distance_m").as_double();
    if (goal_snap_distance > max_goal_snap_distance_m) {
      plan.message =
        "goal_snap_distance_too_far " + metersText(goal_snap_distance) +
        " > " + metersText(max_goal_snap_distance_m);
      const Point3 snapped_start = surfaceNodePoint(start_node);
      const Point3 snapped_goal = surfaceNodePoint(goal_node);
      LOG(WARNING)
        << "PlanPath rejected before search"
        << " reason=" << plan.message
        << " clicked_start=(" << start.x << ", " << start.y << ", " << start.z << ")"
        << " snapped_start=(" <<
        snapped_start.x << ", " << snapped_start.y << ", " << snapped_start.z << ")"
        << " start_node=" << start_node.id
        << " clicked_goal=(" << goal.x << ", " << goal.y << ", " << goal.z << ")"
        << " snapped_goal=(" <<
        snapped_goal.x << ", " << snapped_goal.y << ", " << snapped_goal.z << ")"
        << " goal_node=" << goal_node.id
        << " goal_snap_xy_m=" << goal_snap_distance
        << " max_goal_snap_xy_m=" << max_goal_snap_distance_m
        << " start_snap_xy_m=" << start_snap_distance
        << " start_component=" << start_component_id
        << " goal_component=" << goal_component_id;
      if (stats_out != nullptr) {
        *stats_out = makePlannerStats(plan, 0.0, start_snap_distance, goal_snap_distance);
      }
      if (search_time_ms_out != nullptr) {
        *search_time_ms_out = 0.0;
      }
      return plan;
    }
    const Point3 snapped_start = surfaceNodePoint(start_node);
    const Point3 snapped_goal = surfaceNodePoint(goal_node);
    LOG(INFO)
      << "PlanPath accepted for hybrid graph search"
      << " clicked_start=(" << start.x << ", " << start.y << ", " << start.z << ")"
      << " snapped_start=(" <<
      snapped_start.x << ", " << snapped_start.y << ", " << snapped_start.z << ")"
      << " start_node=" << start_node.id
      << " clicked_goal=(" << goal.x << ", " << goal.y << ", " << goal.z << ")"
      << " snapped_goal=(" <<
      snapped_goal.x << ", " << snapped_goal.y << ", " << snapped_goal.z << ")"
      << " goal_node=" << goal_node.id
      << " start_component=" << start_component_id
      << " goal_component=" << goal_component_id
      << " start_snap_xy_m=" << start_snap_distance
      << " goal_snap_xy_m=" << goal_snap_distance
      << " backbone_nodes=" << (has_backbone_graph_ ? backbone_graph_.nodes().size() : 0U)
      << " backbone_portals=" << (has_backbone_graph_ ? backbone_graph_.portals().size() : 0U);

    const auto t0 = std::chrono::steady_clock::now();
    HybridExperiencePlannerOptions hybrid_options = defaultHybridExperiencePlannerOptions();
    hybrid_options.backbone_cost_scale = get_parameter("hybrid_backbone_cost_scale").as_double();
    hybrid_options.portal_switch_cost = get_parameter("hybrid_portal_switch_cost").as_double();
    hybrid_options.portal_height_error_weight =
      get_parameter("hybrid_portal_height_error_weight").as_double();
    hybrid_options.backbone_low_confidence_penalty =
      get_parameter("hybrid_backbone_low_confidence_penalty").as_double();
    hybrid_options.max_backbone_edge_xy_gap_m =
      get_parameter("hybrid_max_backbone_edge_xy_gap_m").as_double();
    hybrid_options.max_backbone_edge_dz_m =
      get_parameter("hybrid_max_backbone_edge_dz_m").as_double();
    hybrid_options.max_backbone_edge_slope =
      get_parameter("hybrid_max_backbone_edge_slope").as_double();
    hybrid_options.max_portal_xy_distance_m =
      get_parameter("hybrid_max_portal_xy_distance_m").as_double();
    hybrid_options.max_portal_height_error_m =
      get_parameter("hybrid_max_portal_height_error_m").as_double();
    hybrid_options.surface_target_speed_mps =
      get_parameter("hybrid_surface_target_speed_mps").as_double();
    hybrid_options.backbone_target_speed_mps =
      get_parameter("hybrid_backbone_target_speed_mps").as_double();
    hybrid_options.portal_target_speed_mps =
      get_parameter("hybrid_portal_target_speed_mps").as_double();
    plan = HybridExperiencePlanner(plannerOptions(), hybrid_options).plan(
      surface_graph_, backbone_graph_, start_node, goal_node);
    const auto t1 = std::chrono::steady_clock::now();
    const double search_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    if (stats_out != nullptr) {
      *stats_out = makePlannerStats(
        plan, search_time_ms, start_snap_distance, goal_snap_distance);
    }
    if (search_time_ms_out != nullptr) {
      *search_time_ms_out = search_time_ms;
    }
    return plan;
  }

  RouteProgressTrackerOptions routeTrackerOptions() const
  {
    RouteProgressTrackerOptions options;
    options.projection_window_m = get_parameter("tracking_projection_window_m").as_double();
    options.local_route_length_m = get_parameter("tracking_local_route_length_m").as_double();
    options.goal_tolerance_m = get_parameter("tracking_goal_tolerance_m").as_double();
    return options;
  }

  LocalPathSmootherOptions localPathSmootherOptions() const
  {
    LocalPathSmootherOptions options;
    options.min_point_spacing_m = get_parameter("local_path_min_point_spacing_m").as_double();
    options.max_point_spacing_m = get_parameter("local_path_max_point_spacing_m").as_double();
    options.enable_collision_check = get_parameter("local_path_enable_collision_check").as_bool();
    options.bezier_handle_ratio = get_parameter("local_path_bezier_handle_ratio").as_double();
    options.max_smoothness_rad_per_m =
      get_parameter("local_path_max_smoothness_rad_per_m").as_double();
    options.max_turn_angle_rad = get_parameter("local_path_max_turn_angle_rad").as_double();
    options.max_route_deviation_m = get_parameter("local_path_max_route_deviation_m").as_double();
    options.corner_cut_turn_angle_rad =
      get_parameter("local_path_corner_cut_turn_angle_rad").as_double();
    options.min_corner_target_distance_m =
      get_parameter("local_path_min_corner_target_distance_m").as_double();
    return options;
  }

  RollingLocalMapOptions rollingLocalMapOptions() const
  {
    RollingLocalMapOptions options;
    options.robot_radius_m = get_parameter("local_map_robot_radius_m").as_double();
    options.inflation_radius_m = get_parameter("local_map_inflation_radius_m").as_double();
    options.time_window_s = get_parameter("local_map_time_window_s").as_double();
    options.max_obstacle_points = static_cast<std::size_t>(
      std::max<std::int64_t>(1, get_parameter("local_map_max_obstacle_points").as_int()));
    return options;
  }

  RegulatedPurePursuitOptions rppOptions() const
  {
    RegulatedPurePursuitOptions options;
    options.desired_linear_speed_mps =
      get_parameter("tracking_desired_linear_speed_mps").as_double();
    options.max_linear_speed_mps = get_parameter("tracking_max_linear_speed_mps").as_double();
    options.min_linear_speed_mps = get_parameter("tracking_min_linear_speed_mps").as_double();
    options.max_angular_speed_radps = get_parameter("tracking_max_yaw_rate_radps").as_double();
    options.lookahead_m = get_parameter("tracking_lookahead_m").as_double();
    options.max_lateral_accel_mps2 =
      get_parameter("tracking_max_lateral_accel_mps2").as_double();
    options.goal_slowdown_distance_m =
      get_parameter("tracking_goal_slowdown_distance_m").as_double();
    options.collision_time_horizon_s =
      get_parameter("local_collision_time_horizon_s").as_double();
    options.collision_sample_time_s =
      get_parameter("local_collision_sample_time_s").as_double();
    options.max_linear_accel_mps2 =
      get_parameter("tracking_max_linear_accel_mps2").as_double();
    return options;
  }

  void setTrackerPath(const SurfacePlanResult & plan)
  {
    has_tracking_path_ = false;
    if (!plan.success) {
      route_tracker_ = RouteProgressTracker(routeTrackerOptions());
      local_path_smoother_ = LocalPathSmoother(localPathSmootherOptions());
      rolling_local_map_ = RollingLocalMap(rollingLocalMapOptions());
      rpp_controller_ = RegulatedPurePursuitController(rppOptions());
      has_expected_surface_z_ = false;
      local_obstacle_last_accepted_points_ = 0U;
      sim_replay_trace_.clear();
      publishZeroVelocity();
      return;
    }
    std::string error;
    route_tracker_ = RouteProgressTracker(routeTrackerOptions());
    local_path_smoother_ = LocalPathSmoother(localPathSmootherOptions());
    rolling_local_map_ = RollingLocalMap(rollingLocalMapOptions());
    rpp_controller_ = RegulatedPurePursuitController(rppOptions());
    has_expected_surface_z_ = false;
    local_obstacle_last_accepted_points_ = 0U;
    sim_replay_trace_.clear();
    if (!route_tracker_.setPath(plan.global_path, &error)) {
      LOG(WARNING) << "RouteProgressTracker rejected planned path: " << error;
      publishZeroVelocity();
      return;
    }
    route_tracker_.resetProgress();
    rpp_controller_.reset();
    has_tracking_path_ = true;
    if (get_parameter("kinematic_replay_reset_on_plan").as_bool()) {
      initializeKinematicReplay(plan);
    }
    LOG(INFO)
      << "local route tracker loaded path"
      << " points=" << plan.global_path.size()
      << " length_m=" << route_tracker_.pathLength();
  }

  void clearTrackerPath()
  {
    has_tracking_path_ = false;
    route_tracker_ = RouteProgressTracker(routeTrackerOptions());
    local_path_smoother_ = LocalPathSmoother(localPathSmootherOptions());
    rolling_local_map_ = RollingLocalMap(rollingLocalMapOptions());
    rpp_controller_ = RegulatedPurePursuitController(rppOptions());
    has_expected_surface_z_ = false;
    local_obstacle_last_accepted_points_ = 0U;
    sim_replay_trace_.clear();
    publishZeroVelocity();
  }

  void publishZeroVelocity()
  {
    if (!cmd_vel_pub_) {
      return;
    }
    geometry_msgs::msg::Twist twist;
    cmd_vel_pub_->publish(twist);
  }

  bool kinematicReplayEnabled() const
  {
    return get_parameter("enable_kinematic_replay").as_bool();
  }

  void initializeKinematicReplay(const SurfacePlanResult & plan)
  {
    if (!kinematicReplayEnabled() || plan.global_path.empty()) {
      return;
    }
    sim_replay_trace_.clear();
    latest_pose_.x = plan.global_path.front().position.x;
    latest_pose_.y = plan.global_path.front().position.y;
    latest_pose_.yaw_rad = plan.global_path.front().yaw_hint_rad;
    if (!std::isfinite(latest_pose_.yaw_rad) && plan.global_path.size() > 1U) {
      const Point3 & a = plan.global_path.front().position;
      const Point3 & b = plan.global_path[1U].position;
      latest_pose_.yaw_rad = std::atan2(b.y - a.y, b.x - a.x);
    }
    if (!std::isfinite(latest_pose_.yaw_rad)) {
      latest_pose_.yaw_rad = 0.0;
    }
    latest_expected_surface_z_m_ = plan.global_path.front().position.z;
    has_expected_surface_z_ = true;
    latest_odom_time_ = now();
    has_odom_ = true;
    last_tracking_tick_valid_ = false;
    publishKinematicReplayState(0.0, 0.0);
  }

  void integrateKinematicReplay(const RegulatedPurePursuitCommand & command, const double dt_s)
  {
    if (!kinematicReplayEnabled() || !command.valid) {
      return;
    }
    latest_pose_.yaw_rad += command.yaw_rate_radps * dt_s;
    latest_pose_.x += command.linear_speed_mps * dt_s * std::cos(latest_pose_.yaw_rad);
    latest_pose_.y += command.linear_speed_mps * dt_s * std::sin(latest_pose_.yaw_rad);
    latest_odom_time_ = now();
    has_odom_ = true;
    publishKinematicReplayState(command.linear_speed_mps, command.yaw_rate_radps);
  }

  void publishKinematicReplayState(const double linear_speed_mps, const double yaw_rate_radps)
  {
    if (!kinematicReplayEnabled()) {
      return;
    }
    const rclcpp::Time stamp = now();
    const std::string frame_id = markerFrameId();
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id;
    odom.child_frame_id = "tgw_kinematic_replay_base";
    odom.pose.pose.position.x = latest_pose_.x;
    odom.pose.pose.position.y = latest_pose_.y;
    odom.pose.pose.position.z = has_expected_surface_z_ ? latest_expected_surface_z_m_ : 0.0;
    odom.pose.pose.orientation = quaternionFromYaw(latest_pose_.yaw_rad);
    odom.twist.twist.linear.x = linear_speed_mps;
    odom.twist.twist.angular.z = yaw_rate_radps;
    fake_odom_pub_->publish(odom);

    Point3 trace_point{
      odom.pose.pose.position.x,
      odom.pose.pose.position.y,
      odom.pose.pose.position.z};
    if (sim_replay_trace_.empty() || xyDistance(sim_replay_trace_.back(), trace_point) > 0.02) {
      sim_replay_trace_.push_back(trace_point);
      const auto max_points = static_cast<std::size_t>(
        std::max<std::int64_t>(
          2, get_parameter("kinematic_replay_trace_max_points").as_int()));
      if (sim_replay_trace_.size() > max_points) {
        sim_replay_trace_.erase(
          sim_replay_trace_.begin(),
          sim_replay_trace_.begin() +
          static_cast<std::ptrdiff_t>(sim_replay_trace_.size() - max_points));
      }
    }
    if (sim_robot_path_pub_) {
      sim_robot_path_pub_->publish(makePathMsg(stamp, frame_id, sim_replay_trace_));
    }

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = frame_id;
    marker.ns = "tgw_kinematic_replay_robot";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = odom.pose.pose.position;
    marker.pose.position.z += 0.25;
    marker.pose.orientation = odom.pose.pose.orientation;
    marker.scale.x = 0.70;
    marker.scale.y = 0.18;
    marker.scale.z = 0.18;
    marker.color.r = 0.1F;
    marker.color.g = 0.8F;
    marker.color.b = 1.0F;
    marker.color.a = 1.0F;
    sim_robot_marker_pub_->publish(marker);
  }

  std::string trackingStatusJson(
    const RegulatedPurePursuitCommand & command,
    const RouteProgressState & route,
    const LocalPathResult & local_path,
    const std::string & reason,
    const double odom_age_s) const
  {
    std::ostringstream json;
    json << "{";
    json << "\"kind\":\"tracking_status\"";
    json << ",\"enabled\":" <<
      (get_parameter("enable_path_tracking").as_bool() ? "true" : "false");
    json << ",\"has_path\":" << (has_tracking_path_ ? "true" : "false");
    json << ",\"has_odom\":" << (has_odom_ ? "true" : "false");
    json << ",\"valid\":" << (command.valid ? "true" : "false");
    json << ",\"goal_reached\":" << (command.goal_reached ? "true" : "false");
    json << ",\"reason\":\"" << jsonEscape(reason) << "\"";
    json << ",\"status\":\"" << jsonEscape(command.status) << "\"";
    json << ",\"route_status\":\"" << jsonEscape(route.status) << "\"";
    json << ",\"local_path_status\":\"" << jsonEscape(local_path.message) << "\"";
    json << ",\"progress_m\":" << route.progress_m;
    json << ",\"remaining_m\":" << route.remaining_m;
    json << ",\"lateral_error_m\":" << route.lateral_error_m;
    json << ",\"linear_speed_mps\":" << command.linear_speed_mps;
    json << ",\"yaw_rate_radps\":" << command.yaw_rate_radps;
    json << ",\"curvature\":" << command.curvature;
    json << ",\"expected_surface_z_m\":" << route.projected_point.position.z;
    json << ",\"segment_kind\":\"" << pathKindName(route.projected_point.kind) << "\"";
    json << ",\"confidence\":" << route.projected_point.confidence;
    json << ",\"local_path_points\":" << local_path.path.size();
    json << ",\"local_path_length_m\":" << local_path.length_m;
    json << ",\"local_path_smoothness_rad_per_m\":" << local_path.smoothness_rad_per_m;
    json << ",\"local_path_max_turn_angle_rad\":" << local_path.max_turn_angle_rad;
    const auto [trace_smoothness, trace_max_turn] = pointPathSmoothness(sim_replay_trace_);
    json << ",\"sim_trace_points\":" << sim_replay_trace_.size();
    json << ",\"sim_trace_length_m\":" << pointPathLength(sim_replay_trace_);
    json << ",\"sim_trace_smoothness_rad_per_m\":" << trace_smoothness;
    json << ",\"sim_trace_max_turn_angle_rad\":" << trace_max_turn;
    json << ",\"local_obstacle_points\":" << rolling_local_map_.obstacleCount();
    json << ",\"local_obstacle_last_accepted_points\":" << local_obstacle_last_accepted_points_;
    json << ",\"local_obstacle_clouds_received\":" << local_obstacle_clouds_received_;
    json << ",\"local_obstacle_clouds_rejected\":" << local_obstacle_clouds_rejected_;
    json << ",\"odom_age_s\":" << odom_age_s;
    json << "}";
    return json.str();
  }

  void publishTrackingStatus(
    const RegulatedPurePursuitCommand & command,
    const RouteProgressState & route,
    const LocalPathResult & local_path,
    const std::string & reason,
    const double odom_age_s)
  {
    if (!tracking_status_json_pub_) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = trackingStatusJson(command, route, local_path, reason, odom_age_s);
    tracking_status_json_pub_->publish(msg);
    publishTrackingStatusMarker(command, local_path, reason);
  }

  void publishTrackingStatusMarker(
    const RegulatedPurePursuitCommand & command,
    const LocalPathResult & local_path,
    const std::string & reason)
  {
    if (!tracking_status_marker_pub_) {
      return;
    }
    const rclcpp::Time stamp = now();
    const std::string frame_id = markerFrameId();
    Point3 point{
      latest_pose_.x,
      latest_pose_.y,
      has_expected_surface_z_ ? latest_expected_surface_z_m_ : 0.0};
    point.z += 0.85;
    std::ostringstream text;
    float r = 1.0F;
    float g = 0.1F;
    float b = 0.1F;
    if (command.status == "tracking_path_not_set") {
      text << "TRACK IDLE: no path";
      r = 0.65F;
      g = 0.65F;
      b = 0.65F;
    } else if (command.valid && command.goal_reached) {
      text << "TRACK GOAL REACHED";
      r = 0.1F;
      g = 0.9F;
      b = 0.2F;
    } else if (command.valid && command.status == "tracking") {
      text << "TRACK  v=" << metersText(command.linear_speed_mps) << "/s";
      r = 0.1F;
      g = 0.9F;
      b = 0.2F;
    } else {
      const std::string status = !reason.empty() ? reason :
        (!command.status.empty() ? command.status : local_path.message);
      text << "TRACK FAIL: " << status;
    }
    if (local_path.path.size() >= 2U) {
      text << "  local_smooth=" << local_path.smoothness_rad_per_m;
    }
    tracking_status_marker_pub_->publish(makeTextMarker(
      stamp,
      frame_id,
      "tgw_tracking_status",
      0,
      point,
      text.str(),
      0.32,
      r,
      g,
      b));
  }

  void publishTrackerMarkers(
    const RegulatedPurePursuitCommand & command,
    const RouteProgressState & route)
  {
    if (!command.valid || !route.valid) {
      return;
    }
    const rclcpp::Time stamp = now();
    const std::string frame_id = markerFrameId();
    tracker_lookahead_marker_pub_->publish(makeSphereMarker(
      stamp,
      frame_id,
      "tgw_tracker_lookahead",
      0,
      command.lookahead_point,
      0.28,
      0.1F,
      0.7F,
      1.0F));
    tracker_projected_marker_pub_->publish(makeSphereMarker(
      stamp,
      frame_id,
      "tgw_tracker_projected",
      0,
      route.projected_point.position,
      0.22,
      1.0F,
      0.8F,
      0.1F));
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_pose_.x = msg->pose.pose.position.x;
    latest_pose_.y = msg->pose.pose.position.y;
    latest_pose_.yaw_rad = yawFromQuaternion(msg->pose.pose.orientation);
    latest_odom_time_ = now();
    has_odom_ = true;
  }

  void onLocalObstacleCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!get_parameter("enable_local_obstacle_cloud").as_bool()) {
      return;
    }
    ++local_obstacle_clouds_received_;
    if (!has_odom_ || !has_expected_surface_z_) {
      ++local_obstacle_clouds_rejected_;
      return;
    }
    const std::string frame_id = markerFrameId();
    if (get_parameter("local_obstacle_requires_map_frame").as_bool() &&
      !msg->header.frame_id.empty() && msg->header.frame_id != frame_id)
    {
      ++local_obstacle_clouds_rejected_;
      return;
    }

    const double max_range_m = get_parameter("local_obstacle_max_range_m").as_double();
    const double max_range_sq = max_range_m * max_range_m;
    const double min_height =
      get_parameter("local_obstacle_min_height_above_surface_m").as_double();
    const double max_height =
      get_parameter("local_obstacle_max_height_above_surface_m").as_double();
    const auto max_points = static_cast<std::size_t>(
      std::max<std::int64_t>(1, get_parameter("local_obstacle_max_points_per_cloud").as_int()));

    std::vector<Point3> obstacles;
    obstacles.reserve(std::min<std::size_t>(msg->width * msg->height, max_points));
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    for (; iter_x != iter_x.end() && obstacles.size() < max_points; ++iter_x, ++iter_y, ++iter_z) {
      const double x = *iter_x;
      const double y = *iter_y;
      const double z = *iter_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      const double dx = x - latest_pose_.x;
      const double dy = y - latest_pose_.y;
      if (dx * dx + dy * dy > max_range_sq) {
        continue;
      }
      const double height_above_surface = z - latest_expected_surface_z_m_;
      if (height_above_surface < min_height || height_above_surface > max_height) {
        continue;
      }
      obstacles.push_back({x, y, z});
    }

    local_obstacle_last_accepted_points_ = obstacles.size();
    rolling_local_map_.addObstaclePoints(obstacles, now().seconds());
    publishLocalObstacleDebug(frame_id);
  }

  void publishLocalObstacleDebug(const std::string & frame_id)
  {
    if (!local_obstacle_pub_) {
      return;
    }
    std::vector<Point3> points = rolling_local_map_.obstaclePoints();
    const auto max_points = static_cast<std::size_t>(
      std::max<std::int64_t>(1, get_parameter("local_obstacle_debug_max_points").as_int()));
    if (points.size() > max_points) {
      points.erase(points.begin(), points.begin() + static_cast<std::ptrdiff_t>(points.size() - max_points));
    }
    local_obstacle_pub_->publish(makePointCloud(now(), frame_id, points));
  }

  void onTrackingTimer()
  {
    const bool publish_cmd_vel = get_parameter("enable_path_tracking").as_bool();
    const bool replay_enabled = kinematicReplayEnabled();
    if (!publish_cmd_vel && !replay_enabled) {
      return;
    }

    RegulatedPurePursuitCommand command;
    RouteProgressState route;
    LocalPathResult local_path;
    const rclcpp::Time tick = now();
    const double odom_age_s = has_odom_ ? (tick - latest_odom_time_).seconds() :
      std::numeric_limits<double>::infinity();
    if (!has_tracking_path_) {
      command.status = "tracking_path_not_set";
      publishZeroVelocity();
      publishTrackingStatus(command, route, local_path, command.status, odom_age_s);
      return;
    }
    if (!has_odom_) {
      command.status = "tracking_odom_not_received";
      publishZeroVelocity();
      publishTrackingStatus(command, route, local_path, command.status, odom_age_s);
      return;
    }
    const double odom_timeout_s = get_parameter("tracking_odom_timeout_s").as_double();
    if (odom_age_s > odom_timeout_s) {
      command.status = "tracking_odom_stale";
      publishZeroVelocity();
      publishTrackingStatus(command, route, local_path, command.status, odom_age_s);
      return;
    }

    const double dt_s = last_tracking_tick_valid_ ?
      std::max(1.0e-3, (tick - last_tracking_tick_).seconds()) :
      1.0 / std::max(1.0, get_parameter("tracking_frequency_hz").as_double());
    last_tracking_tick_ = tick;
    last_tracking_tick_valid_ = true;

    route = route_tracker_.update(latest_pose_);
    if (!route.valid) {
      command.status = route.status;
      publishZeroVelocity();
      publishTrackingStatus(command, route, local_path, command.status, odom_age_s);
      return;
    }
    latest_expected_surface_z_m_ = route.projected_point.position.z;
    has_expected_surface_z_ = true;
    rolling_local_map_.prune(tick.seconds());
    if (route.goal_reached) {
      command.valid = true;
      command.goal_reached = true;
      command.status = "goal_reached";
      command.remaining_m = route.remaining_m;
      publishZeroVelocity();
      if (replay_enabled) {
        publishKinematicReplayState(0.0, 0.0);
      }
      publishTrackerMarkers(command, route);
      publishTrackingStatus(command, route, local_path, command.status, odom_age_s);
      return;
    }
    local_path = local_path_smoother_.build(latest_pose_, route, rolling_local_map_);
    if (!local_path.success) {
      command.status = local_path.message;
      publishZeroVelocity();
      publishTrackingStatus(command, route, local_path, command.status, odom_age_s);
      return;
    }
    command = rpp_controller_.computeCommand(
      latest_pose_, local_path.path, route.remaining_m, dt_s, rolling_local_map_);
    geometry_msgs::msg::Twist twist;
    if (command.valid) {
      twist.linear.x = command.linear_speed_mps;
      twist.angular.z = command.yaw_rate_radps;
    }
    if (publish_cmd_vel || replay_enabled) {
      cmd_vel_pub_->publish(twist);
    }
    local_path_pub_->publish(makePathMsg(tick, markerFrameId(), local_path.path));
    integrateKinematicReplay(command, dt_s);
    publishTrackerMarkers(command, route);
    publishTrackingStatus(command, route, local_path, command.status, odom_age_s);
  }

  void publishPlanDebug(const SurfacePlanResult & plan)
  {
    const rclcpp::Time stamp = now();
    const std::string frame_id = snapshot_.map_frame.empty() ?
      get_parameter("map_frame").as_string() : snapshot_.map_frame;
    path_pub_->publish(makePathMsg(stamp, frame_id, plan.path));
    hybrid_path_pub_->publish(makePathMsg(stamp, frame_id, plan.path));
    raw_path_pub_->publish(makePathMsg(stamp, frame_id, plan.raw_path));
    start_portal_candidates_pub_->publish(
      makePointCloud(stamp, frame_id, plan.debug_start_portal_candidates));
    goal_portal_candidates_pub_->publish(
      makePointCloud(stamp, frame_id, plan.debug_goal_portal_candidates));
    selected_start_portal_pub_->publish(
      makePointCloud(stamp, frame_id, plan.debug_selected_start_portal));
    selected_goal_portal_pub_->publish(
      makePointCloud(stamp, frame_id, plan.debug_selected_goal_portal));
    selected_backbone_segment_pub_->publish(
      makePointCloud(stamp, frame_id, plan.debug_selected_backbone_segment));
    used_backbone_segment_pub_->publish(
      makePointCloud(stamp, frame_id, plan.debug_selected_backbone_segment));
    std::vector<Point3> used_portals = plan.debug_selected_start_portal;
    used_portals.insert(
      used_portals.end(),
      plan.debug_selected_goal_portal.begin(),
      plan.debug_selected_goal_portal.end());
    used_portals_pub_->publish(makePointCloud(stamp, frame_id, used_portals));
  }

  void publishPlanRouteStatsJson(
    const SurfacePlanResult & plan,
    double search_time_ms,
    double start_snap_distance_m,
    double goal_snap_distance_m)
  {
    std_msgs::msg::String msg;
    msg.data = toPlanRouteStatsJson(
      plan, search_time_ms, start_snap_distance_m, goal_snap_distance_m);
    stats_json_pub_->publish(msg);
  }

  std::string markerFrameId() const
  {
    if (has_snapshot_ && !snapshot_.map_frame.empty()) {
      return snapshot_.map_frame;
    }
    return get_parameter("map_frame").as_string();
  }

  Point3 snappedMarkerPoint(const Point3 & point, double * snap_distance_m) const
  {
    SurfaceNodeId snapped_node;
    double distance = 0.0;
    if (nearestSurfaceGraphNode(point, &snapped_node, &distance)) {
      if (snap_distance_m != nullptr) {
        *snap_distance_m = distance;
      }
      const SurfaceNode * node = surface_graph_.node(snapped_node);
      if (node != nullptr) {
        return {
          (static_cast<double>(node->x) + 0.5) * surface_graph_.resolution(),
          (static_cast<double>(node->y) + 0.5) * surface_graph_.resolution(),
          node->z};
      }
    }
    if (snap_distance_m != nullptr) {
      *snap_distance_m = -1.0;
    }
    return point;
  }

  void publishEndpointMarker(
    const Point3 & point,
    const bool is_start)
  {
    const rclcpp::Time stamp = now();
    const std::string frame_id = markerFrameId();
    double snap_distance_m = 0.0;
    const Point3 marker_point = snappedMarkerPoint(point, &snap_distance_m);
    const bool snapped = snap_distance_m >= 0.0;
    const std::string label = is_start ? "START" : "GOAL";
    const std::string text = snapped ?
      (label + " snap " + std::to_string(snap_distance_m).substr(0, 4) + "m") :
      (label + " unsnapped");
    auto & pub = is_start ? start_marker_pub_ : goal_marker_pub_;
    if (is_start) {
      pub->publish(makeSphereMarker(
        stamp, frame_id, "tgw_start", 0, marker_point, 0.45, 0.0F, 1.0F, 0.15F));
      pub->publish(makeTextMarker(
        stamp, frame_id, "tgw_start", 1, marker_point, text, 0.35, 0.0F, 1.0F, 0.15F));
    } else {
      pub->publish(makeSphereMarker(
        stamp, frame_id, "tgw_goal", 0, marker_point, 0.45, 1.0F, 0.1F, 0.1F));
      pub->publish(makeTextMarker(
        stamp, frame_id, "tgw_goal", 1, marker_point, text, 0.35, 1.0F, 0.1F, 0.1F));
    }
  }

  void publishPlanStatusMarker(const SurfacePlanResult & plan)
  {
    if (!has_clicked_start_ || !has_clicked_goal_) {
      return;
    }
    const rclcpp::Time stamp = now();
    const std::string frame_id = markerFrameId();
    Point3 point{
      0.5 * (clicked_start_.x + clicked_goal_.x),
      0.5 * (clicked_start_.y + clicked_goal_.y),
      0.5 * (clicked_start_.z + clicked_goal_.z) + 0.8};
    if (!plan.path.empty()) {
      point = plan.path[plan.path.size() / 2U];
      point.z += 0.8;
    }
    const std::string text = plan.success ?
      ("PLAN OK " + std::to_string(plan.path.size()) + " pts") :
      ("PLAN FAIL: " + plan.message);
    plan_status_marker_pub_->publish(makeTextMarker(
      stamp,
      frame_id,
      "tgw_plan_status",
      0,
      point,
      text,
      0.35,
      plan.success ? 0.2F : 1.0F,
      plan.success ? 1.0F : 0.2F,
      plan.success ? 0.2F : 0.2F));
  }

  visualization_msgs::msg::Marker makeDeleteAllMarker(const std::string & frame_id) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = frame_id;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    return marker;
  }

  void handlePlanPath(
    const std::shared_ptr<tgw_planner::srv::PlanPath::Request> request,
    std::shared_ptr<tgw_planner::srv::PlanPath::Response> response)
  {
    tgw_planner::msg::PlannerStats stats;
    double search_time_ms = 0.0;
    const SurfacePlanResult plan = planBetween(
      posePoint(request->start), posePoint(request->goal), &stats, &search_time_ms);
    response->success = plan.success;
    response->message = plan.message;
    response->stats = stats;
    const std::string frame_id = snapshot_.map_frame.empty() ?
      get_parameter("map_frame").as_string() : snapshot_.map_frame;
    response->path = makePathMsg(now(), frame_id, plan.path);
    publishPlanDebug(plan);
    publishPlanRouteStatsJson(
      plan, search_time_ms, stats.start_snap_distance_m, stats.goal_snap_distance_m);
    setTrackerPath(plan);
    LOG(INFO)
      << "PlanPath"
      << " success=" << (plan.success ? "true" : "false")
      << " waypoints=" << plan.path.size()
      << " raw_waypoints=" << plan.raw_path.size()
      << " expanded=" << plan.metrics.expanded_nodes
      << " generated=" << plan.metrics.generated_nodes
      << " search_ms=" << search_time_ms
      << " message=" << plan.message;
  }

  void handleClearPlan(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    has_clicked_start_ = false;
    has_clicked_goal_ = false;
    clicked_start_ = {};
    clicked_goal_ = {};

    const rclcpp::Time stamp = now();
    const std::string frame_id = markerFrameId();
    path_pub_->publish(makePathMsg(stamp, frame_id, {}));
    hybrid_path_pub_->publish(makePathMsg(stamp, frame_id, {}));
    local_path_pub_->publish(makePathMsg(stamp, frame_id, {}));
    sim_robot_path_pub_->publish(makePathMsg(stamp, frame_id, {}));
    raw_path_pub_->publish(makePathMsg(stamp, frame_id, {}));
    start_portal_candidates_pub_->publish(makePointCloud(stamp, frame_id, {}));
    goal_portal_candidates_pub_->publish(makePointCloud(stamp, frame_id, {}));
    selected_start_portal_pub_->publish(makePointCloud(stamp, frame_id, {}));
    selected_goal_portal_pub_->publish(makePointCloud(stamp, frame_id, {}));
    selected_backbone_segment_pub_->publish(makePointCloud(stamp, frame_id, {}));
    used_backbone_segment_pub_->publish(makePointCloud(stamp, frame_id, {}));
    used_portals_pub_->publish(makePointCloud(stamp, frame_id, {}));
    start_marker_pub_->publish(makeDeleteAllMarker(frame_id));
    goal_marker_pub_->publish(makeDeleteAllMarker(frame_id));
    plan_status_marker_pub_->publish(makeDeleteAllMarker(frame_id));
    tracker_lookahead_marker_pub_->publish(makeDeleteAllMarker(frame_id));
    tracker_projected_marker_pub_->publish(makeDeleteAllMarker(frame_id));
    tracking_status_marker_pub_->publish(makeDeleteAllMarker(frame_id));
    sim_robot_marker_pub_->publish(makeDeleteAllMarker(frame_id));
    sim_replay_trace_.clear();
    clearTrackerPath();

    response->success = true;
    response->message = "cleared TGW start, goal, planned path, and tracking path";
    LOG(INFO) << response->message;
  }

  void onStartPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    clicked_start_ = stampedPoint(*msg);
    has_clicked_start_ = true;
    publishEndpointMarker(clicked_start_, true);
    maybePlanClickedPoints();
  }

  void onGoalPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    clicked_goal_ = stampedPoint(*msg);
    has_clicked_goal_ = true;
    publishEndpointMarker(clicked_goal_, false);
    maybePlanClickedPoints();
  }

  void onStartPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    clicked_start_ = posePoint(*msg);
    has_clicked_start_ = true;
    publishEndpointMarker(clicked_start_, true);
    maybePlanClickedPoints();
  }

  void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    clicked_goal_ = posePoint(*msg);
    has_clicked_goal_ = true;
    publishEndpointMarker(clicked_goal_, false);
    maybePlanClickedPoints();
  }

  void maybePlanClickedPoints()
  {
    if (!has_clicked_start_ || !has_clicked_goal_) {
      return;
    }
    tgw_planner::msg::PlannerStats stats;
    double search_time_ms = 0.0;
    const SurfacePlanResult plan = planBetween(
      clicked_start_, clicked_goal_, &stats, &search_time_ms);
    publishPlanDebug(plan);
    publishPlanRouteStatsJson(
      plan, search_time_ms, stats.start_snap_distance_m, stats.goal_snap_distance_m);
    publishPlanStatusMarker(plan);
    setTrackerPath(plan);
    LOG(INFO)
      << "clicked PlanPath"
      << " success=" << (plan.success ? "true" : "false")
      << " waypoints=" << plan.path.size()
      << " raw_waypoints=" << plan.raw_path.size()
      << " expanded=" << plan.metrics.expanded_nodes
      << " generated=" << plan.metrics.generated_nodes
      << " search_ms=" << search_time_ms
      << " message=" << plan.message;
  }

  void publishTrajectory(const N3NavResource & resource)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = now();
    cloud.header.frame_id = resource.map_frame.empty() ?
      get_parameter("map_frame").as_string() : resource.map_frame;

    const auto max_points = static_cast<std::size_t>(
      std::max<std::int64_t>(1, get_parameter("max_trajectory_points").as_int()));
    const std::size_t stride = resource.dense_trajectory.size() > max_points ?
      ((resource.dense_trajectory.size() + max_points - 1U) / max_points) : 1U;
    const std::size_t output_points =
      (resource.dense_trajectory.size() + stride - 1U) / stride;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(output_points);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    for (std::size_t i = 0; i < resource.dense_trajectory.size(); i += stride) {
      const auto & point = resource.dense_trajectory[i].pose_world_lidar.translation;
      *iter_x = static_cast<float>(point.x);
      *iter_y = static_cast<float>(point.y);
      *iter_z = static_cast<float>(point.z);
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }

    trajectory_pub_->publish(cloud);
  }

  void publishProjectionDebug(
    const N3NavResource & resource,
    const TrajectoryProjectionResult & projection)
  {
    const rclcpp::Time stamp = now();
    const std::string frame_id = resource.map_frame.empty() ?
      get_parameter("map_frame").as_string() : resource.map_frame;

    std::vector<Point3> projected_support_points;
    projected_support_points.reserve(projection.projected_support_samples.size());
    for (const ProjectedSupportSample & sample : projection.projected_support_samples) {
      projected_support_points.push_back(sample.support_position);
    }
    projected_support_pub_->publish(
      makePointCloud(stamp, frame_id, projected_support_points));

    std::vector<Point3> proven_reachable_points;
    proven_reachable_points.reserve(projection.observed_seed_cells.size());
    const double resolution_m = projectorOptions().resolution_m;
    for (const auto & cell : projection.observed_seed_cells) {
      proven_reachable_points.push_back({
        (static_cast<double>(cell.x) + 0.5) * resolution_m,
        (static_cast<double>(cell.y) + 0.5) * resolution_m,
        (static_cast<double>(cell.z) + 0.5) * resolution_m});
    }
    proven_reachable_pub_->publish(
      makePointCloud(stamp, frame_id, proven_reachable_points));

    std::vector<Point3> trajectory_bridge_points;
    trajectory_bridge_points.reserve(projection.bridge_seed_cells.size());
    for (const auto & cell : projection.bridge_seed_cells) {
      trajectory_bridge_points.push_back({
        (static_cast<double>(cell.x) + 0.5) * resolution_m,
        (static_cast<double>(cell.y) + 0.5) * resolution_m,
        (static_cast<double>(cell.z) + 0.5) * resolution_m});
    }
    trajectory_bridge_pub_->publish(
      makePointCloud(stamp, frame_id, trajectory_bridge_points));

    std::vector<Point3> rejected_points;
    std::vector<Point3> rejected_no_support_points;
    std::vector<Point3> rejected_ambiguous_multifloor_points;
    rejected_points.reserve(projection.rejected_samples.size());
    for (const RejectedProjectionSample & sample : projection.rejected_samples) {
      rejected_points.push_back(sample.trajectory_position);
      if (sample.reason == "support_projection_failed") {
        rejected_no_support_points.push_back(sample.trajectory_position);
      } else if (sample.reason == "support_projection_ambiguous_multifloor") {
        rejected_ambiguous_multifloor_points.push_back(sample.trajectory_position);
      }
    }
    rejected_projection_pub_->publish(
      makePointCloud(stamp, frame_id, rejected_points));
    rejected_no_support_pub_->publish(
      makePointCloud(stamp, frame_id, rejected_no_support_points));
    rejected_ambiguous_multifloor_pub_->publish(
      makePointCloud(stamp, frame_id, rejected_ambiguous_multifloor_points));
  }

  void publishExpansionDebug(
    const N3NavResource & resource,
    const ExperienceBuildResult & build)
  {
    if (!build.success) {
      return;
    }
    const rclcpp::Time stamp = now();
    const std::string frame_id = resource.map_frame.empty() ?
      get_parameter("map_frame").as_string() : resource.map_frame;
    std::vector<Point3> expanded_points;
    expanded_points.reserve(build.snapshot.surface.traversable_cells.size());
    for (const GridIndex & cell : build.snapshot.surface.traversable_cells) {
      expanded_points.push_back(cellCenter(cell, build.snapshot.resolution_m));
    }
    expanded_reachable_pub_->publish(makePointCloud(stamp, frame_id, expanded_points));

    std::vector<Point3> support_candidate_points;
    support_candidate_points.reserve(build.snapshot.surface.surface_cells.size());
    for (const auto & entry : build.snapshot.surface.surface_cells) {
      if (entry.second.label == tgw_planner::core::SurfaceLabel::TrajectoryBridge) {
        continue;
      }
      support_candidate_points.push_back(cellCenter(entry.first, build.snapshot.resolution_m));
    }
    support_candidate_pub_->publish(makePointCloud(stamp, frame_id, support_candidate_points));

    std::vector<Point3> boundary_points;
    boundary_points.reserve(build.snapshot.surface.boundary_cells.size());
    for (const GridIndex & cell : build.snapshot.surface.boundary_cells) {
      boundary_points.push_back(cellCenter(cell, build.snapshot.resolution_m));
    }
    boundary_pub_->publish(makePointCloud(stamp, frame_id, boundary_points));

    std::vector<IntensityPoint> clearance_points;
    clearance_points.reserve(build.snapshot.clearance.distances().size());
    for (const auto & entry : build.snapshot.clearance.distances()) {
      clearance_points.push_back(
        {cellCenter(entry.first, build.snapshot.resolution_m), entry.second});
    }
    clearance_pub_->publish(makeIntensityCloud(stamp, frame_id, clearance_points));

    std::vector<IntensityPoint> risk_points;
    risk_points.reserve(build.snapshot.risk.risks().size());
    for (const auto & entry : build.snapshot.risk.risks()) {
      risk_points.push_back(
        {cellCenter(entry.first, build.snapshot.resolution_m), entry.second});
    }
    risk_pub_->publish(makeIntensityCloud(stamp, frame_id, risk_points));
  }

  void publishPlannerConnectivityDebug(const N3NavResource & resource)
  {
    if (!has_snapshot_ || !has_surface_graph_) {
      return;
    }
    const rclcpp::Time stamp = now();
    const std::string frame_id = resource.map_frame.empty() ?
      get_parameter("map_frame").as_string() : resource.map_frame;
    std::vector<IntensityPoint> component_points;
    component_points.reserve(surface_graph_.nodes().size());
    for (const SurfaceNode & node : surface_graph_.nodes()) {
      component_points.push_back({
        {
          (static_cast<double>(node.x) + 0.5) * surface_graph_.resolution(),
          (static_cast<double>(node.y) + 0.5) * surface_graph_.resolution(),
          node.z},
        static_cast<double>(surface_graph_.componentId(node.id))});
    }
    planner_component_pub_->publish(makeIntensityCloud(stamp, frame_id, component_points));
  }

  void publishBackboneDebug(const N3NavResource & resource)
  {
    if (!has_backbone_graph_) {
      return;
    }
    const rclcpp::Time stamp = now();
    const std::string frame_id = resource.map_frame.empty() ?
      get_parameter("map_frame").as_string() : resource.map_frame;

    std::vector<IntensityPoint> backbone_points;
    backbone_points.reserve(backbone_graph_.nodes().size());
    for (const auto & node : backbone_graph_.nodes()) {
      backbone_points.push_back({
        node.path_position,
        node.has_surface_portal ? static_cast<double>(node.nearest_surface_component_id) : -1.0});
    }
    backbone_pub_->publish(makeIntensityCloud(stamp, frame_id, backbone_points));

    std::vector<IntensityPoint> portal_points;
    portal_points.reserve(backbone_graph_.portals().size());
    for (const auto & portal : backbone_graph_.portals()) {
      const auto * surface_node = surface_graph_.node(portal.surface_node);
      if (surface_node == nullptr) {
        continue;
      }
      portal_points.push_back({
        {
          (static_cast<double>(surface_node->x) + 0.5) * surface_graph_.resolution(),
          (static_cast<double>(surface_node->y) + 0.5) * surface_graph_.resolution(),
          surface_node->z + 0.10},
        static_cast<double>(portal.surface_component_id)});
    }
    portal_pub_->publish(makeIntensityCloud(stamp, frame_id, portal_points));
  }

  N3MapReader reader_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_geometry_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr support_candidate_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr projected_support_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr trajectory_bridge_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr proven_reachable_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr expanded_reachable_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rejected_projection_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rejected_no_support_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rejected_ambiguous_multifloor_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr boundary_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clearance_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr risk_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr planner_component_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr backbone_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr portal_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr start_portal_candidates_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr goal_portal_candidates_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr selected_start_portal_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr selected_goal_portal_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr selected_backbone_segment_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr hybrid_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr used_backbone_segment_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr used_portals_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr start_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr plan_status_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tracker_lookahead_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tracker_projected_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tracking_status_marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_obstacle_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_json_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tracking_status_json_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fake_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr sim_robot_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr sim_robot_marker_pub_;
  rclcpp::Service<tgw_planner::srv::PlanPath>::SharedPtr plan_path_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_plan_srv_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr local_obstacle_sub_;
  rclcpp::TimerBase::SharedPtr tracking_timer_;
  RouteProgressTracker route_tracker_;
  LocalPathSmoother local_path_smoother_;
  RollingLocalMap rolling_local_map_;
  RegulatedPurePursuitController rpp_controller_;
  RoutePose2D latest_pose_;
  rclcpp::Time latest_odom_time_;
  rclcpp::Time last_tracking_tick_;
  double latest_expected_surface_z_m_{0.0};
  tgw_planner::core::ExperienceSnapshot snapshot_;
  ExperienceSurfaceGraph surface_graph_;
  ExperienceBackboneGraph backbone_graph_;
  bool has_snapshot_{false};
  bool has_surface_graph_{false};
  bool has_backbone_graph_{false};
  bool has_tracking_path_{false};
  bool has_odom_{false};
  bool has_expected_surface_z_{false};
  bool last_tracking_tick_valid_{false};
  std::size_t local_obstacle_last_accepted_points_{0U};
  std::size_t local_obstacle_clouds_received_{0U};
  std::size_t local_obstacle_clouds_rejected_{0U};
  std::vector<Point3> sim_replay_trace_;
  Point3 clicked_start_;
  Point3 clicked_goal_;
  bool has_clicked_start_{false};
  bool has_clicked_goal_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::InitOptions init_options;
  init_options.auto_initialize_logging(false);
  rclcpp::init(argc, argv, init_options);
  google::InitGoogleLogging(argv[0]);
  rclcpp::spin(std::make_shared<TgwExperiencePlannerNode>());
  rclcpp::shutdown();
  google::ShutdownGoogleLogging();
  return 0;
}
