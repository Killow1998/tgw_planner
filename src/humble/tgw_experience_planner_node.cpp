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
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "tgw_planner/core/experience_surface_builder.hpp"
#include "tgw_planner/core/experience_surface_graph.hpp"
#include "tgw_planner/core/grid_index.hpp"
#include "tgw_planner/core/experience_backbone_graph.hpp"
#include "tgw_planner/core/hybrid_experience_planner.hpp"
#include "tgw_planner/core/n3map_reader.hpp"
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
using tgw_planner::core::SurfaceAstarPlanner;
using tgw_planner::core::SurfacePlanResult;
using tgw_planner::core::SurfacePlannerOptions;
using tgw_planner::core::SurfaceTransitionValidator;
using tgw_planner::core::HybridExperiencePlanner;
using tgw_planner::core::HybridExperiencePlannerOptions;
using tgw_planner::core::TrajectoryProjectionResult;
using tgw_planner::core::TrajectoryProjector;
using tgw_planner::core::TrajectoryProjectorOptions;

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
    declare_parameter<std::string>("pbstream_path", "");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<double>("nav_resolution_m", 0.10);
    declare_parameter<double>("raw_resolution_m", 0.05);
    declare_parameter<double>("lidar_to_footprint_x_m", 0.0);
    declare_parameter<double>("lidar_to_footprint_y_m", 0.0);
    declare_parameter<double>("support_search_below_min_m", 0.10);
    declare_parameter<double>("support_search_below_max_m", 1.00);
    declare_parameter<double>("support_max_jump_m", 0.35);
    declare_parameter<bool>("allow_support_reanchor_on_jump", true);
    declare_parameter<int>("support_xy_search_radius_cells", 2);
    declare_parameter<int>("support_xy_retry_radius_cells", 8);
    declare_parameter<double>("max_trajectory_bridge_gap_m", 2.00);
    declare_parameter<double>("max_trajectory_bridge_height_delta_m", 0.80);
    declare_parameter<double>("trajectory_bridge_sample_step_m", 0.10);
    declare_parameter<double>("robot_length_m", 0.70);
    declare_parameter<double>("robot_width_m", 0.43);
    declare_parameter<double>("body_clearance_height_m", 0.65);
    declare_parameter<double>("base_to_front_m", 0.20);
    declare_parameter<double>("min_footprint_support_ratio", 0.50);
    declare_parameter<double>("footprint_support_height_tolerance_m", 0.20);
    declare_parameter<int>("expansion_radius_cells", 2);
    declare_parameter<int>("max_expansion_steps", 12);
    declare_parameter<int>("vertical_tolerance_cells", 3);
    declare_parameter<double>("max_expansion_step_height_m", 0.28);
    declare_parameter<int>("experience_anchor_radius_cells", 24);
    declare_parameter<double>("experience_anchor_height_tolerance_m", 0.35);
    declare_parameter<int>("experience_anchor_vertical_tolerance_cells", 3);
    declare_parameter<bool>("enable_hole_filling", true);
    declare_parameter<int>("hole_fill_iterations", 2);
    declare_parameter<int>("min_hole_fill_neighbors", 5);
    declare_parameter<double>("max_hole_fill_height_spread_m", 0.12);
    declare_parameter<double>("plan_max_step_height_m", 0.35);
    declare_parameter<int>("plan_max_iterations", 250000);
    declare_parameter<double>("plan_bridge_cost", 2.5);
    declare_parameter<double>("max_start_snap_distance_m", 1.50);
    declare_parameter<double>("max_goal_snap_distance_m", 1.50);
    declare_parameter<double>("graph_max_normal_edge_slope", 3.0);
    declare_parameter<double>("graph_max_bridge_edge_slope", 8.0);
    declare_parameter<double>("graph_bridge_attach_max_dz_m", 0.35);
    declare_parameter<double>("planner_footprint_min_support_ratio", 0.80);
    declare_parameter<double>("backbone_min_node_spacing_m", 0.20);
    declare_parameter<double>("backbone_max_portal_xy_distance_m", 1.20);
    declare_parameter<double>("backbone_max_portal_height_error_m", 0.45);
    declare_parameter<double>("backbone_min_portal_clearance_m", 0.0);
    declare_parameter<double>("hybrid_backbone_cost_scale", 1.2);
    declare_parameter<double>("hybrid_portal_switch_cost", 0.5);
    declare_parameter<double>("hybrid_portal_height_error_weight", 0.25);
    declare_parameter<double>("hybrid_backbone_low_confidence_penalty", 0.5);
    declare_parameter<double>("planner_multifloor_z_range_m", 1.50);
    declare_parameter<int>("max_trajectory_points", 200000);
    declare_parameter<int>("max_geometry_debug_points", 400000);
    declare_parameter<std::string>("planner_log_dir", "logs");
    declare_parameter<int>("planner_log_retention_days", 7);

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
    stats_json_pub_ = create_publisher<std_msgs::msg::String>(
      "/tgw_experience/stats_json", latched_qos);
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
      SurfaceGraphBuildOptions graph_options;
      graph_options.max_edge_height_delta_m = planner_options.max_step_height_m;
      graph_options.max_bridge_edge_height_delta_m =
        get_parameter("max_trajectory_bridge_height_delta_m").as_double();
      graph_options.max_bridge_attach_height_delta_m =
        get_parameter("graph_bridge_attach_max_dz_m").as_double();
      graph_options.max_edge_slope = get_parameter("graph_max_normal_edge_slope").as_double();
      graph_options.max_bridge_edge_slope =
        get_parameter("graph_max_bridge_edge_slope").as_double();
      surface_graph_.build(snapshot_, SurfaceTransitionValidator(planner_options), graph_options);
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
    ExperienceBackboneOptions options;
    options.min_node_spacing_m = get_parameter("backbone_min_node_spacing_m").as_double();
    options.max_portal_xy_distance_m =
      get_parameter("backbone_max_portal_xy_distance_m").as_double();
    options.max_portal_height_error_m =
      get_parameter("backbone_max_portal_height_error_m").as_double();
    options.min_portal_clearance_m =
      get_parameter("backbone_min_portal_clearance_m").as_double();
    return options;
  }

  TrajectoryProjectorOptions projectorOptions() const
  {
    TrajectoryProjectorOptions options;
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
    ExperienceSurfaceBuilderOptions options;
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
    SurfacePlannerOptions options;
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
      plan.message = "start_not_on_reachable_surface";
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
      plan.message = "goal_unreachable_outside_experience_surface";
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
    HybridExperiencePlannerOptions hybrid_options;
    hybrid_options.backbone_cost_scale = get_parameter("hybrid_backbone_cost_scale").as_double();
    hybrid_options.portal_switch_cost = get_parameter("hybrid_portal_switch_cost").as_double();
    hybrid_options.portal_height_error_weight =
      get_parameter("hybrid_portal_height_error_weight").as_double();
    hybrid_options.backbone_low_confidence_penalty =
      get_parameter("hybrid_backbone_low_confidence_penalty").as_double();
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

    response->success = true;
    response->message = "cleared TGW start, goal, and planned path";
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
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr used_backbone_segment_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr used_portals_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr start_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr plan_status_marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_json_pub_;
  rclcpp::Service<tgw_planner::srv::PlanPath>::SharedPtr plan_path_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_plan_srv_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  tgw_planner::core::ExperienceSnapshot snapshot_;
  ExperienceSurfaceGraph surface_graph_;
  ExperienceBackboneGraph backbone_graph_;
  bool has_snapshot_{false};
  bool has_surface_graph_{false};
  bool has_backbone_graph_{false};
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
