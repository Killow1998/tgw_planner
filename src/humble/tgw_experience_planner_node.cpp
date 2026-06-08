#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

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
#include "tgw_planner/core/grid_index.hpp"
#include "tgw_planner/core/n3map_reader.hpp"
#include "tgw_planner/core/surface_astar_planner.hpp"
#include "tgw_planner/core/trajectory_projector.hpp"
#include "tgw_planner/msg/planner_stats.hpp"
#include "tgw_planner/srv/plan_path.hpp"

namespace
{
using tgw_planner::core::N3MapReadResult;
using tgw_planner::core::N3MapReader;
using tgw_planner::core::N3NavResource;
using tgw_planner::core::Point3;
using tgw_planner::core::ProjectedSupportSample;
using tgw_planner::core::RejectedProjectionSample;
using tgw_planner::core::ExperienceBuildResult;
using tgw_planner::core::ExperienceSurfaceBuilder;
using tgw_planner::core::ExperienceSurfaceBuilderOptions;
using tgw_planner::core::GridIndex;
using tgw_planner::core::SurfaceAstarPlanner;
using tgw_planner::core::SurfacePlanResult;
using tgw_planner::core::SurfacePlannerOptions;
using tgw_planner::core::TrajectoryProjectionResult;
using tgw_planner::core::TrajectoryProjector;
using tgw_planner::core::TrajectoryProjectorOptions;

struct IntensityPoint
{
  Point3 point;
  double intensity{0.0};
};

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
  const ExperienceBuildResult * build)
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
    declare_parameter<int>("max_trajectory_points", 200000);
    declare_parameter<int>("max_geometry_debug_points", 400000);

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
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/tgw_experience/path", latched_qos);
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
      RCLCPP_ERROR(get_logger(), "%s", result.message.c_str());
      return;
    }

    const N3MapReadResult result = reader_.readPbstream(pbstream_path);
    if (!result.success) {
      publishStats(result, pbstream_path, nullptr, nullptr);
      RCLCPP_ERROR(
        get_logger(), "pbstream load failed: [%s] %s",
        result.error_code.c_str(), result.message.c_str());
      return;
    }

    const TrajectoryProjectionResult projection = TrajectoryProjector(projectorOptions()).project(
      result.resource);
    const ExperienceBuildResult build = ExperienceSurfaceBuilder(builderOptions()).build(
      result.resource);
    if (build.success) {
      snapshot_ = build.snapshot;
      has_snapshot_ = true;
    }
    publishKeyframeGeometry(result.resource);
    publishTrajectory(result.resource);
    publishProjectionDebug(result.resource, projection);
    publishExpansionDebug(result.resource, build);
    publishStats(result, pbstream_path, &projection, &build);
    std::unordered_map<std::string, std::size_t> rejected_counts;
    for (const RejectedProjectionSample & sample : projection.rejected_samples) {
      ++rejected_counts[sample.reason];
    }
    RCLCPP_INFO(
      get_logger(),
      "loaded n3map experience resource: keyframes=%zu dense_trajectory=%zu raw_geometry=%zu support_candidates=%zu projected_support=%zu observed_seed=%zu bridge_seed=%zu expanded_reachable=%zu rejected_projection=%zu no_support=%zu ambiguous_multifloor=%zu reanchored_support=%zu retry_support=%zu bridge_used_as_expansion_anchor=%zu support_components=%zu anchored_components=%zu rejected_unanchored_component=%zu footprint_rejected=%zu body_obstructed_rejected=%zu anchor_envelope_rejected=%zu hole_filled=%zu frame=%s body=%s",
      result.resource.keyframes.size(),
      result.resource.dense_trajectory.size(),
      build.success ? build.raw_geometry_cell_count : 0U,
      build.success ? build.support_candidate_count : 0U,
      projection.projected_support_samples.size(),
      projection.observed_seed_cells.size(),
      projection.bridge_seed_cells.size(),
      build.success ? build.snapshot.surface.traversable_cells.size() : 0U,
      projection.rejected_samples.size(),
      rejected_counts["support_projection_failed"],
      rejected_counts["support_projection_ambiguous_multifloor"],
      projection.reanchored_support_samples,
      projection.retry_support_samples,
      build.success ? build.bridge_used_as_expansion_anchor : 0U,
      build.success ? build.support_component_count : 0U,
      build.success ? build.anchored_support_component_count : 0U,
      build.success ? build.rejected_unanchored_component_cells : 0U,
      projection.footprint_rejected_samples,
      build.success ? build.body_obstructed_rejected_count : 0U,
      build.success ? build.anchor_envelope_rejected_count : 0U,
      build.success ? build.hole_filled_count : 0U,
      result.resource.map_frame.c_str(),
      result.resource.body_frame.c_str());
    if (!build.success) {
      RCLCPP_WARN(
        get_logger(), "experience expansion failed: [%s] %s",
        build.error_code.c_str(), build.message.c_str());
    }
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
    msg.data = toStatsJson(result, pbstream_path, projection, build);
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
    options.require_footprint_support = true;
    options.enable_shortcut = true;
    return options;
  }

  bool nearestTraversableCell(
    const Point3 & point, GridIndex * nearest_cell, double * nearest_distance_m) const
  {
    if (!has_snapshot_) {
      return false;
    }
    const double layer_tolerance_m = 0.75 * snapshot_.resolution_m;
    const double z_ceiling = point.z + 0.5 * snapshot_.resolution_m;
    double nearest_xy_distance = std::numeric_limits<double>::infinity();
    double target_layer_z = -std::numeric_limits<double>::infinity();
    bool found_layer = false;

    for (const GridIndex & cell : snapshot_.surface.traversable_cells) {
      const Point3 center = cellCenter(cell, snapshot_.resolution_m);
      const double dx = point.x - center.x;
      const double dy = point.y - center.y;
      const double xy_distance = std::sqrt(dx * dx + dy * dy);
      if (center.z > z_ceiling) {
        continue;
      }
      if (snapshot_.surface.blocked_cells.find(cell) != snapshot_.surface.blocked_cells.end() ||
        snapshot_.surface.forbidden_cells.find(cell) != snapshot_.surface.forbidden_cells.end())
      {
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
    GridIndex best_safe_cell;
    GridIndex best_fallback_cell;
    double best_distance = 0.0;
    double best_fallback_distance = 0.0;
    bool found_safe = false;
    bool found_fallback = false;

    for (const GridIndex & cell : snapshot_.surface.traversable_cells) {
      const Point3 center = cellCenter(cell, snapshot_.resolution_m);
      if (std::abs(center.z - target_layer_z) > layer_tolerance_m) {
        continue;
      }
      const double dx = point.x - center.x;
      const double dy = point.y - center.y;
      const double xy_distance = std::sqrt(dx * dx + dy * dy);
      if (snapshot_.surface.blocked_cells.find(cell) != snapshot_.surface.blocked_cells.end() ||
        snapshot_.surface.forbidden_cells.find(cell) != snapshot_.surface.forbidden_cells.end())
      {
        continue;
      }

      const double clearance = snapshot_.clearance.clearanceDistance(cell);
      if (clearance >= required_clearance_m) {
        const double score = xy_distance + 0.05 / std::max(clearance, 1.0e-3);
        if (!found_safe || score < best_safe_score) {
          best_safe_score = score;
          best_distance = tgw_planner::core::distance3d(point, center);
          best_safe_cell = cell;
          found_safe = true;
        }
      }

      const double fallback_score = clearance - 0.05 * xy_distance;
      if (!found_fallback || fallback_score > best_fallback_score) {
        best_fallback_score = fallback_score;
        best_fallback_distance = tgw_planner::core::distance3d(point, center);
        best_fallback_cell = cell;
        found_fallback = true;
      }
    }

    if (found_safe) {
      *nearest_cell = best_safe_cell;
      *nearest_distance_m = best_distance;
      return true;
    }
    if (found_fallback) {
      *nearest_cell = best_fallback_cell;
      *nearest_distance_m = best_fallback_distance;
      return true;
    }
    *nearest_distance_m = 0.0;
    return false;
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

    GridIndex start_cell;
    GridIndex goal_cell;
    double start_snap_distance = 0.0;
    double goal_snap_distance = 0.0;
    if (!nearestTraversableCell(start, &start_cell, &start_snap_distance)) {
      plan.message = "start could not snap to traversable experience surface";
      if (stats_out != nullptr) {
        *stats_out = makePlannerStats(plan, 0.0, start_snap_distance, goal_snap_distance);
      }
      if (search_time_ms_out != nullptr) {
        *search_time_ms_out = 0.0;
      }
      return plan;
    }
    if (!nearestTraversableCell(goal, &goal_cell, &goal_snap_distance)) {
      plan.message = "goal could not snap to traversable experience surface";
      if (stats_out != nullptr) {
        *stats_out = makePlannerStats(plan, 0.0, start_snap_distance, goal_snap_distance);
      }
      if (search_time_ms_out != nullptr) {
        *search_time_ms_out = 0.0;
      }
      return plan;
    }

    const auto t0 = std::chrono::steady_clock::now();
    plan = SurfaceAstarPlanner(plannerOptions()).plan(snapshot_, start_cell, goal_cell);
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
    raw_path_pub_->publish(makePathMsg(stamp, frame_id, plan.raw_path));
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
    GridIndex snapped_cell;
    double distance = 0.0;
    if (nearestTraversableCell(point, &snapped_cell, &distance)) {
      if (snap_distance_m != nullptr) {
        *snap_distance_m = distance;
      }
      return cellCenter(snapped_cell, snapshot_.resolution_m);
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
    RCLCPP_INFO(
      get_logger(),
      "PlanPath success=%s waypoints=%zu raw_waypoints=%zu expanded=%u generated=%u search_ms=%.3f message=%s",
      plan.success ? "true" : "false",
      plan.path.size(),
      plan.raw_path.size(),
      plan.metrics.expanded_nodes,
      plan.metrics.generated_nodes,
      search_time_ms,
      plan.message.c_str());
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
    raw_path_pub_->publish(makePathMsg(stamp, frame_id, {}));
    start_marker_pub_->publish(makeDeleteAllMarker(frame_id));
    goal_marker_pub_->publish(makeDeleteAllMarker(frame_id));
    plan_status_marker_pub_->publish(makeDeleteAllMarker(frame_id));

    response->success = true;
    response->message = "cleared TGW start, goal, and planned path";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
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
    publishPlanStatusMarker(plan);
    RCLCPP_INFO(
      get_logger(),
      "clicked PlanPath success=%s waypoints=%zu raw_waypoints=%zu expanded=%u generated=%u search_ms=%.3f message=%s",
      plan.success ? "true" : "false",
      plan.path.size(),
      plan.raw_path.size(),
      plan.metrics.expanded_nodes,
      plan.metrics.generated_nodes,
      search_time_ms,
      plan.message.c_str());
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
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
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
  bool has_snapshot_{false};
  Point3 clicked_start_;
  Point3 clicked_goal_;
  bool has_clicked_start_{false};
  bool has_clicked_goal_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TgwExperiencePlannerNode>());
  rclcpp::shutdown();
  return 0;
}
