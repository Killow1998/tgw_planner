#include <algorithm>
#include <cstddef>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"

#include "tgw_planner/core/experience_surface_builder.hpp"
#include "tgw_planner/core/grid_index.hpp"
#include "tgw_planner/core/n3map_reader.hpp"
#include "tgw_planner/core/trajectory_projector.hpp"

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
  json << ",\"projected_support_count\":" <<
    (projection ? projection->projected_support_samples.size() : 0U);
  json << ",\"proven_reachable_count\":" <<
    (projection ? projection->proven_seed_cells.size() : 0U);
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
    declare_parameter<int>("max_trajectory_points", 200000);
    declare_parameter<int>("max_geometry_debug_points", 400000);

    const auto latched_qos = rclcpp::QoS(1).transient_local().reliable();
    trajectory_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tgw_experience/trajectory_cloud", latched_qos);
    keyframe_geometry_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tgw_experience/keyframe_geometry_cloud", latched_qos);
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
    stats_json_pub_ = create_publisher<std_msgs::msg::String>(
      "/tgw_experience/stats_json", latched_qos);

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
      "loaded n3map experience resource: keyframes=%zu dense_trajectory=%zu projected_support=%zu proven_reachable=%zu expanded_reachable=%zu rejected_projection=%zu no_support=%zu ambiguous_multifloor=%zu reanchored_support=%zu retry_support=%zu bridge_seeds=%zu footprint_rejected=%zu body_obstructed_rejected=%zu anchor_envelope_rejected=%zu hole_filled=%zu frame=%s body=%s",
      result.resource.keyframes.size(),
      result.resource.dense_trajectory.size(),
      projection.projected_support_samples.size(),
      projection.proven_seed_cells.size(),
      build.success ? build.snapshot.surface.traversable_cells.size() : 0U,
      projection.rejected_samples.size(),
      rejected_counts["support_projection_failed"],
      rejected_counts["support_projection_ambiguous_multifloor"],
      projection.reanchored_support_samples,
      projection.retry_support_samples,
      projection.trajectory_bridge_seed_count,
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
    proven_reachable_points.reserve(projection.proven_seed_cells.size());
    const double resolution_m = projectorOptions().resolution_m;
    for (const auto & cell : projection.proven_seed_cells) {
      proven_reachable_points.push_back({
        (static_cast<double>(cell.x) + 0.5) * resolution_m,
        (static_cast<double>(cell.y) + 0.5) * resolution_m,
        (static_cast<double>(cell.z) + 0.5) * resolution_m});
    }
    proven_reachable_pub_->publish(
      makePointCloud(stamp, frame_id, proven_reachable_points));

    std::vector<Point3> trajectory_bridge_points;
    trajectory_bridge_points.reserve(projection.bridged_seed_cells.size());
    for (const auto & cell : projection.bridged_seed_cells) {
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
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_json_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TgwExperiencePlannerNode>());
  rclcpp::shutdown();
  return 0;
}
