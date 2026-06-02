#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_set>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "tgw_planner/core/navigation_map.hpp"
#include "tgw_planner/core/voxel_astar_planner.hpp"
#include "tgw_planner/msg/planner_stats.hpp"
#include "tgw_planner/srv/plan_path.hpp"
#include "tgw_planner/srv/set_blocked_region.hpp"

namespace
{
using tgw_planner::core::BuildStats;
using tgw_planner::core::GridIndex;
using tgw_planner::core::GridIndexHash;
using tgw_planner::core::MapCounts;
using tgw_planner::core::NavigationMap;
using tgw_planner::core::PlanResult;
using tgw_planner::core::PlannerMetrics;
using tgw_planner::core::Point3;
using tgw_planner::core::StairFlight;
using tgw_planner::core::StairFlightRejectReason;
using tgw_planner::core::VoxelAstarPlanner;

Point3 toCorePoint(const geometry_msgs::msg::Point & point)
{
  return {point.x, point.y, point.z};
}

Point3 toCorePoint(const geometry_msgs::msg::PoseStamped & pose)
{
  return {pose.pose.position.x, pose.pose.position.y, pose.pose.position.z};
}

geometry_msgs::msg::Point toRosPoint(const Point3 & point)
{
  geometry_msgs::msg::Point out;
  out.x = point.x;
  out.y = point.y;
  out.z = point.z;
  return out;
}

std::size_t markerStride(std::size_t count, std::size_t limit)
{
  if (limit == 0U || count <= limit) {
    return 1U;
  }
  return static_cast<std::size_t>(std::ceil(static_cast<double>(count) / static_cast<double>(limit)));
}
}  // namespace

class TgwPlannerNode : public rclcpp::Node
{
public:
  TgwPlannerNode()
  : Node("tgw_planner_node")
  {
    declare_parameter<std::string>("pcd_file", "");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("map_id", "tgw_nav_map");
    declare_parameter<double>("robot_radius_m", 0.35);
    declare_parameter<double>("robot_height_m", 0.50);
    declare_parameter<double>("robot_length_m", 0.70);
    declare_parameter<double>("robot_width_m", 0.43);
    declare_parameter<double>("base_to_front_m", 0.20);
    declare_parameter<double>("map_resolution_m", 0.20);
    declare_parameter<int>("max_iterations", 250000);
    declare_parameter<int>("max_marker_cells", 120000);
    declare_parameter<std::string>("save_map_dir", "");

    planner_ = std::make_unique<VoxelAstarPlanner>(
      static_cast<std::uint32_t>(std::max<std::int64_t>(1, get_parameter("max_iterations").as_int())));

    const auto latched_qos = rclcpp::QoS(1).transient_local().reliable();
    occupied_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/nav_map/occupied_markers", latched_qos);
    traversable_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/nav_map/traversable_markers", latched_qos);
    blocked_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/nav_map/blocked_markers", latched_qos);
    occupied_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/nav_map/occupied_cloud", latched_qos);
    traversable_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/nav_map/traversable_cloud", latched_qos);
    forbidden_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/nav_map/forbidden_cloud", latched_qos);
    blocked_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/nav_map/blocked_cloud", latched_qos);
    risk_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/nav_map/risk_cloud", latched_qos);
    surface_candidates_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/nav_map/surface_candidates_cloud", latched_qos);
    accepted_floor_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/nav_map/accepted_floor_cloud", latched_qos);
    accepted_stair_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/nav_map/accepted_stair_cloud", latched_qos);
    stair_flight_id_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/nav_map/stair_flight_id_cloud", latched_qos);
    rejected_ceiling_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/nav_map/rejected_ceiling_cloud", latched_qos);
    rejected_clearance_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/nav_map/rejected_clearance_cloud", latched_qos);
    rejected_collision_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/nav_map/rejected_collision_cloud", latched_qos);
    rejected_stair_noise_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/nav_map/rejected_stair_noise_cloud", latched_qos);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/planned_path", latched_qos);
    path_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/planned_path_marker", latched_qos);
    stair_centerline_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/nav_map/stair_centerline_markers", latched_qos);
    landing_component_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/nav_map/landing_component_markers", latched_qos);
    stair_entry_exit_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/nav_map/stair_entry_exit_markers", latched_qos);
    stair_safe_corridor_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/nav_map/stair_safe_corridor_markers", latched_qos);
    start_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/start_marker", latched_qos);
    goal_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/goal_marker", latched_qos);
    stats_pub_ = create_publisher<tgw_planner::msg::PlannerStats>(
      "/planner_stats", latched_qos);
    stats_json_pub_ = create_publisher<std_msgs::msg::String>("/planner_stats_json", latched_qos);

    start_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/start_point", latched_qos, std::bind(&TgwPlannerNode::onStartPoint, this, std::placeholders::_1));
    goal_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", latched_qos, std::bind(&TgwPlannerNode::onGoalPoint, this, std::placeholders::_1));
    start_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/start_pose", latched_qos, std::bind(&TgwPlannerNode::onStartPose, this, std::placeholders::_1));
    goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", latched_qos, std::bind(&TgwPlannerNode::onGoalPose, this, std::placeholders::_1));
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", latched_qos,
      std::bind(&TgwPlannerNode::onInitialPose, this, std::placeholders::_1));

    plan_srv_ = create_service<tgw_planner::srv::PlanPath>(
      "/plan_path", std::bind(
        &TgwPlannerNode::handlePlanPath, this, std::placeholders::_1, std::placeholders::_2));
    blocked_srv_ = create_service<tgw_planner::srv::SetBlockedRegion>(
      "/nav_map/set_blocked_region",
      std::bind(
        &TgwPlannerNode::handleSetBlockedRegion, this, std::placeholders::_1,
        std::placeholders::_2));

    const auto pcd_file = get_parameter("pcd_file").as_string();
    if (!pcd_file.empty()) {
      loadMap(pcd_file);
    } else {
      RCLCPP_WARN(get_logger(), "[NavMapBuilder] no pcd_file set; /plan_path will fail until restart with a map");
    }
  }

private:
  void loadMap(const std::string & pcd_file)
  {
    const double requested_resolution = get_parameter("map_resolution_m").as_double();
    if (requested_resolution <= 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "[NavMapBuilder] map_resolution_m not set, using default 0.20 m");
    }

    BuildStats stats;
    const bool ok = map_.loadFromPcd(
      pcd_file, requested_resolution, get_parameter("robot_radius_m").as_double(),
      get_parameter("robot_height_m").as_double(), get_parameter("map_frame").as_string(),
      get_parameter("map_id").as_string(), stats, get_parameter("robot_length_m").as_double(),
      get_parameter("robot_width_m").as_double(), get_parameter("base_to_front_m").as_double());
    last_build_stats_ = stats;
    if (!ok) {
      RCLCPP_ERROR(get_logger(), "[NavMapBuilder] %s", stats.message.c_str());
      return;
    }

    RCLCPP_INFO(get_logger(), "[NavMapBuilder] source_pcd: %s", stats.source_pcd.c_str());
    RCLCPP_INFO(get_logger(), "[NavMapBuilder] source_points: %lu", stats.source_points);
    RCLCPP_INFO(get_logger(), "[NavMapBuilder] resolution_m: %.3f", map_.resolution());
    RCLCPP_INFO(
      get_logger(),
      "[NavMapBuilder] footprint length/width/height/base_to_front: %.3f / %.3f / %.3f / %.3f",
      map_.robotLength(), map_.robotWidth(), map_.robotHeight(), map_.baseToFront());
    RCLCPP_INFO(get_logger(), "[NavMapBuilder] occupied_cells: %u", stats.counts.occupied_cells);
    RCLCPP_INFO(get_logger(), "[NavMapBuilder] traversable_cells: %u", stats.counts.traversable_cells);
    RCLCPP_INFO(
      get_logger(), "[NavMapBuilder] surface_candidates: %zu",
      map_.surfaceCandidateCells().size());
    RCLCPP_INFO(
      get_logger(), "[NavMapBuilder] accepted_floor_cells: %zu",
      map_.acceptedFloorCells().size());
    RCLCPP_INFO(
      get_logger(), "[NavMapBuilder] accepted_stair_cells: %zu",
      map_.acceptedStairCells().size());
    RCLCPP_INFO(
      get_logger(), "[NavMapBuilder] stair_centerlines: %zu",
      map_.stairCenterlines().size());
    RCLCPP_INFO(
      get_logger(), "[NavMapBuilder] floor_component_count: %zu",
      map_.floorComponents().size());
    RCLCPP_INFO(
      get_logger(), "[NavMapBuilder] landing_component_count: %zu",
      map_.landingComponents().size());
    RCLCPP_INFO(
      get_logger(), "[NavMapBuilder] accepted_stair_flight_count: %zu",
      map_.stairFlights().size());
    const auto & stair_diag = map_.stairFlightDiagnostics();
    RCLCPP_INFO(
      get_logger(),
      "[NavMapBuilder] stair_flight_diagnostics: raw_segments=%zu "
      "segment_width_rejected=%zu fit_rejected=%zu accepted_candidates=%zu "
      "merged_candidates=%zu",
      stair_diag.raw_segments, stair_diag.segment_width_rejected, stair_diag.fit_rejected,
      stair_diag.accepted_candidates, stair_diag.merged_candidates);
    RCLCPP_INFO(
      get_logger(),
      "[NavMapBuilder] stair_flight_fit_rejects: too_few=%zu no_axis=%zu short_or_low=%zu "
      "negative_slope=%zu slope_out=%zu residual=%zu nonmonotonic=%zu narrow=%zu "
      "missing_portals=%zu same_floor_ends=%zu",
      stair_diag.fit_reject_counts[static_cast<std::size_t>(
        StairFlightRejectReason::TooFewCells)],
      stair_diag.fit_reject_counts[static_cast<std::size_t>(
        StairFlightRejectReason::NoAxis)],
      stair_diag.fit_reject_counts[static_cast<std::size_t>(
        StairFlightRejectReason::TooShortOrLow)],
      stair_diag.fit_reject_counts[static_cast<std::size_t>(
        StairFlightRejectReason::NegativeSlope)],
      stair_diag.fit_reject_counts[static_cast<std::size_t>(
        StairFlightRejectReason::SlopeOutOfRange)],
      stair_diag.fit_reject_counts[static_cast<std::size_t>(
        StairFlightRejectReason::ResidualTooHigh)],
      stair_diag.fit_reject_counts[static_cast<std::size_t>(
        StairFlightRejectReason::NonMonotonic)],
      stair_diag.fit_reject_counts[static_cast<std::size_t>(
        StairFlightRejectReason::TooNarrow)],
      stair_diag.fit_reject_counts[static_cast<std::size_t>(
        StairFlightRejectReason::MissingPortals)],
      stair_diag.fit_reject_counts[static_cast<std::size_t>(
        StairFlightRejectReason::SameFloorBothEnds)]);
    RCLCPP_INFO(
      get_logger(), "[NavMapBuilder] rejected_stair_noise_cells: %zu",
      map_.rejectedStairNoiseCells().size());
    for (const auto & flight : map_.stairFlights()) {
      RCLCPP_INFO(
        get_logger(),
        "[NavMapBuilder] StairFlight id=%d cells=%zu width=%.3f length=%.3f slope=%.3f "
        "z=[%.3f, %.3f] components=[%d, %d] low=[%.3f, %.3f, %.3f] high=[%.3f, %.3f, %.3f]",
        flight.id, flight.cells.size(), flight.width_m, flight.length_m, flight.slope,
        flight.z_min, flight.z_max, flight.low_component_id, flight.high_component_id,
        flight.low_endpoint.x, flight.low_endpoint.y, flight.low_endpoint.z,
        flight.high_endpoint.x, flight.high_endpoint.y, flight.high_endpoint.z);
    }
    RCLCPP_INFO(
      get_logger(), "[NavMapBuilder] forbidden_cells: %zu", map_.forbiddenCells().size());
    RCLCPP_INFO(
      get_logger(), "[NavMapBuilder] rejected_ceiling_cells: %zu",
      map_.rejectedCeilingCells().size());
    RCLCPP_INFO(
      get_logger(), "[NavMapBuilder] rejected_clearance_cells: %zu",
      map_.rejectedClearanceCells().size());
    RCLCPP_INFO(
      get_logger(), "[NavMapBuilder] rejected_collision_cells: %zu",
      map_.rejectedCollisionCells().size());
    RCLCPP_INFO(get_logger(), "[NavMapBuilder] blocked_cells: %u", stats.counts.blocked_cells);
    RCLCPP_INFO(get_logger(), "[NavMapBuilder] risk_cells: %u", stats.counts.risk_cells);
    RCLCPP_INFO(get_logger(), "[NavMapBuilder] build_time_ms: %.3f", stats.build_time_ms);
    RCLCPP_INFO(
      get_logger(), "[NavMapBuilder] bounds_min: [%.3f, %.3f, %.3f]",
      stats.bounds_min.x, stats.bounds_min.y, stats.bounds_min.z);
    RCLCPP_INFO(
      get_logger(), "[NavMapBuilder] bounds_max: [%.3f, %.3f, %.3f]",
      stats.bounds_max.x, stats.bounds_max.y, stats.bounds_max.z);
    publishMapLayers();
    saveMapPackageIfRequested();
  }

  void onStartPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose.position = msg->point;
    pose.pose.orientation.w = 1.0;
    setStart(pose);
  }

  void onGoalPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose.position = msg->point;
    pose.pose.orientation.w = 1.0;
    setGoal(pose);
  }

  void onStartPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    setStart(*msg);
  }

  void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    setGoal(*msg);
  }

  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;
    setStart(pose);
  }

  void setStart(const geometry_msgs::msg::PoseStamped & pose)
  {
    start_pose_ = snapPoseForDisplay(normalizedPose(pose), "start");
    has_start_ = true;
    publishPoseMarker(start_pose_, start_marker_pub_, "start", 0.0F, 0.95F, 1.0F);
    planIfReady();
  }

  void setGoal(const geometry_msgs::msg::PoseStamped & pose)
  {
    goal_pose_ = snapPoseForDisplay(normalizedPose(pose), "goal");
    has_goal_ = true;
    publishPoseMarker(goal_pose_, goal_marker_pub_, "goal", 1.0F, 0.82F, 0.0F);
    planIfReady();
  }

  geometry_msgs::msg::PoseStamped snapPoseForDisplay(
    const geometry_msgs::msg::PoseStamped & pose, const std::string & label)
  {
    if (!map_.ready()) {
      return pose;
    }
    GridIndex snapped;
    double distance_m = 0.0;
    if (!planner_->snapToTraversable(map_, toCorePoint(pose), snapped, distance_m)) {
      RCLCPP_WARN(
        get_logger(), "[Planner] %s pose could not snap to a traversable layer", label.c_str());
      return pose;
    }

    auto out = pose;
    out.header.frame_id = map_.mapFrame();
    out.pose.position = toRosPoint(map_.gridToWorld(snapped));
    RCLCPP_INFO(
      get_logger(), "[Planner] %s pose snapped by %.3f m to [%.3f, %.3f, %.3f]",
      label.c_str(), distance_m, out.pose.position.x, out.pose.position.y, out.pose.position.z);
    return out;
  }

  geometry_msgs::msg::PoseStamped normalizedPose(const geometry_msgs::msg::PoseStamped & pose) const
  {
    auto out = pose;
    out.header.frame_id = out.header.frame_id.empty() ? map_.mapFrame() : out.header.frame_id;
    if (out.pose.orientation.w == 0.0 && out.pose.orientation.x == 0.0 &&
      out.pose.orientation.y == 0.0 && out.pose.orientation.z == 0.0)
    {
      out.pose.orientation.w = 1.0;
    }
    return out;
  }

  void planIfReady()
  {
    if (has_start_ && has_goal_) {
      PlanResult result = runPlanner(start_pose_, goal_pose_);
      logPlanResult(result, start_pose_, goal_pose_);
    }
  }

  void handlePlanPath(
    const std::shared_ptr<tgw_planner::srv::PlanPath::Request> request,
    std::shared_ptr<tgw_planner::srv::PlanPath::Response> response)
  {
    const auto start = normalizedPose(request->start);
    const auto goal = normalizedPose(request->goal);
    PlanResult result = runPlanner(start, goal);
    response->success = result.success;
    response->message = result.message;
    response->path = toPathMsg(result, goal.pose.orientation);
    response->stats = toStatsMsg(result.metrics);
    logPlanResult(result, start, goal);
  }

  void handleSetBlockedRegion(
    const std::shared_ptr<tgw_planner::srv::SetBlockedRegion::Request> request,
    std::shared_ptr<tgw_planner::srv::SetBlockedRegion::Response> response)
  {
    if (!map_.ready()) {
      response->success = false;
      response->message = "navigation map is not ready";
      response->affected_cells = 0;
      return;
    }

    if (request->operation == "add") {
      response->affected_cells = map_.addBlockedRegion(toCorePoint(request->min), toCorePoint(request->max));
      response->success = true;
      response->message = "blocked region added";
    } else if (request->operation == "remove") {
      response->affected_cells =
        map_.removeBlockedRegion(toCorePoint(request->min), toCorePoint(request->max));
      response->success = true;
      response->message = "blocked region removed";
    } else if (request->operation == "clear") {
      response->affected_cells = map_.clearBlockedRegions();
      response->success = true;
      response->message = "blocked regions cleared";
    } else {
      response->success = false;
      response->message = "operation must be add, remove, or clear";
      response->affected_cells = 0;
      return;
    }

    publishMapLayers();
    saveMapPackageIfRequested();
    RCLCPP_INFO(
      get_logger(), "[NavMapBuilder] blocked operation=%s affected_cells=%u reason=%s",
      request->operation.c_str(), response->affected_cells, request->reason.c_str());
  }

  void saveMapPackageIfRequested()
  {
    const std::string save_map_dir = get_parameter("save_map_dir").as_string();
    if (save_map_dir.empty()) {
      return;
    }

    std::string message;
    if (map_.saveToMapPackage(save_map_dir, last_build_stats_.source_pcd, message)) {
      RCLCPP_INFO(get_logger(), "[NavMapBuilder] %s", message.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "[NavMapBuilder] %s", message.c_str());
    }
  }

  PlanResult runPlanner(
    const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
  {
    PlanResult result = planner_->plan(map_, toCorePoint(start), toCorePoint(goal));
    publishSnappedPoseMarkers(result, start, goal);
    const nav_msgs::msg::Path path_msg = toPathMsg(result, goal.pose.orientation);
    path_pub_->publish(path_msg);
    publishPathMarker(result);
    auto stats_msg = toStatsMsg(result.metrics);
    stats_pub_->publish(stats_msg);
    std_msgs::msg::String json_msg;
    json_msg.data = toStatsJson(result.metrics);
    stats_json_pub_->publish(json_msg);
    return result;
  }

  void publishSnappedPoseMarkers(
    const PlanResult & result, const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
  {
    if (result.start_snap_success) {
      auto snapped_start = start;
      snapped_start.header.frame_id = map_.mapFrame();
      snapped_start.pose.position = toRosPoint(map_.gridToWorld(result.start_cell));
      publishPoseMarker(snapped_start, start_marker_pub_, "start", 0.0F, 0.95F, 1.0F);
    }
    if (result.goal_snap_success) {
      auto snapped_goal = goal;
      snapped_goal.header.frame_id = map_.mapFrame();
      snapped_goal.pose.position = toRosPoint(map_.gridToWorld(result.goal_cell));
      publishPoseMarker(snapped_goal, goal_marker_pub_, "goal", 1.0F, 0.82F, 0.0F);
    }
  }

  nav_msgs::msg::Path toPathMsg(
    const PlanResult & result, const geometry_msgs::msg::Quaternion & goal_orientation) const
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = map_.mapFrame();
    path.poses.reserve(result.path.size());
    for (std::size_t i = 0; i < result.path.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = toRosPoint(result.path[i]);
      pose.pose.orientation.w = 1.0;
      if (i + 1U == result.path.size()) {
        pose.pose.orientation = goal_orientation;
      }
      path.poses.push_back(pose);
    }
    return path;
  }

  tgw_planner::msg::PlannerStats toStatsMsg(const PlannerMetrics & metrics) const
  {
    const MapCounts counts = map_.counts();
    tgw_planner::msg::PlannerStats msg;
    msg.stamp = now();
    msg.map_id = map_.mapId();
    msg.success = metrics.success;
    msg.failure_reason = metrics.failure_reason;
    msg.build_time_ms = last_build_stats_.build_time_ms;
    msg.search_time_ms = metrics.search_time_ms;
    msg.total_plan_time_ms = metrics.total_plan_time_ms;
    msg.occupied_cells = counts.occupied_cells;
    msg.traversable_cells = counts.traversable_cells;
    msg.blocked_cells = counts.blocked_cells;
    msg.risk_cells = counts.risk_cells;
    msg.expanded_nodes = metrics.expanded_nodes;
    msg.generated_nodes = metrics.generated_nodes;
    msg.reopened_nodes = metrics.reopened_nodes;
    msg.max_open_set_size = metrics.max_open_set_size;
    msg.path_waypoints = metrics.path_waypoints;
    msg.path_length_m = metrics.path_length_m;
    msg.path_vertical_gain_m = metrics.path_vertical_gain_m;
    msg.path_vertical_loss_m = metrics.path_vertical_loss_m;
    msg.start_snap_distance_m = metrics.start_snap_distance_m;
    msg.goal_snap_distance_m = metrics.goal_snap_distance_m;
    msg.map_resolution_m = map_.resolution();
    msg.robot_radius_m = map_.robotRadius();
    return msg;
  }

  std::string toStatsJson(const PlannerMetrics & metrics) const
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    out << "{";
    out << "\"success\":" << (metrics.success ? "true" : "false") << ",";
    out << "\"map_id\":\"" << map_.mapId() << "\",";
    out << "\"failure_reason\":\"" << metrics.failure_reason << "\",";
    out << "\"search_time_ms\":" << metrics.search_time_ms << ",";
    out << "\"expanded_nodes\":" << metrics.expanded_nodes << ",";
    out << "\"generated_nodes\":" << metrics.generated_nodes << ",";
    out << "\"max_open_set_size\":" << metrics.max_open_set_size << ",";
    out << "\"path_waypoints\":" << metrics.path_waypoints << ",";
    out << "\"path_length_m\":" << metrics.path_length_m << ",";
    out << "\"vertical_gain_m\":" << metrics.path_vertical_gain_m << ",";
    out << "\"vertical_loss_m\":" << metrics.path_vertical_loss_m << ",";
    out << "\"start_snap_distance_m\":" << metrics.start_snap_distance_m << ",";
    out << "\"goal_snap_distance_m\":" << metrics.goal_snap_distance_m << ",";
    out << "\"resolution_m\":" << map_.resolution();
    out << "}";
    return out.str();
  }

  void publishMapLayers()
  {
    if (!map_.ready()) {
      return;
    }
    publishCellSetMarker(map_.occupiedCells(), occupied_marker_pub_, "occupied", 0.25F, 0.25F, 0.25F, 0.35F);
    publishCellSetMarker(
      map_.traversableCells(), traversable_marker_pub_, "traversable", 0.15F, 0.75F, 0.45F, 0.55F);
    publishCellSetMarker(map_.blockedCells(), blocked_marker_pub_, "blocked", 0.95F, 0.1F, 0.1F, 0.8F);
    publishCellSetCloud(map_.occupiedCells(), occupied_cloud_pub_, 1.0F);
    publishCellSetCloud(map_.traversableCells(), traversable_cloud_pub_, 2.0F);
    publishCellSetCloud(map_.forbiddenCells(), forbidden_cloud_pub_, 3.0F);
    publishCellSetCloud(map_.blockedCells(), blocked_cloud_pub_, 3.0F);
    publishCellSetCloud(map_.surfaceCandidateCells(), surface_candidates_cloud_pub_, 4.0F);
    publishCellSetCloud(map_.acceptedFloorCells(), accepted_floor_cloud_pub_, 5.0F);
    publishCellSetCloud(map_.acceptedStairCells(), accepted_stair_cloud_pub_, 6.0F);
    publishStairFlightIdCloud();
    publishCellSetCloud(map_.rejectedCeilingCells(), rejected_ceiling_cloud_pub_, 7.0F);
    publishCellSetCloud(map_.rejectedClearanceCells(), rejected_clearance_cloud_pub_, 8.0F);
    publishCellSetCloud(map_.rejectedCollisionCells(), rejected_collision_cloud_pub_, 9.0F);
    publishCellSetCloud(map_.rejectedStairNoiseCells(), rejected_stair_noise_cloud_pub_, 10.0F);
    publishRiskCloud();
    publishStairCenterlineMarkers();
    publishLandingComponentMarkers();
    publishStairEntryExitMarkers();
    publishStairSafeCorridorMarkers();
  }

  void publishCellSetMarker(
    const std::unordered_set<GridIndex, GridIndexHash> & cells,
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & publisher,
    const std::string & ns, float r, float g, float b, float a)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = map_.mapFrame();
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = map_.resolution();
    marker.scale.y = map_.resolution();
    marker.scale.z = map_.resolution();
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;

    const auto max_cells =
      static_cast<std::size_t>(std::max<std::int64_t>(0, get_parameter("max_marker_cells").as_int()));
    const std::size_t stride = markerStride(cells.size(), max_cells);
    marker.points.reserve(std::min(cells.size(), max_cells == 0U ? cells.size() : max_cells));
    std::size_t i = 0U;
    for (const auto & cell : cells) {
      if ((i++ % stride) != 0U) {
        continue;
      }
      marker.points.push_back(toRosPoint(map_.gridToWorld(cell)));
    }
    publisher->publish(marker);
  }

  void publishCellSetCloud(
    const std::unordered_set<GridIndex, GridIndexHash> & cells,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
    float intensity)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = now();
    cloud.header.frame_id = map_.mapFrame();
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(cells.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_i(cloud, "intensity");
    for (const auto & cell : cells) {
      const Point3 point = map_.gridToWorld(cell);
      *iter_x = static_cast<float>(point.x);
      *iter_y = static_cast<float>(point.y);
      *iter_z = static_cast<float>(point.z);
      *iter_i = intensity;
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_i;
    }
    publisher->publish(cloud);
  }

  void publishRiskCloud()
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = now();
    cloud.header.frame_id = map_.mapFrame();
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(map_.riskCosts().size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_i(cloud, "intensity");
    for (const auto & entry : map_.riskCosts()) {
      const Point3 point = map_.gridToWorld(entry.first);
      *iter_x = static_cast<float>(point.x);
      *iter_y = static_cast<float>(point.y);
      *iter_z = static_cast<float>(point.z);
      *iter_i = static_cast<float>(entry.second);
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_i;
    }
    risk_cloud_pub_->publish(cloud);
  }

  void publishStairFlightIdCloud()
  {
    std::size_t cell_count = 0U;
    for (const auto & flight : map_.stairFlights()) {
      cell_count += flight.cells.size();
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = now();
    cloud.header.frame_id = map_.mapFrame();
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(cell_count);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_i(cloud, "intensity");
    for (const auto & flight : map_.stairFlights()) {
      for (const auto & cell : flight.cells) {
        const Point3 point = map_.gridToWorld(cell);
        *iter_x = static_cast<float>(point.x);
        *iter_y = static_cast<float>(point.y);
        *iter_z = static_cast<float>(point.z);
        *iter_i = static_cast<float>(flight.id + 1);
        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_i;
      }
    }
    stair_flight_id_cloud_pub_->publish(cloud);
  }

  void publishStairCenterlineMarkers()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = map_.mapFrame();
    marker.ns = "stair_centerlines";
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.18;
    marker.color.r = 0.0F;
    marker.color.g = 0.95F;
    marker.color.b = 1.0F;
    marker.color.a = 1.0F;

    const auto centerlines = map_.stairCenterlines();
    for (const auto & centerline : centerlines) {
      if (centerline.size() < 2U) {
        continue;
      }
      for (std::size_t i = 1; i < centerline.size(); ++i) {
        marker.points.push_back(toRosPoint(centerline[i - 1]));
        marker.points.push_back(toRosPoint(centerline[i]));
      }
    }
    if (marker.points.empty()) {
      marker.action = visualization_msgs::msg::Marker::DELETE;
    }
    stair_centerline_marker_pub_->publish(marker);
  }

  void publishLandingComponentMarkers()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = map_.mapFrame();
    marker.ns = "landing_components";
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.45;
    marker.scale.y = 0.45;
    marker.scale.z = 0.18;
    marker.color.r = 1.0F;
    marker.color.g = 0.9F;
    marker.color.b = 0.0F;
    marker.color.a = 0.9F;
    for (const auto & component : map_.landingComponents()) {
      marker.points.push_back(toRosPoint(component.centroid));
    }
    if (marker.points.empty()) {
      marker.action = visualization_msgs::msg::Marker::DELETE;
    }
    landing_component_marker_pub_->publish(marker);
  }

  void publishStairEntryExitMarkers()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = map_.mapFrame();
    marker.ns = "stair_portals";
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.42;
    marker.scale.y = 0.42;
    marker.scale.z = 0.42;
    marker.color.r = 1.0F;
    marker.color.g = 0.35F;
    marker.color.b = 0.0F;
    marker.color.a = 1.0F;
    for (const auto & flight : map_.stairFlights()) {
      marker.points.push_back(toRosPoint(flight.low_endpoint));
      marker.points.push_back(toRosPoint(flight.high_endpoint));
    }
    if (marker.points.empty()) {
      marker.action = visualization_msgs::msg::Marker::DELETE;
    }
    stair_entry_exit_marker_pub_->publish(marker);
  }

  void publishStairSafeCorridorMarkers()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = map_.mapFrame();
    marker.ns = "stair_safe_corridors";
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.color.r = 0.25F;
    marker.color.g = 0.65F;
    marker.color.b = 1.0F;
    marker.color.a = 0.8F;
    for (const auto & flight : map_.stairFlights()) {
      if (flight.centerline.size() < 2U) {
        continue;
      }
      for (std::size_t i = 1; i < flight.centerline.size(); ++i) {
        for (const double sign : {-1.0, 1.0}) {
          Point3 a = flight.centerline[i - 1];
          Point3 b = flight.centerline[i];
          a.x += sign * flight.side_axis.x * flight.safe_half_width_m;
          a.y += sign * flight.side_axis.y * flight.safe_half_width_m;
          b.x += sign * flight.side_axis.x * flight.safe_half_width_m;
          b.y += sign * flight.side_axis.y * flight.safe_half_width_m;
          marker.points.push_back(toRosPoint(a));
          marker.points.push_back(toRosPoint(b));
        }
      }
    }
    if (marker.points.empty()) {
      marker.action = visualization_msgs::msg::Marker::DELETE;
    }
    stair_safe_corridor_marker_pub_->publish(marker);
  }

  void publishPathMarker(const PlanResult & result)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = map_.mapFrame();
    marker.ns = "planned_path";
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.32;
    marker.color.r = 1.0F;
    marker.color.g = 0.62F;
    marker.color.b = 0.0F;
    marker.color.a = 1.0F;
    marker.points.reserve(result.path.size());
    for (const auto & point : result.path) {
      marker.points.push_back(toRosPoint(point));
    }
    if (marker.points.empty()) {
      marker.action = visualization_msgs::msg::Marker::DELETE;
    }
    path_marker_pub_->publish(marker);
  }

  void publishPoseMarker(
    const geometry_msgs::msg::PoseStamped & pose,
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & publisher,
    const std::string & ns, float r, float g, float b)
  {
    auto make_point = [](double x, double y, double z) {
      geometry_msgs::msg::Point point;
      point.x = x;
      point.y = y;
      point.z = z;
      return point;
    };

    visualization_msgs::msg::Marker base;
    base.header.stamp = now();
    base.header.frame_id = map_.mapFrame();
    base.ns = ns;
    base.action = visualization_msgs::msg::Marker::ADD;
    base.pose.orientation.w = 1.0;
    base.color.r = r;
    base.color.g = g;
    base.color.b = b;
    base.color.a = 1.0;

    visualization_msgs::msg::Marker height_line = base;
    height_line.id = 0;
    height_line.type = visualization_msgs::msg::Marker::LINE_LIST;
    height_line.scale.x = 0.10;
    height_line.points.push_back(make_point(pose.pose.position.x, pose.pose.position.y, 0.0));
    height_line.points.push_back(pose.pose.position);
    publisher->publish(height_line);

    visualization_msgs::msg::Marker arrow = base;
    arrow.id = 1;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.pose = pose.pose;
    arrow.scale.x = 1.2;
    arrow.scale.y = 0.28;
    arrow.scale.z = 0.28;
    publisher->publish(arrow);

    visualization_msgs::msg::Marker text = base;
    text.id = 2;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.pose.position =
      make_point(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z + 0.8);
    text.scale.z = 0.48;
    std::ostringstream label;
    label << ns << " z=" << std::fixed << std::setprecision(2) << pose.pose.position.z << "m";
    text.text = label.str();
    publisher->publish(text);
  }

  void logPlanResult(
    const PlanResult & result, const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
  {
    RCLCPP_INFO(
      get_logger(), "[Planner] start_raw: [%.3f, %.3f, %.3f]", start.pose.position.x,
      start.pose.position.y, start.pose.position.z);
    RCLCPP_INFO(
      get_logger(), "[Planner] goal_raw: [%.3f, %.3f, %.3f]", goal.pose.position.x,
      goal.pose.position.y, goal.pose.position.z);
    RCLCPP_INFO(
      get_logger(), "[Planner] start_snapped: [%d, %d, %d]", result.start_cell.x,
      result.start_cell.y, result.start_cell.z);
    RCLCPP_INFO(
      get_logger(), "[Planner] goal_snapped: [%d, %d, %d]", result.goal_cell.x,
      result.goal_cell.y, result.goal_cell.z);
    RCLCPP_INFO(
      get_logger(), "[Planner] start_cell_type: stair=%s floor_or_landing=%s flight_id=%d",
      map_.isStairCell(result.start_cell) ? "true" : "false",
      map_.isFloorOrLandingCell(result.start_cell) ? "true" : "false",
      map_.stairFlightId(result.start_cell));
    RCLCPP_INFO(
      get_logger(), "[Planner] goal_cell_type: stair=%s floor_or_landing=%s flight_id=%d",
      map_.isStairCell(result.goal_cell) ? "true" : "false",
      map_.isFloorOrLandingCell(result.goal_cell) ? "true" : "false",
      map_.stairFlightId(result.goal_cell));
    RCLCPP_INFO(
      get_logger(), "[Planner] start_snap_distance_m: %.3f",
      result.metrics.start_snap_distance_m);
    RCLCPP_INFO(
      get_logger(), "[Planner] goal_snap_distance_m: %.3f", result.metrics.goal_snap_distance_m);
    RCLCPP_INFO(get_logger(), "[Planner] success: %s", result.success ? "true" : "false");
    if (!result.success) {
      RCLCPP_WARN(get_logger(), "[Planner] failure_reason: %s", result.message.c_str());
    }
    RCLCPP_INFO(get_logger(), "[Planner] search_time_ms: %.3f", result.metrics.search_time_ms);
    RCLCPP_INFO(get_logger(), "[Planner] expanded_nodes: %u", result.metrics.expanded_nodes);
    RCLCPP_INFO(get_logger(), "[Planner] generated_nodes: %u", result.metrics.generated_nodes);
    RCLCPP_INFO(get_logger(), "[Planner] max_open_set_size: %u", result.metrics.max_open_set_size);
    RCLCPP_INFO(get_logger(), "[Planner] path_waypoints: %u", result.metrics.path_waypoints);
    RCLCPP_INFO(get_logger(), "[Planner] path_length_m: %.3f", result.metrics.path_length_m);
    RCLCPP_INFO(get_logger(), "[Planner] vertical_gain_m: %.3f", result.metrics.path_vertical_gain_m);
    RCLCPP_INFO(get_logger(), "[Planner] vertical_loss_m: %.3f", result.metrics.path_vertical_loss_m);
  }

  NavigationMap map_;
  BuildStats last_build_stats_;
  std::unique_ptr<VoxelAstarPlanner> planner_;

  bool has_start_{false};
  bool has_goal_{false};
  geometry_msgs::msg::PoseStamped start_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr occupied_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traversable_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr blocked_marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr traversable_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr forbidden_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr blocked_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr risk_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surface_candidates_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr accepted_floor_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr accepted_stair_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr stair_flight_id_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rejected_ceiling_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rejected_clearance_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rejected_collision_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rejected_stair_noise_cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr stair_centerline_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr landing_component_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr stair_entry_exit_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr stair_safe_corridor_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr start_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
  rclcpp::Publisher<tgw_planner::msg::PlannerStats>::SharedPtr stats_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_json_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr start_point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

  rclcpp::Service<tgw_planner::srv::PlanPath>::SharedPtr plan_srv_;
  rclcpp::Service<tgw_planner::srv::SetBlockedRegion>::SharedPtr blocked_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TgwPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
