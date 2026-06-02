#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

#include "tgw_planner/core/clearance_field.hpp"
#include "tgw_planner/core/map_snapshot.hpp"
#include "tgw_planner/core/mapping_options.hpp"
#include "tgw_planner/core/path_validator.hpp"
#include "tgw_planner/core/probabilistic_voxel_map.hpp"
#include "tgw_planner/core/raycast_integrator.hpp"
#include "tgw_planner/core/risk_field.hpp"
#include "tgw_planner/core/robot_footprint.hpp"
#include "tgw_planner/core/surface_astar_planner.hpp"
#include "tgw_planner/core/surface_extractor.hpp"
#include "tgw_planner/msg/mapping_stats.hpp"
#include "tgw_planner/msg/planner_stats.hpp"
#include "tgw_planner/srv/export_static_cloud.hpp"
#include "tgw_planner/srv/get_snapshot.hpp"
#include "tgw_planner/srv/load_map.hpp"
#include "tgw_planner/srv/plan_path.hpp"
#include "tgw_planner/srv/save_map.hpp"
#include "tgw_planner/srv/set_blocked_region.hpp"

namespace
{
using tgw_planner::core::GridIndex;
using tgw_planner::core::MappingOptions;
using tgw_planner::core::NavigationSnapshot;
using tgw_planner::core::Point3;
using tgw_planner::core::PathValidationOptions;
using tgw_planner::core::PathValidator;
using tgw_planner::core::Pose3;
using tgw_planner::core::ProbabilisticVoxelMap;
using tgw_planner::core::RaycastIntegrator;
using tgw_planner::core::RaycastStats;
using tgw_planner::core::RiskField;
using tgw_planner::core::RiskFieldOptions;
using tgw_planner::core::RobotFootprint;
using tgw_planner::core::RobotFootprintOptions;
using tgw_planner::core::ScanInput;
using tgw_planner::core::SelfFilterBox;
using tgw_planner::core::SurfaceExtractionOptions;
using tgw_planner::core::SurfaceExtractor;
using tgw_planner::core::SurfaceMap;
using tgw_planner::core::ClearanceField;
using tgw_planner::core::SurfaceAstarPlanner;
using tgw_planner::core::SurfacePlannerOptions;

double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + 1.0e-9 * static_cast<double>(stamp.nanosec);
}

Pose3 poseFromTransform(const geometry_msgs::msg::TransformStamped & transform)
{
  Pose3 pose;
  pose.translation = {
    transform.transform.translation.x,
    transform.transform.translation.y,
    transform.transform.translation.z};

  tf2::Quaternion q{
    transform.transform.rotation.x,
    transform.transform.rotation.y,
    transform.transform.rotation.z,
    transform.transform.rotation.w};
  q.normalize();
  tf2::Matrix3x3 matrix(q);
  pose.rotation = {
    matrix[0][0], matrix[0][1], matrix[0][2],
    matrix[1][0], matrix[1][1], matrix[1][2],
    matrix[2][0], matrix[2][1], matrix[2][2]};
  return pose;
}

Pose3 poseFromPoseStamped(const geometry_msgs::msg::PoseStamped & msg)
{
  Pose3 pose;
  pose.translation = {msg.pose.position.x, msg.pose.position.y, msg.pose.position.z};
  tf2::Quaternion q{
    msg.pose.orientation.x,
    msg.pose.orientation.y,
    msg.pose.orientation.z,
    msg.pose.orientation.w};
  if (q.length2() <= 1.0e-9) {
    q = tf2::Quaternion{0.0, 0.0, 0.0, 1.0};
  }
  q.normalize();
  tf2::Matrix3x3 matrix(q);
  pose.rotation = {
    matrix[0][0], matrix[0][1], matrix[0][2],
    matrix[1][0], matrix[1][1], matrix[1][2],
    matrix[2][0], matrix[2][1], matrix[2][2]};
  return pose;
}

double yawFromPoseStamped(const geometry_msgs::msg::PoseStamped & msg)
{
  tf2::Quaternion q{
    msg.pose.orientation.x,
    msg.pose.orientation.y,
    msg.pose.orientation.z,
    msg.pose.orientation.w};
  if (q.length2() <= 1.0e-9) {
    q = tf2::Quaternion{0.0, 0.0, 0.0, 1.0};
  }
  q.normalize();
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}
}  // namespace

class TgwRealtimeMappingNode : public rclcpp::Node
{
public:
  TgwRealtimeMappingNode()
  : Node("tgw_realtime_mapping_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    const MappingOptions mapping_options = mappingOptions();
    map_ = ProbabilisticVoxelMap(mapping_options);
    integrator_ = RaycastIntegrator(mapping_options, selfFilterBox(), mountSelfFilterBox());
    surface_extractor_ = SurfaceExtractor(surfaceOptions());
    footprint_ = RobotFootprint(footprintOptions());
    risk_options_ = riskOptions();
    planner_options_ = plannerOptions();
    planner_options_.footprint = footprint_.options();
    validation_options_ = validationOptions();

    points_topic_ = declare_parameter<std::string>("points_topic", "/tgw_mapping/points");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/tgw_mapping/pose");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    sensor_frame_override_ = declare_parameter<std::string>("sensor_frame", "");
    use_tf_ = declare_parameter<bool>("use_tf", true);
    assume_cloud_in_map_frame_ = declare_parameter<bool>("assume_cloud_in_map_frame", false);
    mapping_enabled_ = declare_parameter<bool>("start_enabled", true);
    max_points_per_scan_ = declare_parameter<int>("max_points_per_scan", 120000);
    publish_period_ms_ = declare_parameter<int>("publish_period_ms", 1000);
    medial_axis_min_clearance_m_ =
      declare_parameter<double>("medial_axis_min_clearance_m", medial_axis_min_clearance_m_);
    max_snap_distance_m_ = std::max(
      map_.resolution(),
      declare_parameter<double>("planner_max_snap_distance_m", max_snap_distance_m_));

    rclcpp::QoS cloud_qos{rclcpp::SensorDataQoS()};
    points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      points_topic_, cloud_qos,
      std::bind(&TgwRealtimeMappingNode::onPointCloud, this, std::placeholders::_1));
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, rclcpp::QoS(10),
      std::bind(&TgwRealtimeMappingNode::onPose, this, std::placeholders::_1));

    rclcpp::QoS latched_qos(1);
    latched_qos.transient_local().reliable();
    occupied_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/tgw_map/occupied_cloud", latched_qos);
    free_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/tgw_map/free_cloud", latched_qos);
    dynamic_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/tgw_map/dynamic_suspect_cloud", latched_qos);
    static_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/tgw_map/static_candidate_cloud", latched_qos);
    surface_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/tgw_map/surface_cloud", latched_qos);
    traversable_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/tgw_map/traversable_cloud", latched_qos);
    boundary_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/tgw_map/boundary_cloud", latched_qos);
    dropoff_boundary_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("/tgw_map/dropoff_boundary_cloud", latched_qos);
    wall_boundary_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("/tgw_map/wall_boundary_cloud", latched_qos);
    clearance_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/tgw_map/clearance_cloud", latched_qos);
    medial_axis_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("/tgw_map/medial_axis_cloud", latched_qos);
    risk_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/tgw_map/risk_cloud", latched_qos);
    blocked_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/tgw_map/blocked_cloud", latched_qos);
    forbidden_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/tgw_map/forbidden_cloud", latched_qos);
    planned_path_pub_ = create_publisher<nav_msgs::msg::Path>("/tgw_map/planned_path", latched_qos);
    planned_path_compat_pub_ = create_publisher<nav_msgs::msg::Path>("/planned_path", latched_qos);
    planned_path_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("/planned_path_marker", latched_qos);
    start_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("/start_marker", latched_qos);
    goal_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("/goal_marker", latched_qos);
    planner_stats_pub_ =
      create_publisher<tgw_planner::msg::PlannerStats>("/planner_stats", latched_qos);
    planner_stats_json_pub_ =
      create_publisher<std_msgs::msg::String>("/planner_stats_json", latched_qos);
    mapping_stats_pub_ = create_publisher<tgw_planner::msg::MappingStats>(
      "/tgw_mapping/stats", latched_qos);
    stats_json_pub_ = create_publisher<std_msgs::msg::String>("/tgw_map/stats_json", latched_qos);

    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "/tgw_mapping/start",
      std::bind(&TgwRealtimeMappingNode::handleStart, this, std::placeholders::_1, std::placeholders::_2));
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "/tgw_mapping/stop",
      std::bind(&TgwRealtimeMappingNode::handleStop, this, std::placeholders::_1, std::placeholders::_2));
    pause_srv_ = create_service<std_srvs::srv::Trigger>(
      "/tgw_mapping/pause",
      std::bind(&TgwRealtimeMappingNode::handlePause, this, std::placeholders::_1, std::placeholders::_2));
    clear_srv_ = create_service<std_srvs::srv::Trigger>(
      "/tgw_mapping/clear",
      std::bind(&TgwRealtimeMappingNode::handleClear, this, std::placeholders::_1, std::placeholders::_2));
    save_map_srv_ = create_service<tgw_planner::srv::SaveMap>(
      "/tgw_mapping/save_map",
      std::bind(&TgwRealtimeMappingNode::handleSaveMap, this, std::placeholders::_1, std::placeholders::_2));
    load_map_srv_ = create_service<tgw_planner::srv::LoadMap>(
      "/tgw_mapping/load_map",
      std::bind(&TgwRealtimeMappingNode::handleLoadMap, this, std::placeholders::_1, std::placeholders::_2));
    export_static_cloud_srv_ = create_service<tgw_planner::srv::ExportStaticCloud>(
      "/tgw_mapping/export_static_pcd",
      std::bind(
        &TgwRealtimeMappingNode::handleExportStaticCloud, this,
        std::placeholders::_1, std::placeholders::_2));
    get_snapshot_srv_ = create_service<tgw_planner::srv::GetSnapshot>(
      "/tgw_mapping/get_snapshot",
      std::bind(
        &TgwRealtimeMappingNode::handleGetSnapshot, this,
        std::placeholders::_1, std::placeholders::_2));
    plan_srv_ = create_service<tgw_planner::srv::PlanPath>(
      "/tgw_map/plan_path",
      std::bind(&TgwRealtimeMappingNode::handlePlanPath, this, std::placeholders::_1, std::placeholders::_2));
    plan_compat_srv_ = create_service<tgw_planner::srv::PlanPath>(
      "/plan_path",
      std::bind(&TgwRealtimeMappingNode::handlePlanPath, this, std::placeholders::_1, std::placeholders::_2));
    set_blocked_region_srv_ = create_service<tgw_planner::srv::SetBlockedRegion>(
      "/tgw_map/set_blocked_region",
      std::bind(
        &TgwRealtimeMappingNode::handleSetBlockedRegion, this,
        std::placeholders::_1, std::placeholders::_2));
    set_blocked_region_compat_srv_ = create_service<tgw_planner::srv::SetBlockedRegion>(
      "/nav_map/set_blocked_region",
      std::bind(
        &TgwRealtimeMappingNode::handleSetBlockedRegion, this,
        std::placeholders::_1, std::placeholders::_2));

    publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(std::max(100, publish_period_ms_)),
      std::bind(&TgwRealtimeMappingNode::publishSnapshot, this));

    RCLCPP_INFO(
      get_logger(),
      "[RealtimeMapping] started points_topic=%s pose_topic=%s map_frame=%s use_tf=%s",
      points_topic_.c_str(), pose_topic_.c_str(), map_frame_.c_str(), use_tf_ ? "true" : "false");
  }

private:
  struct BlockedRegion
  {
    Point3 min;
    Point3 max;
    std::string reason;
  };

  MappingOptions mappingOptions()
  {
    MappingOptions options;
    options.resolution_m = declare_parameter<double>("resolution_m", options.resolution_m);
    options.max_range_m = declare_parameter<double>("max_range_m", options.max_range_m);
    options.min_range_m = declare_parameter<double>("min_range_m", options.min_range_m);
    options.p_hit = declare_parameter<double>("p_hit", options.p_hit);
    options.p_miss = declare_parameter<double>("p_miss", options.p_miss);
    options.p_occupied_threshold =
      declare_parameter<double>("p_occupied_threshold", options.p_occupied_threshold);
    options.p_free_threshold = declare_parameter<double>("p_free_threshold", options.p_free_threshold);
    options.min_static_hits = declare_parameter<int>("min_static_hits", options.min_static_hits);
    options.min_distinct_views =
      declare_parameter<int>("min_distinct_views", options.min_distinct_views);
    options.min_static_lifetime_sec =
      declare_parameter<double>("min_static_lifetime_sec", options.min_static_lifetime_sec);
    options.dynamic_clear_ratio_threshold =
      declare_parameter<double>("dynamic_clear_ratio_threshold", options.dynamic_clear_ratio_threshold);
    options.enable_self_filter = declare_parameter<bool>("enable_self_filter", options.enable_self_filter);
    options.enable_dynamic_filter =
      declare_parameter<bool>("enable_dynamic_filter", options.enable_dynamic_filter);
    return options;
  }

  SelfFilterBox selfFilterBox()
  {
    SelfFilterBox box;
    box.enabled = declare_parameter<bool>("enable_body_self_filter", box.enabled);
    box.min_x = declare_parameter<double>("self_filter_min_x", box.min_x);
    box.max_x = declare_parameter<double>("self_filter_max_x", box.max_x);
    box.min_y = declare_parameter<double>("self_filter_min_y", box.min_y);
    box.max_y = declare_parameter<double>("self_filter_max_y", box.max_y);
    box.min_z = declare_parameter<double>("self_filter_min_z", box.min_z);
    box.max_z = declare_parameter<double>("self_filter_max_z", box.max_z);
    return box;
  }

  SelfFilterBox mountSelfFilterBox()
  {
    SelfFilterBox box;
    box.enabled = declare_parameter<bool>("enable_mount_self_filter", false);
    box.min_x = declare_parameter<double>("mount_self_filter_min_x", 0.0);
    box.max_x = declare_parameter<double>("mount_self_filter_max_x", 0.0);
    box.min_y = declare_parameter<double>("mount_self_filter_min_y", 0.0);
    box.max_y = declare_parameter<double>("mount_self_filter_max_y", 0.0);
    box.min_z = declare_parameter<double>("mount_self_filter_min_z", 0.0);
    box.max_z = declare_parameter<double>("mount_self_filter_max_z", 0.0);
    return box;
  }

  SurfaceExtractionOptions surfaceOptions()
  {
    SurfaceExtractionOptions options;
    options.robot_height_m = declare_parameter<double>("robot_height_m", options.robot_height_m);
    options.max_step_height_m = declare_parameter<double>("max_step_height_m", options.max_step_height_m);
    options.min_static_hits = declare_parameter<int>("surface_min_static_hits", options.min_static_hits);
    options.require_static_support =
      declare_parameter<bool>("surface_require_static_support", options.require_static_support);
    options.require_observed_free_space = declare_parameter<bool>(
      "surface_require_observed_free_space", options.require_observed_free_space);
    options.allow_observed_free_bridge = declare_parameter<bool>(
      "surface_allow_observed_free_bridge", options.allow_observed_free_bridge);
    return options;
  }

  SurfacePlannerOptions plannerOptions()
  {
    SurfacePlannerOptions options;
    options.w_clearance = declare_parameter<double>("planner_w_clearance", options.w_clearance);
    options.w_risk = declare_parameter<double>("planner_w_risk", options.w_risk);
    options.w_slope = declare_parameter<double>("planner_w_slope", options.w_slope);
    options.w_turn = declare_parameter<double>("planner_w_turn", options.w_turn);
    options.w_unknown = declare_parameter<double>("planner_w_unknown", options.w_unknown);
    options.max_iterations = static_cast<std::uint32_t>(
      declare_parameter<int>("planner_max_iterations", static_cast<int>(options.max_iterations)));
    options.require_footprint_support =
      declare_parameter<bool>("planner_require_footprint", options.require_footprint_support);
    options.max_step_height_m =
      declare_parameter<double>("planner_max_step_height_m", options.max_step_height_m);
    options.swept_sample_step_m =
      declare_parameter<double>("planner_swept_sample_step_m", options.swept_sample_step_m);
    options.enable_shortcut =
      declare_parameter<bool>("planner_enable_shortcut", options.enable_shortcut);
    options.shortcut_sample_step_m =
      declare_parameter<double>("planner_shortcut_sample_step_m", options.shortcut_sample_step_m);
    options.shortcut_clearance_ratio =
      declare_parameter<double>("planner_shortcut_clearance_ratio", options.shortcut_clearance_ratio);
    options.shortcut_safety_margin_m =
      declare_parameter<double>("planner_shortcut_safety_margin_m", options.shortcut_safety_margin_m);
    return options;
  }

  RiskFieldOptions riskOptions()
  {
    RiskFieldOptions options;
    options.boundary_risk = declare_parameter<double>("risk_boundary", options.boundary_risk);
    options.dropoff_risk = declare_parameter<double>("risk_dropoff", options.dropoff_risk);
    options.wall_risk = declare_parameter<double>("risk_wall", options.wall_risk);
    options.forbidden_risk = declare_parameter<double>("risk_forbidden", options.forbidden_risk);
    options.low_clearance_risk =
      declare_parameter<double>("risk_low_clearance", options.low_clearance_risk);
    options.low_clearance_threshold_m =
      declare_parameter<double>("risk_low_clearance_threshold_m", options.low_clearance_threshold_m);
    return options;
  }

  RobotFootprintOptions footprintOptions()
  {
    RobotFootprintOptions options;
    options.length_m = declare_parameter<double>("robot_length_m", options.length_m);
    options.width_m = declare_parameter<double>("robot_width_m", options.width_m);
    options.base_to_front_m = declare_parameter<double>("base_to_front_m", options.base_to_front_m);
    get_parameter("robot_height_m", options.height_m);
    return options;
  }

  PathValidationOptions validationOptions()
  {
    PathValidationOptions options;
    options.sample_step_m = declare_parameter<double>("validation_sample_step_m", options.sample_step_m);
    options.min_clearance_m =
      declare_parameter<double>("validation_min_clearance_m", options.min_clearance_m);
    options.low_clearance_report_threshold_m = declare_parameter<double>(
      "validation_low_clearance_report_threshold_m", options.low_clearance_report_threshold_m);
    options.max_step_height_m =
      declare_parameter<double>("validation_max_step_height_m", options.max_step_height_m);
    options.require_footprint_support =
      declare_parameter<bool>("validation_require_footprint", options.require_footprint_support);
    return options;
  }

  void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    latest_pose_ = poseFromPoseStamped(*msg);
    latest_pose_valid_ = true;
    latest_pose_stamp_sec_ = stampSeconds(msg->header.stamp);
  }

  void onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    ++received_clouds_;
    if (!mapping_enabled_) {
      return;
    }

    Pose3 sensor_pose;
    if (!resolveSensorPose(*msg, sensor_pose)) {
      ++dropped_clouds_;
      return;
    }

    ScanInput scan;
    scan.sensor_pose_map = sensor_pose;
    scan.stamp_sec = stampSeconds(msg->header.stamp);
    if (scan.stamp_sec <= 0.0) {
      scan.stamp_sec = now().seconds();
    }
    scan.view_id = ++view_id_;
    scan.points_sensor_frame.reserve(
      static_cast<std::size_t>(std::min<int>(max_points_per_scan_, msg->width * msg->height)));

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    int kept = 0;
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      if (kept >= max_points_per_scan_) {
        break;
      }
      scan.points_sensor_frame.push_back({*iter_x, *iter_y, *iter_z});
      ++kept;
    }

    last_raycast_stats_ = integrator_.insertScan(scan, map_);
    ++integrated_clouds_;
  }

  bool resolveSensorPose(const sensor_msgs::msg::PointCloud2 & msg, Pose3 & pose)
  {
    const std::string source_frame =
      sensor_frame_override_.empty() ? msg.header.frame_id : sensor_frame_override_;
    if (use_tf_ && !source_frame.empty()) {
      try {
        const auto transform = tf_buffer_.lookupTransform(
          map_frame_, source_frame, msg.header.stamp, rclcpp::Duration::from_seconds(0.05));
        pose = poseFromTransform(transform);
        return true;
      } catch (const std::exception & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "[RealtimeMapping] TF lookup failed map_frame=%s source_frame=%s: %s",
          map_frame_.c_str(), source_frame.c_str(), error.what());
      }
    }

    if (latest_pose_valid_) {
      pose = latest_pose_;
      return true;
    }

    if (assume_cloud_in_map_frame_ || msg.header.frame_id == map_frame_) {
      pose = Pose3{};
      return true;
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[RealtimeMapping] no sensor pose available; set use_tf=true, publish %s, or set "
      "assume_cloud_in_map_frame=true",
      pose_topic_.c_str());
    return false;
  }

  sensor_msgs::msg::PointCloud2 makeCloud(
    const std::vector<GridIndex> & cells, float intensity, bool clearance_intensity = false,
    const ClearanceField * clearance = nullptr) const
  {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = map_frame_;
    msg.header.stamp = now();
    msg.height = 1;
    msg.width = static_cast<std::uint32_t>(cells.size());
    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(cells.size());
    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_i(msg, "intensity");
    for (const GridIndex & cell : cells) {
      const Point3 point = map_.gridToWorld(cell);
      *iter_x = static_cast<float>(point.x);
      *iter_y = static_cast<float>(point.y);
      *iter_z = static_cast<float>(point.z);
      *iter_i = clearance_intensity && clearance != nullptr ?
        static_cast<float>(clearance->clearanceDistance(cell)) : intensity;
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_i;
    }
    return msg;
  }

  sensor_msgs::msg::PointCloud2 makeRiskCloud(const RiskField & risk) const
  {
    std::vector<GridIndex> cells;
    cells.reserve(risk.risks().size());
    for (const auto & entry : risk.risks()) {
      cells.push_back(entry.first);
    }

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = map_frame_;
    msg.header.stamp = now();
    msg.height = 1;
    msg.width = static_cast<std::uint32_t>(cells.size());
    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(cells.size());
    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_i(msg, "intensity");
    for (const GridIndex & cell : cells) {
      const Point3 point = map_.gridToWorld(cell);
      *iter_x = static_cast<float>(point.x);
      *iter_y = static_cast<float>(point.y);
      *iter_z = static_cast<float>(point.z);
      *iter_i = static_cast<float>(risk.riskCost(cell));
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_i;
    }
    return msg;
  }

  std::vector<GridIndex> setToVector(const std::unordered_set<GridIndex, tgw_planner::core::GridIndexHash> & set) const
  {
    return {set.begin(), set.end()};
  }

  bool pointInsideRegion(const Point3 & point, const BlockedRegion & region) const
  {
    return point.x >= region.min.x && point.x <= region.max.x &&
           point.y >= region.min.y && point.y <= region.max.y &&
           point.z >= region.min.z && point.z <= region.max.z;
  }

  bool regionsIntersect(const BlockedRegion & a, const BlockedRegion & b) const
  {
    return a.min.x <= b.max.x && a.max.x >= b.min.x &&
           a.min.y <= b.max.y && a.max.y >= b.min.y &&
           a.min.z <= b.max.z && a.max.z >= b.min.z;
  }

  BlockedRegion normalizeBlockedRegion(
    const geometry_msgs::msg::Point & min_point,
    const geometry_msgs::msg::Point & max_point,
    const std::string & reason) const
  {
    BlockedRegion region;
    region.min = {
      std::min(min_point.x, max_point.x),
      std::min(min_point.y, max_point.y),
      std::min(min_point.z, max_point.z)};
    region.max = {
      std::max(min_point.x, max_point.x),
      std::max(min_point.y, max_point.y),
      std::max(min_point.z, max_point.z)};
    region.reason = reason;
    return region;
  }

  std::unordered_set<GridIndex, tgw_planner::core::GridIndexHash> collectBlockedCells(
    const SurfaceMap & surface) const
  {
    std::unordered_set<GridIndex, tgw_planner::core::GridIndexHash> blocked;
    if (blocked_regions_.empty()) {
      return blocked;
    }
    for (const auto & entry : surface.surface_cells) {
      const GridIndex & cell = entry.first;
      if (surface.traversable_cells.find(cell) == surface.traversable_cells.end()) {
        continue;
      }
      const Point3 point = map_.gridToWorld(cell);
      for (const BlockedRegion & region : blocked_regions_) {
        if (pointInsideRegion(point, region)) {
          blocked.insert(cell);
          break;
        }
      }
    }
    return blocked;
  }

  void applyBlockedRegions(SurfaceMap & surface) const
  {
    surface.blocked_cells = collectBlockedCells(surface);
    for (const GridIndex & cell : loaded_blocked_cells_) {
      if (surface.traversable_cells.find(cell) != surface.traversable_cells.end()) {
        surface.blocked_cells.insert(cell);
      }
    }
    for (const GridIndex & cell : surface.blocked_cells) {
      surface.traversable_cells.erase(cell);
      surface.forbidden_cells.insert(cell);
    }
    if (!surface.blocked_cells.empty()) {
      surface_extractor_.rebuildBoundaryLayer(surface, map_);
    }
  }

  SurfaceMap extractSurfaceWithBlocked() const
  {
    SurfaceMap surface = surface_extractor_.extract(map_);
    applyBlockedRegions(surface);
    return surface;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr makePclCloud(const std::vector<GridIndex> & cells) const
  {
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    cloud->reserve(cells.size());
    for (const GridIndex & cell : cells) {
      const Point3 point = map_.gridToWorld(cell);
      pcl::PointXYZI pcl_point;
      pcl_point.x = static_cast<float>(point.x);
      pcl_point.y = static_cast<float>(point.y);
      pcl_point.z = static_cast<float>(point.z);
      pcl_point.intensity = map_.probability(cell);
      cloud->push_back(pcl_point);
    }
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
  }

  bool writePcd(
    const std::filesystem::path & path, const std::vector<GridIndex> & cells,
    std::string & message) const
  {
    try {
      const std::filesystem::path parent = path.parent_path();
      if (!parent.empty()) {
        std::filesystem::create_directories(parent);
      }
      if (cells.empty()) {
        std::ofstream out(path);
        if (!out) {
          message = "failed to open empty PCD: " + path.string();
          return false;
        }
        out << "# .PCD v0.7 - Point Cloud Data file format\n";
        out << "VERSION 0.7\n";
        out << "FIELDS x y z intensity\n";
        out << "SIZE 4 4 4 4\n";
        out << "TYPE F F F F\n";
        out << "COUNT 1 1 1 1\n";
        out << "WIDTH 0\n";
        out << "HEIGHT 1\n";
        out << "VIEWPOINT 0 0 0 1 0 0 0\n";
        out << "POINTS 0\n";
        out << "DATA ascii\n";
        return true;
      }
      const auto cloud = makePclCloud(cells);
      if (pcl::io::savePCDFileBinary(path.string(), *cloud) != 0) {
        message = "failed to write PCD: " + path.string();
        return false;
      }
      return true;
    } catch (const std::exception & error) {
      message = error.what();
      return false;
    }
  }

  struct LoadedPcdCells
  {
    std::vector<std::pair<GridIndex, float>> cells;
    bool exists{false};
  };

  struct LoadedVoxelEvidence
  {
    std::vector<std::pair<GridIndex, tgw_planner::core::VoxelState>> voxels;
    bool exists{false};
  };

  LoadedPcdCells loadPcdCells(const std::filesystem::path & path, std::string & message) const
  {
    LoadedPcdCells loaded;
    if (!std::filesystem::exists(path)) {
      return loaded;
    }
    loaded.exists = true;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(path.string(), cloud) != 0) {
      if (pcdDeclaresZeroPoints(path)) {
        return loaded;
      }
      message = "failed to load PCD: " + path.string();
      loaded.exists = false;
      return loaded;
    }
    loaded.cells.reserve(cloud.size());
    for (const auto & point : cloud) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      loaded.cells.emplace_back(
        map_.worldToGrid({point.x, point.y, point.z}),
        std::isfinite(point.intensity) ? point.intensity : 0.5F);
    }
    return loaded;
  }

  bool pcdDeclaresZeroPoints(const std::filesystem::path & path) const
  {
    std::ifstream in(path);
    std::string line;
    while (std::getline(in, line)) {
      if (line == "POINTS 0" || line == "WIDTH 0") {
        return true;
      }
    }
    return false;
  }

  bool writeVoxelEvidenceCsv(const std::filesystem::path & path, std::string & message) const
  {
    try {
      const std::filesystem::path parent = path.parent_path();
      if (!parent.empty()) {
        std::filesystem::create_directories(parent);
      }
      std::ofstream out(path);
      if (!out) {
        message = "failed to open voxel evidence csv: " + path.string();
        return false;
      }
      out << std::fixed << std::setprecision(9);
      out << "x,y,z,log_odds,hit_count,miss_count,ray_pass_count,first_seen_time,"
        "last_seen_time,last_hit_time,last_miss_time,distinct_view_count,last_view_id,"
        "occupied,free,dynamic_suspect,static_candidate\n";
      for (const auto & entry : map_.voxels()) {
        const GridIndex & cell = entry.first;
        const auto & state = entry.second;
        out << cell.x << "," << cell.y << "," << cell.z << ","
          << state.log_odds << ","
          << state.hit_count << ","
          << state.miss_count << ","
          << state.ray_pass_count << ","
          << state.first_seen_time << ","
          << state.last_seen_time << ","
          << state.last_hit_time << ","
          << state.last_miss_time << ","
          << state.distinct_view_count << ","
          << state.last_view_id << ","
          << (state.occupied ? 1 : 0) << ","
          << (state.free ? 1 : 0) << ","
          << (state.dynamic_suspect ? 1 : 0) << ","
          << (state.static_candidate ? 1 : 0) << "\n";
      }
      return true;
    } catch (const std::exception & error) {
      message = error.what();
      return false;
    }
  }

  LoadedVoxelEvidence loadVoxelEvidenceCsv(
    const std::filesystem::path & path, std::string & message) const
  {
    LoadedVoxelEvidence loaded;
    if (!std::filesystem::exists(path)) {
      return loaded;
    }
    loaded.exists = true;
    std::ifstream in(path);
    if (!in) {
      message = "failed to open voxel evidence csv: " + path.string();
      loaded.exists = false;
      return loaded;
    }

    std::string line;
    if (!std::getline(in, line)) {
      return loaded;
    }
    std::size_t line_number = 1U;
    while (std::getline(in, line)) {
      ++line_number;
      if (trim(line).empty()) {
        continue;
      }
      std::replace(line.begin(), line.end(), ',', ' ');
      std::istringstream row(line);
      int x = 0;
      int y = 0;
      int z = 0;
      int occupied = 0;
      int free = 0;
      int dynamic_suspect = 0;
      int static_candidate = 0;
      unsigned int hit_count = 0U;
      unsigned int miss_count = 0U;
      unsigned int ray_pass_count = 0U;
      unsigned int distinct_view_count = 0U;
      tgw_planner::core::VoxelState state;
      if (!(row >> x >> y >> z >>
          state.log_odds >>
          hit_count >>
          miss_count >>
          ray_pass_count >>
          state.first_seen_time >>
          state.last_seen_time >>
          state.last_hit_time >>
          state.last_miss_time >>
          distinct_view_count >>
          state.last_view_id >>
          occupied >>
          free >>
          dynamic_suspect >>
          static_candidate))
      {
        message = "invalid voxel evidence row at " + path.string() + ":" +
          std::to_string(line_number);
        loaded.exists = false;
        loaded.voxels.clear();
        return loaded;
      }
      state.hit_count = static_cast<std::uint16_t>(
        std::min<unsigned int>(hit_count, std::numeric_limits<std::uint16_t>::max()));
      state.miss_count = static_cast<std::uint16_t>(
        std::min<unsigned int>(miss_count, std::numeric_limits<std::uint16_t>::max()));
      state.ray_pass_count = static_cast<std::uint16_t>(
        std::min<unsigned int>(ray_pass_count, std::numeric_limits<std::uint16_t>::max()));
      state.distinct_view_count = static_cast<std::uint16_t>(
        std::min<unsigned int>(distinct_view_count, std::numeric_limits<std::uint16_t>::max()));
      state.occupied = occupied != 0;
      state.free = free != 0;
      state.dynamic_suspect = dynamic_suspect != 0;
      state.static_candidate = static_candidate != 0;
      loaded.voxels.emplace_back(GridIndex{x, y, z}, state);
    }
    return loaded;
  }

  float probabilityToLogOddsForLoad(float probability) const
  {
    const float p = std::clamp(probability, 1.0e-6F, 1.0F - 1.0e-6F);
    return std::log(p / (1.0F - p));
  }

  tgw_planner::core::VoxelState makeLoadedVoxel(
    float probability, bool occupied, bool free, bool static_candidate, bool dynamic_suspect) const
  {
    tgw_planner::core::VoxelState state;
    state.log_odds = probabilityToLogOddsForLoad(probability);
    state.occupied = occupied;
    state.free = free;
    state.static_candidate = static_candidate;
    state.dynamic_suspect = dynamic_suspect;
    state.hit_count = occupied ? static_cast<std::uint16_t>(
      std::max(1, map_.options().min_static_hits)) : 0U;
    state.miss_count = free ? 1U : 0U;
    state.ray_pass_count = free ? 1U : 0U;
    state.distinct_view_count = occupied ? static_cast<std::uint16_t>(
      std::max(1, map_.options().min_distinct_views)) : 0U;
    state.first_seen_time = 0.0;
    state.last_seen_time = std::max(1.0, map_.options().min_static_lifetime_sec);
    state.last_hit_time = occupied ? state.last_seen_time : 0.0;
    state.last_miss_time = free ? state.last_seen_time : 0.0;
    state.last_view_id = 0;
    return state;
  }

  std::uint32_t loadLayer(
    const LoadedPcdCells & loaded, bool occupied, bool free, bool static_candidate,
    bool dynamic_suspect)
  {
    for (const auto & entry : loaded.cells) {
      const float fallback_probability = occupied ? 0.90F : (free ? 0.10F : 0.50F);
      const float probability = std::isfinite(entry.second) ?
        std::clamp(entry.second, 1.0e-4F, 1.0F - 1.0e-4F) : fallback_probability;
      map_.setVoxelState(
        entry.first,
        makeLoadedVoxel(
          probability, occupied, free, static_candidate, dynamic_suspect));
    }
    return static_cast<std::uint32_t>(loaded.cells.size());
  }

  bool writeTextFile(
    const std::filesystem::path & path, const std::string & content,
    std::string & message) const
  {
    try {
      const std::filesystem::path parent = path.parent_path();
      if (!parent.empty()) {
        std::filesystem::create_directories(parent);
      }
      std::ofstream out(path);
      if (!out) {
        message = "failed to open file: " + path.string();
        return false;
      }
      out << content << "\n";
      return true;
    } catch (const std::exception & error) {
      message = error.what();
      return false;
    }
  }

  std::string buildBlockedRegionsYaml() const
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    out << "blocked_regions:\n";
    for (const BlockedRegion & region : blocked_regions_) {
      out << "  - min: [" << region.min.x << ", " << region.min.y << ", " << region.min.z << "]\n";
      out << "    max: [" << region.max.x << ", " << region.max.y << ", " << region.max.z << "]\n";
      out << "    reason: \"" << yamlEscape(region.reason) << "\"\n";
    }
    return out.str();
  }

  std::string buildMapMetadataYaml() const
  {
    const MappingOptions & mapping = map_.options();
    const std::vector<GridIndex> occupied_cells = map_.occupiedVoxels();
    const std::vector<GridIndex> free_cells = map_.freeVoxels();
    const std::vector<GridIndex> static_cells = map_.staticCandidateVoxels();
    const std::vector<GridIndex> dynamic_cells = map_.dynamicSuspectVoxels();
    std::ostringstream out;
    out << std::fixed << std::setprecision(6);
    out << "map_format: tgw_realtime_map_v1\n";
    out << "map_input_mode: realtime_raycast\n";
    out << "voxel_evidence_schema: tgw_voxel_evidence_csv_v1\n";
    out << "map_frame: \"" << yamlEscape(map_frame_) << "\"\n";
    out << "created_at_unix_sec: " <<
      std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count() << "\n";
    out << "resolution_m: " << mapping.resolution_m << "\n";
    out << "max_range_m: " << mapping.max_range_m << "\n";
    out << "min_range_m: " << mapping.min_range_m << "\n";
    out << "p_hit: " << mapping.p_hit << "\n";
    out << "p_miss: " << mapping.p_miss << "\n";
    out << "p_occupied_threshold: " << mapping.p_occupied_threshold << "\n";
    out << "p_free_threshold: " << mapping.p_free_threshold << "\n";
    out << "min_static_hits: " << mapping.min_static_hits << "\n";
    out << "min_distinct_views: " << mapping.min_distinct_views << "\n";
    out << "min_static_lifetime_sec: " << mapping.min_static_lifetime_sec << "\n";
    out << "dynamic_clear_ratio_threshold: " << mapping.dynamic_clear_ratio_threshold << "\n";
    out << "enable_dynamic_filter: " << (mapping.enable_dynamic_filter ? "true" : "false") << "\n";
    out << "voxel_count: " << map_.size() << "\n";
    out << "occupied_voxels: " << occupied_cells.size() << "\n";
    out << "free_voxels: " << free_cells.size() << "\n";
    out << "static_candidate_voxels: " << static_cells.size() << "\n";
    out << "dynamic_suspect_voxels: " << dynamic_cells.size() << "\n";
    out << "blocked_region_count: " << blocked_regions_.size() << "\n";
    return out.str();
  }

  bool parseYamlScalarDouble(
    const std::string & line, const std::string & key, double & value) const
  {
    const std::string pattern = key + ":";
    const auto key_pos = line.find(pattern);
    if (key_pos == std::string::npos) {
      return false;
    }
    std::istringstream in(trim(line.substr(key_pos + pattern.size())));
    return static_cast<bool>(in >> value);
  }

  bool parseYamlScalarString(
    const std::string & line, const std::string & key, std::string & value) const
  {
    const std::string pattern = key + ":";
    const auto key_pos = line.find(pattern);
    if (key_pos == std::string::npos) {
      return false;
    }
    std::string payload = trim(line.substr(key_pos + pattern.size()));
    if (payload.size() >= 2U && payload.front() == '"' && payload.back() == '"') {
      payload = payload.substr(1U, payload.size() - 2U);
    }
    value = payload;
    return true;
  }

  bool validateMapMetadata(
    const std::filesystem::path & path, std::string & message) const
  {
    message.clear();
    if (!std::filesystem::exists(path)) {
      return true;
    }
    std::ifstream in(path);
    if (!in) {
      message = "failed to open map metadata: " + path.string();
      return false;
    }

    bool have_resolution = false;
    double saved_resolution = 0.0;
    std::string map_format;
    std::string map_input_mode;
    std::string line;
    while (std::getline(in, line)) {
      const std::string stripped = trim(line);
      if (stripped.empty()) {
        continue;
      }
      parseYamlScalarString(stripped, "map_format", map_format);
      parseYamlScalarString(stripped, "map_input_mode", map_input_mode);
      if (parseYamlScalarDouble(stripped, "resolution_m", saved_resolution)) {
        have_resolution = true;
      }
    }
    if (map_format != "tgw_realtime_map_v1") {
      message = "unsupported realtime map format in " + path.string() + ": " + map_format;
      return false;
    }
    if (map_input_mode != "realtime_raycast") {
      message = "unsupported map input mode in " + path.string() + ": " + map_input_mode;
      return false;
    }
    if (!have_resolution) {
      message = "map metadata missing resolution_m: " + path.string();
      return false;
    }
    if (std::abs(saved_resolution - map_.resolution()) > 1.0e-9) {
      std::ostringstream out;
      out << std::fixed << std::setprecision(6);
      out << "map resolution mismatch for " << path.string() <<
        ": saved=" << saved_resolution << " current=" << map_.resolution();
      message = out.str();
      return false;
    }
    return true;
  }

  std::string trim(const std::string & value) const
  {
    const auto begin = value.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
      return "";
    }
    const auto end = value.find_last_not_of(" \t\r\n");
    return value.substr(begin, end - begin + 1U);
  }

  bool parseYamlPointLine(const std::string & line, const std::string & key, Point3 & point) const
  {
    const std::string pattern = key + ":";
    const auto key_pos = line.find(pattern);
    if (key_pos == std::string::npos) {
      return false;
    }
    const auto left = line.find('[', key_pos + pattern.size());
    const auto right = line.find(']', left == std::string::npos ? key_pos : left);
    if (left == std::string::npos || right == std::string::npos || right <= left) {
      return false;
    }
    std::string payload = line.substr(left + 1U, right - left - 1U);
    std::replace(payload.begin(), payload.end(), ',', ' ');
    std::istringstream in(payload);
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    if (!(in >> x >> y >> z)) {
      return false;
    }
    point = {x, y, z};
    return true;
  }

  bool parseYamlReasonLine(const std::string & line, std::string & reason) const
  {
    const auto key_pos = line.find("reason:");
    if (key_pos == std::string::npos) {
      return false;
    }
    std::string payload = trim(line.substr(key_pos + 7U));
    if (payload.size() >= 2U && payload.front() == '"' && payload.back() == '"') {
      payload = payload.substr(1U, payload.size() - 2U);
    }
    std::string unescaped;
    unescaped.reserve(payload.size());
    bool escaping = false;
    for (const char ch : payload) {
      if (escaping) {
        unescaped.push_back(ch);
        escaping = false;
      } else if (ch == '\\') {
        escaping = true;
      } else {
        unescaped.push_back(ch);
      }
    }
    if (escaping) {
      unescaped.push_back('\\');
    }
    reason = unescaped;
    return true;
  }

  std::vector<BlockedRegion> loadBlockedRegionsYaml(
    const std::filesystem::path & path, std::string & message) const
  {
    message.clear();
    std::vector<BlockedRegion> regions;
    if (!std::filesystem::exists(path)) {
      return regions;
    }

    std::ifstream in(path);
    if (!in) {
      message = "failed to open blocked regions file: " + path.string();
      return regions;
    }

    BlockedRegion current;
    bool have_min = false;
    bool have_max = false;
    bool have_reason = false;
    auto flush_current = [&]() {
      if (!have_min && !have_max && !have_reason) {
        return;
      }
      if (have_min && have_max) {
        regions.push_back(current);
      }
      current = BlockedRegion{};
      have_min = false;
      have_max = false;
      have_reason = false;
    };

    std::string line;
    std::size_t line_number = 0U;
    while (std::getline(in, line)) {
      ++line_number;
      const std::string stripped = trim(line);
      if (stripped.empty() || stripped == "blocked_regions:") {
        continue;
      }
      if (stripped.rfind("- min:", 0) == 0) {
        flush_current();
        if (!parseYamlPointLine(stripped, "min", current.min)) {
          message = "invalid blocked region min at " + path.string() + ":" +
            std::to_string(line_number);
          regions.clear();
          return regions;
        }
        have_min = true;
        continue;
      }
      if (stripped.rfind("max:", 0) == 0) {
        if (!parseYamlPointLine(stripped, "max", current.max)) {
          message = "invalid blocked region max at " + path.string() + ":" +
            std::to_string(line_number);
          regions.clear();
          return regions;
        }
        have_max = true;
        continue;
      }
      if (stripped.rfind("reason:", 0) == 0) {
        if (!parseYamlReasonLine(stripped, current.reason)) {
          message = "invalid blocked region reason at " + path.string() + ":" +
            std::to_string(line_number);
          regions.clear();
          return regions;
        }
        have_reason = true;
        continue;
      }
      message = "unsupported blocked region line at " + path.string() + ":" +
        std::to_string(line_number);
      regions.clear();
      return regions;
    }
    flush_current();
    return regions;
  }

  std::string yamlEscape(const std::string & value) const
  {
    std::string escaped;
    escaped.reserve(value.size());
    for (const char ch : value) {
      if (ch == '\\' || ch == '"') {
        escaped.push_back('\\');
      }
      escaped.push_back(ch);
    }
    return escaped;
  }

  std::string jsonEscape(const std::string & value) const
  {
    std::string escaped;
    escaped.reserve(value.size());
    for (const char ch : value) {
      if (ch == '\\' || ch == '"') {
        escaped.push_back('\\');
        escaped.push_back(ch);
      } else if (ch == '\n') {
        escaped += "\\n";
      } else if (ch == '\r') {
        escaped += "\\r";
      } else if (ch == '\t') {
        escaped += "\\t";
      } else {
        escaped.push_back(ch);
      }
    }
    return escaped;
  }

  std::string plannerStatsJson(const tgw_planner::msg::PlannerStats & stats) const
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    out << "{";
    out << "\"map_id\":\"" << jsonEscape(stats.map_id) << "\",";
    out << "\"map_input_mode\":\"realtime_raycast\",";
    out << "\"success\":" << (stats.success ? "true" : "false") << ",";
    out << "\"failure_reason\":\"" << jsonEscape(stats.failure_reason) << "\",";
    out << "\"final_path_validated\":" << (stats.final_path_validated ? "true" : "false") << ",";
    out << "\"final_path_fallback_to_raw\":" <<
      (stats.final_path_fallback_to_raw ? "true" : "false") << ",";
    out << "\"final_path_validation_failure\":\"" <<
      jsonEscape(stats.final_path_validation_failure) << "\",";
    out << "\"search_time_ms\":" << stats.search_time_ms << ",";
    out << "\"total_plan_time_ms\":" << stats.total_plan_time_ms << ",";
    out << "\"occupied_cells\":" << stats.occupied_cells << ",";
    out << "\"traversable_cells\":" << stats.traversable_cells << ",";
    out << "\"blocked_cells\":" << stats.blocked_cells << ",";
    out << "\"risk_cells\":" << stats.risk_cells << ",";
    out << "\"expanded_nodes\":" << stats.expanded_nodes << ",";
    out << "\"generated_nodes\":" << stats.generated_nodes << ",";
    out << "\"raw_path_waypoints\":" << stats.raw_path_waypoints << ",";
    out << "\"raw_path_length_m\":" << stats.raw_path_length_m << ",";
    out << "\"postprocess_floor_shortcuts\":" << stats.postprocess_floor_shortcuts << ",";
    out << "\"path_waypoints\":" << stats.path_waypoints << ",";
    out << "\"path_length_m\":" << stats.path_length_m << ",";
    out << "\"path_vertical_gain_m\":" << stats.path_vertical_gain_m << ",";
    out << "\"path_vertical_loss_m\":" << stats.path_vertical_loss_m << ",";
    out << "\"min_path_clearance_m\":" << stats.min_path_clearance_m << ",";
    out << "\"mean_path_clearance_m\":" << stats.mean_path_clearance_m << ",";
    out << "\"clearance_cost_sum\":" << stats.clearance_cost_sum << ",";
    out << "\"unknown_cost_sum\":" << stats.unknown_cost_sum << ",";
    out << "\"risk_cost_sum\":" << stats.risk_cost_sum << ",";
    out << "\"max_path_risk\":" << stats.max_path_risk << ",";
    out << "\"low_clearance_samples\":" << stats.low_clearance_samples << ",";
    out << "\"start_snap_distance_m\":" << stats.start_snap_distance_m << ",";
    out << "\"goal_snap_distance_m\":" << stats.goal_snap_distance_m << ",";
    out << "\"map_resolution_m\":" << stats.map_resolution_m;
    out << "}";
    return out.str();
  }

  void publishPlannerStats(const tgw_planner::msg::PlannerStats & stats) const
  {
    planner_stats_pub_->publish(stats);
    std_msgs::msg::String msg;
    msg.data = plannerStatsJson(stats);
    planner_stats_json_pub_->publish(msg);
  }

  void publishSnapshot()
  {
    if (map_.size() == 0U) {
      publishStatsJson();
      return;
    }

    const SurfaceMap surface = extractSurfaceWithBlocked();
    ClearanceField clearance;
    clearance.compute(surface.traversable_cells, surface.boundary_cells, map_.resolution());
    tgw_planner::core::RiskField risk(risk_options_);
    risk.compute(surface, clearance);

    occupied_pub_->publish(makeCloud(map_.occupiedVoxels(), 1.0F));
    free_pub_->publish(makeCloud(map_.freeVoxels(), 2.0F));
    dynamic_pub_->publish(makeCloud(map_.dynamicSuspectVoxels(), 3.0F));
    static_pub_->publish(makeCloud(map_.staticCandidateVoxels(), 4.0F));

    std::vector<GridIndex> surface_cells;
    surface_cells.reserve(surface.surface_cells.size());
    for (const auto & entry : surface.surface_cells) {
      surface_cells.push_back(entry.first);
    }
    surface_pub_->publish(makeCloud(surface_cells, 5.0F));
    traversable_pub_->publish(makeCloud(setToVector(surface.traversable_cells), 6.0F));
    boundary_pub_->publish(makeCloud(setToVector(surface.boundary_cells), 7.0F));
    dropoff_boundary_pub_->publish(makeCloud(setToVector(surface.dropoff_boundary_cells), 8.0F));
    wall_boundary_pub_->publish(makeCloud(setToVector(surface.wall_boundary_cells), 9.0F));
    clearance_pub_->publish(
      makeCloud(setToVector(surface.traversable_cells), 0.0F, true, &clearance));
    const std::vector<GridIndex> medial_axis =
      clearance.medialAxisCells(medial_axis_min_clearance_m_);
    medial_axis_pub_->publish(makeCloud(medial_axis, 11.0F, true, &clearance));
    risk_pub_->publish(makeRiskCloud(risk));
    blocked_pub_->publish(makeCloud(setToVector(surface.blocked_cells), 12.0F));
    forbidden_pub_->publish(makeCloud(setToVector(surface.forbidden_cells), 10.0F));
    publishStatsJson(&surface, &clearance, &risk, medial_axis.size());
  }

  NavigationSnapshot buildNavigationSnapshot() const
  {
    NavigationSnapshot snapshot;
    snapshot.surface = extractSurfaceWithBlocked();
    const std::vector<GridIndex> free_cells = map_.freeVoxels();
    snapshot.observed_free_cells.insert(free_cells.begin(), free_cells.end());
    snapshot.clearance.compute(
      snapshot.surface.traversable_cells, snapshot.surface.boundary_cells, map_.resolution());
    snapshot.risk = tgw_planner::core::RiskField(risk_options_);
    snapshot.risk.compute(snapshot.surface, snapshot.clearance);
    snapshot.resolution_m = map_.resolution();
    return snapshot;
  }

  bool snapToTraversable(
    const NavigationSnapshot & snapshot, const Point3 & point, const double yaw_rad,
    const bool require_footprint_support, GridIndex & snapped, double & snap_distance_m) const
  {
    const GridIndex seed = map_.worldToGrid(point);
    double best_distance_cells = std::numeric_limits<double>::infinity();
    double best_clearance_m = -std::numeric_limits<double>::infinity();
    bool found = false;
    const int max_radius_cells =
      std::max(1, static_cast<int>(std::ceil(max_snap_distance_m_ / map_.resolution())));
    for (int radius = 0; radius <= max_radius_cells; ++radius) {
      for (int dx = -radius; dx <= radius; ++dx) {
        for (int dy = -radius; dy <= radius; ++dy) {
          for (int dz = -radius; dz <= radius; ++dz) {
            if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != radius) {
              continue;
            }
            const GridIndex candidate{seed.x + dx, seed.y + dy, seed.z + dz};
            if (!isRealtimePoseCellUsable(snapshot, candidate)) {
              continue;
            }
            const Point3 candidate_point = map_.gridToWorld(candidate);
            if (require_footprint_support &&
              !isRealtimeFootprintSupported(snapshot, candidate_point, yaw_rad))
            {
              continue;
            }
            const double distance_cells =
              std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz));
            const double distance_m = distance_cells * map_.resolution();
            if (distance_m > max_snap_distance_m_ + 1.0e-9) {
              continue;
            }
            const double clearance_m = snapshot.clearance.clearanceDistance(candidate);
            if (distance_cells < best_distance_cells ||
              (std::abs(distance_cells - best_distance_cells) <= 1.0e-9 &&
              clearance_m > best_clearance_m))
            {
              best_distance_cells = distance_cells;
              best_clearance_m = clearance_m;
              snapped = candidate;
              found = true;
            }
          }
        }
      }
      if (found) {
        break;
      }
    }
    snap_distance_m = found ? best_distance_cells * map_.resolution() : 0.0;
    return found;
  }

  bool isRealtimePoseCellUsable(
    const NavigationSnapshot & snapshot, const GridIndex & cell) const
  {
    return snapshot.surface.traversable_cells.find(cell) != snapshot.surface.traversable_cells.end() &&
           snapshot.surface.forbidden_cells.find(cell) == snapshot.surface.forbidden_cells.end() &&
           snapshot.surface.blocked_cells.find(cell) == snapshot.surface.blocked_cells.end();
  }

  bool isRealtimeFootprintSupported(
    const NavigationSnapshot & snapshot, const Point3 & point, const double yaw_rad) const
  {
    return footprint_.isSupported(snapshot.surface, point, yaw_rad, snapshot.resolution_m);
  }

  nav_msgs::msg::Path makePathMessage(const std::vector<Point3> & path) const
  {
    nav_msgs::msg::Path msg;
    msg.header.frame_id = map_frame_;
    msg.header.stamp = now();
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

  void publishPathMarker(const std::vector<Point3> & path)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = now();
    marker.ns = "tgw_realtime_path";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.16;
    marker.color.r = 1.0F;
    marker.color.g = 0.55F;
    marker.color.b = 0.0F;
    marker.color.a = 1.0F;
    marker.points.reserve(path.size());
    for (const Point3 & point : path) {
      geometry_msgs::msg::Point ros_point;
      ros_point.x = point.x;
      ros_point.y = point.y;
      ros_point.z = point.z;
      marker.points.push_back(ros_point);
    }
    if (marker.points.size() < 2U) {
      marker.action = visualization_msgs::msg::Marker::DELETE;
    }
    planned_path_marker_pub_->publish(marker);
  }

  void publishPoseMarker(
    const Point3 & point,
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & publisher,
    const std::string & ns, float r, float g, float b)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = point.z;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.32;
    marker.scale.y = 0.32;
    marker.scale.z = 0.32;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0F;
    publisher->publish(marker);
  }

  double verticalGain(const std::vector<Point3> & path) const
  {
    double gain = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
      gain += std::max(0.0, path[i].z - path[i - 1U].z);
    }
    return gain;
  }

  double verticalLoss(const std::vector<Point3> & path) const
  {
    double loss = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
      loss += std::max(0.0, path[i - 1U].z - path[i].z);
    }
    return loss;
  }

  double clearanceCostSum(
    const NavigationSnapshot & snapshot, const std::vector<GridIndex> & cells) const
  {
    double sum = 0.0;
    for (const GridIndex & cell : cells) {
      sum += snapshot.clearance.clearancePenalty(cell);
    }
    return sum;
  }

  double unknownCostSum(
    const NavigationSnapshot & snapshot, const std::vector<GridIndex> & cells) const
  {
    if (snapshot.observed_free_cells.empty()) {
      return 0.0;
    }
    double sum = 0.0;
    for (const GridIndex & cell : cells) {
      if (snapshot.observed_free_cells.find(cell) == snapshot.observed_free_cells.end()) {
        sum += 1.0;
      }
    }
    return sum;
  }

  double riskCostSum(
    const NavigationSnapshot & snapshot, const std::vector<GridIndex> & cells) const
  {
    double sum = 0.0;
    for (const GridIndex & cell : cells) {
      sum += snapshot.risk.riskCost(cell);
    }
    return sum;
  }

  double maxPathRisk(
    const NavigationSnapshot & snapshot, const std::vector<GridIndex> & cells) const
  {
    double max_risk = 0.0;
    for (const GridIndex & cell : cells) {
      max_risk = std::max(max_risk, snapshot.risk.riskCost(cell));
    }
    return max_risk;
  }

  void handlePlanPath(
    const std::shared_ptr<tgw_planner::srv::PlanPath::Request> request,
    std::shared_ptr<tgw_planner::srv::PlanPath::Response> response)
  {
    const auto started = std::chrono::steady_clock::now();
    const NavigationSnapshot snapshot = buildNavigationSnapshot();
    response->stats.stamp = now();
    response->stats.map_id = "realtime_raycast";
    response->stats.map_resolution_m = map_.resolution();
    response->stats.occupied_cells = static_cast<std::uint32_t>(map_.occupiedVoxels().size());
    response->stats.traversable_cells =
      static_cast<std::uint32_t>(snapshot.surface.traversable_cells.size());
    response->stats.blocked_cells = static_cast<std::uint32_t>(snapshot.surface.blocked_cells.size());
    response->stats.risk_cells = static_cast<std::uint32_t>(snapshot.risk.risks().size());

    GridIndex start;
    GridIndex goal;
    const Point3 start_point{
      request->start.pose.position.x, request->start.pose.position.y, request->start.pose.position.z};
    const Point3 goal_point{
      request->goal.pose.position.x, request->goal.pose.position.y, request->goal.pose.position.z};
    const double plan_dx = goal_point.x - start_point.x;
    const double plan_dy = goal_point.y - start_point.y;
    const double snap_yaw = std::hypot(plan_dx, plan_dy) > 1.0e-6 ?
      std::atan2(plan_dy, plan_dx) : yawFromPoseStamped(request->start);
    const bool snap_requires_footprint =
      planner_options_.require_footprint_support || validation_options_.require_footprint_support;

    double endpoint_snap_yaw = snap_yaw;
    bool endpoints_snapped = false;
    bool start_snap_failed = false;
    bool goal_snap_failed = false;
    for (int snap_attempt = 0; snap_attempt < 4; ++snap_attempt) {
      double start_snap_distance = 0.0;
      double goal_snap_distance = 0.0;
      GridIndex candidate_start;
      GridIndex candidate_goal;
      start_snap_failed = !snapToTraversable(
        snapshot, start_point, endpoint_snap_yaw, snap_requires_footprint, candidate_start,
        start_snap_distance);
      if (start_snap_failed) {
        break;
      }
      goal_snap_failed = !snapToTraversable(
        snapshot, goal_point, endpoint_snap_yaw, snap_requires_footprint, candidate_goal,
        goal_snap_distance);
      if (goal_snap_failed) {
        break;
      }

      const Point3 snapped_start_point = map_.gridToWorld(candidate_start);
      const Point3 snapped_goal_point = map_.gridToWorld(candidate_goal);
      const double snapped_dx = snapped_goal_point.x - snapped_start_point.x;
      const double snapped_dy = snapped_goal_point.y - snapped_start_point.y;
      const double snapped_yaw = std::hypot(snapped_dx, snapped_dy) > 1.0e-6 ?
        std::atan2(snapped_dy, snapped_dx) : endpoint_snap_yaw;
      if (!snap_requires_footprint ||
        (isRealtimeFootprintSupported(snapshot, snapped_start_point, snapped_yaw) &&
        isRealtimeFootprintSupported(snapshot, snapped_goal_point, snapped_yaw)))
      {
        start = candidate_start;
        goal = candidate_goal;
        response->stats.start_snap_distance_m = start_snap_distance;
        response->stats.goal_snap_distance_m = goal_snap_distance;
        endpoints_snapped = true;
        break;
      }
      endpoint_snap_yaw = snapped_yaw;
    }

    if (!endpoints_snapped && start_snap_failed) {
      response->success = false;
      response->message = snap_requires_footprint ?
        "failed to snap start to footprint-supported realtime traversable surface" :
        "failed to snap start to realtime traversable surface";
      response->stats.success = false;
      response->stats.failure_reason = response->message;
      publishPathMarker({});
      publishPlannerStats(response->stats);
      return;
    }
    if (!endpoints_snapped && goal_snap_failed) {
      response->success = false;
      response->message = snap_requires_footprint ?
        "failed to snap goal to footprint-supported realtime traversable surface" :
        "failed to snap goal to realtime traversable surface";
      response->stats.success = false;
      response->stats.failure_reason = response->message;
      publishPathMarker({});
      publishPlannerStats(response->stats);
      return;
    }
    if (!endpoints_snapped) {
      response->success = false;
      response->message =
        "failed to snap start/goal to a mutually footprint-supported realtime heading";
      response->stats.success = false;
      response->stats.failure_reason = response->message;
      publishPathMarker({});
      publishPlannerStats(response->stats);
      return;
    }
    publishPoseMarker(map_.gridToWorld(start), start_marker_pub_, "tgw_realtime_start", 0.0F, 0.95F, 1.0F);
    publishPoseMarker(map_.gridToWorld(goal), goal_marker_pub_, "tgw_realtime_goal", 1.0F, 0.32F, 0.0F);

    const auto search_started = std::chrono::steady_clock::now();
    const SurfaceAstarPlanner planner(planner_options_);
    const auto result = planner.plan(snapshot, start, goal);
    const auto search_finished = std::chrono::steady_clock::now();

    response->success = result.success;
    response->message = result.message;
    response->stats.success = result.success;
    response->stats.failure_reason = result.metrics.failure_reason;
    response->stats.search_time_ms =
      std::chrono::duration<double, std::milli>(search_finished - search_started).count();
    response->stats.total_plan_time_ms =
      std::chrono::duration<double, std::milli>(search_finished - started).count();
    response->stats.expanded_nodes = result.metrics.expanded_nodes;
    response->stats.generated_nodes = result.metrics.generated_nodes;
    response->stats.raw_path_waypoints = result.metrics.raw_path_waypoints;
    response->stats.raw_path_length_m = result.metrics.raw_path_length_m;
    response->stats.postprocess_floor_shortcuts = result.metrics.shortcut_count;
    response->stats.path_waypoints = static_cast<std::uint32_t>(result.path.size());
    response->stats.path_length_m = result.metrics.path_length_m;
    response->stats.path_vertical_gain_m = verticalGain(result.path);
    response->stats.path_vertical_loss_m = verticalLoss(result.path);
    response->stats.min_path_clearance_m = result.metrics.min_path_clearance_m;
    response->stats.mean_path_clearance_m = result.metrics.mean_path_clearance_m;
    response->stats.clearance_cost_sum = result.metrics.clearance_cost_sum;
    response->stats.unknown_cost_sum = result.metrics.unknown_cost_sum;
    response->stats.risk_cost_sum = result.metrics.risk_cost_sum;
    response->stats.max_path_risk = result.metrics.max_path_risk;
    response->stats.low_clearance_samples = result.metrics.low_clearance_samples;
    response->stats.final_path_validated = result.metrics.final_path_validated;
    response->stats.final_path_fallback_to_raw = result.metrics.final_path_fallback_to_raw;
    response->stats.final_path_validation_failure = result.metrics.final_path_validation_failure;

    if (!result.success) {
      response->path = makePathMessage(result.path);
      publishPathMarker({});
      publishPlannerStats(response->stats);
      return;
    }

    const PathValidator validator(footprint_, validation_options_);
    const auto validation = validator.validate(snapshot, result.path);
    response->stats.final_path_validated = validation.valid;
    response->stats.final_path_fallback_to_raw = false;
    response->stats.final_path_validation_failure = validation.failure_reason;
    response->stats.min_path_clearance_m = validation.min_clearance_m;
    response->stats.mean_path_clearance_m = validation.mean_clearance_m;
    response->stats.low_clearance_samples = validation.low_clearance_samples;

    if (!validation.valid) {
      if (!result.raw_path.empty() && result.raw_cells != result.cells) {
        const auto raw_validation = validator.validate(snapshot, result.raw_path);
        if (raw_validation.valid) {
          response->success = true;
          response->message =
            "postprocessed path validation failed; fell back to raw surface A* path";
          response->path = makePathMessage(result.raw_path);
          response->stats.success = true;
          response->stats.failure_reason.clear();
          response->stats.final_path_validated = true;
          response->stats.final_path_fallback_to_raw = true;
          response->stats.final_path_validation_failure = validation.failure_reason;
          response->stats.path_waypoints = static_cast<std::uint32_t>(result.raw_path.size());
          response->stats.path_length_m = result.metrics.raw_path_length_m;
          response->stats.path_vertical_gain_m = verticalGain(result.raw_path);
          response->stats.path_vertical_loss_m = verticalLoss(result.raw_path);
          response->stats.min_path_clearance_m = raw_validation.min_clearance_m;
          response->stats.mean_path_clearance_m = raw_validation.mean_clearance_m;
          response->stats.clearance_cost_sum = clearanceCostSum(snapshot, result.raw_cells);
          response->stats.unknown_cost_sum = unknownCostSum(snapshot, result.raw_cells);
          response->stats.risk_cost_sum = riskCostSum(snapshot, result.raw_cells);
          response->stats.max_path_risk = maxPathRisk(snapshot, result.raw_cells);
          response->stats.low_clearance_samples = raw_validation.low_clearance_samples;
          planned_path_pub_->publish(response->path);
          planned_path_compat_pub_->publish(response->path);
          publishPathMarker(result.raw_path);
          publishPlannerStats(response->stats);
          return;
        }
      }
      response->success = false;
      response->message = "final path validation failed: " + validation.failure_reason;
      response->stats.success = false;
      response->stats.failure_reason = response->message;
      response->path = makePathMessage(result.path);
      publishPathMarker({});
      publishPlannerStats(response->stats);
      return;
    }

    response->path = makePathMessage(result.path);
    planned_path_pub_->publish(response->path);
    planned_path_compat_pub_->publish(response->path);
    publishPathMarker(result.path);
    publishPlannerStats(response->stats);
  }

  std::string buildStatsJson(
    const SurfaceMap * surface = nullptr, const ClearanceField * clearance = nullptr,
    const tgw_planner::core::RiskField * risk = nullptr, std::size_t medial_axis_cells = 0U) const
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    out << "{";
    out << "\"map_input_mode\":\"realtime_raycast\",";
    out << "\"mapping_enabled\":" << (mapping_enabled_ ? "true" : "false") << ",";
    out << "\"received_clouds\":" << received_clouds_ << ",";
    out << "\"integrated_clouds\":" << integrated_clouds_ << ",";
    out << "\"dropped_clouds\":" << dropped_clouds_ << ",";
    out << "\"voxel_count\":" << map_.size() << ",";
    out << "\"occupied_voxels\":" << map_.occupiedVoxels().size() << ",";
    out << "\"free_voxels\":" << map_.freeVoxels().size() << ",";
    out << "\"static_candidate_voxels\":" << map_.staticCandidateVoxels().size() << ",";
    out << "\"dynamic_suspect_voxels\":" << map_.dynamicSuspectVoxels().size() << ",";
    out << "\"last_scan_input_points\":" << last_raycast_stats_.input_points << ",";
    out << "\"last_scan_inserted_points\":" << last_raycast_stats_.inserted_points << ",";
    out << "\"last_scan_filtered_invalid\":" << last_raycast_stats_.filtered_invalid << ",";
    out << "\"last_scan_filtered_range\":" << last_raycast_stats_.filtered_range << ",";
    out << "\"last_scan_filtered_self\":" << last_raycast_stats_.filtered_self << ",";
    out << "\"last_scan_hit_updates\":" << last_raycast_stats_.hit_updates << ",";
    out << "\"last_scan_miss_updates\":" << last_raycast_stats_.miss_updates << ",";
    out << "\"last_scan_dynamic_suspect_voxels_after_decay\":" <<
      last_raycast_stats_.dynamic_suspect_voxels_after_decay << ",";
    out << "\"last_scan_static_candidate_voxels_after_decay\":" <<
      last_raycast_stats_.static_candidate_voxels_after_decay;
    if (surface != nullptr) {
      out << ",\"surface_cells\":" << surface->surface_cells.size();
      out << ",\"traversable_cells\":" << surface->traversable_cells.size();
      out << ",\"boundary_cells\":" << surface->boundary_cells.size();
      out << ",\"blocked_region_count\":" << blocked_regions_.size();
      out << ",\"blocked_cells\":" << surface->blocked_cells.size();
      out << ",\"forbidden_cells\":" << surface->forbidden_cells.size();
    }
    if (clearance != nullptr) {
      out << ",\"clearance_cells\":" << clearance->distances().size();
      out << ",\"medial_axis_cells\":" << medial_axis_cells;
    }
    if (risk != nullptr) {
      out << ",\"risk_cells\":" << risk->risks().size();
    }
    out << "}";
    return out.str();
  }

  tgw_planner::msg::MappingStats buildMappingStats(
    const SurfaceMap * surface = nullptr, const ClearanceField * clearance = nullptr,
    const tgw_planner::core::RiskField * risk = nullptr, std::size_t medial_axis_cells = 0U) const
  {
    tgw_planner::msg::MappingStats msg;
    msg.stamp = now();
    msg.map_id = "realtime_raycast";
    msg.map_input_mode = "realtime_raycast";
    msg.mapping_enabled = mapping_enabled_;
    msg.received_clouds = received_clouds_;
    msg.integrated_clouds = integrated_clouds_;
    msg.dropped_clouds = dropped_clouds_;
    msg.voxel_count = map_.size();
    msg.occupied_voxels = map_.occupiedVoxels().size();
    msg.free_voxels = map_.freeVoxels().size();
    msg.static_candidate_voxels = map_.staticCandidateVoxels().size();
    msg.dynamic_suspect_voxels = map_.dynamicSuspectVoxels().size();
    msg.last_scan_input_points = last_raycast_stats_.input_points;
    msg.last_scan_inserted_points = last_raycast_stats_.inserted_points;
    msg.last_scan_filtered_invalid = last_raycast_stats_.filtered_invalid;
    msg.last_scan_filtered_range = last_raycast_stats_.filtered_range;
    msg.last_scan_filtered_self = last_raycast_stats_.filtered_self;
    msg.last_scan_hit_updates = last_raycast_stats_.hit_updates;
    msg.last_scan_miss_updates = last_raycast_stats_.miss_updates;
    msg.last_scan_dynamic_suspect_voxels_after_decay =
      last_raycast_stats_.dynamic_suspect_voxels_after_decay;
    msg.last_scan_static_candidate_voxels_after_decay =
      last_raycast_stats_.static_candidate_voxels_after_decay;
    if (surface != nullptr) {
      msg.surface_cells = surface->surface_cells.size();
      msg.traversable_cells = surface->traversable_cells.size();
      msg.boundary_cells = surface->boundary_cells.size();
      msg.blocked_region_count = blocked_regions_.size();
      msg.blocked_cells = surface->blocked_cells.size();
      msg.forbidden_cells = surface->forbidden_cells.size();
    }
    if (clearance != nullptr) {
      msg.clearance_cells = clearance->distances().size();
      msg.medial_axis_cells = medial_axis_cells;
    }
    if (risk != nullptr) {
      msg.risk_cells = risk->risks().size();
    }
    msg.map_resolution_m = map_.resolution();
    return msg;
  }

  void publishStatsJson(
    const SurfaceMap * surface = nullptr, const ClearanceField * clearance = nullptr,
    const tgw_planner::core::RiskField * risk = nullptr, std::size_t medial_axis_cells = 0U)
  {
    mapping_stats_pub_->publish(buildMappingStats(surface, clearance, risk, medial_axis_cells));
    std_msgs::msg::String msg;
    msg.data = buildStatsJson(surface, clearance, risk, medial_axis_cells);
    stats_json_pub_->publish(msg);
  }

  void handleStart(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    mapping_enabled_ = true;
    response->success = true;
    response->message = "realtime mapping started";
  }

  void handleStop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    mapping_enabled_ = false;
    response->success = true;
    response->message = "realtime mapping stopped";
  }

  void handlePause(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    mapping_enabled_ = false;
    response->success = true;
    response->message = "realtime mapping paused";
  }

  void handleClear(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    map_.clear();
    blocked_regions_.clear();
    loaded_blocked_cells_.clear();
    response->success = true;
    response->message = "realtime map cleared";
    publishStatsJson();
  }

  void handleSetBlockedRegion(
    const std::shared_ptr<tgw_planner::srv::SetBlockedRegion::Request> request,
    std::shared_ptr<tgw_planner::srv::SetBlockedRegion::Response> response)
  {
    const std::string operation = request->operation;
    const SurfaceMap before_surface = extractSurfaceWithBlocked();
    const std::size_t before_count = before_surface.blocked_cells.size();

    if (operation == "clear") {
      blocked_regions_.clear();
      loaded_blocked_cells_.clear();
      response->success = true;
      response->message = "cleared realtime blocked regions";
      response->affected_cells = static_cast<std::uint32_t>(before_count);
      return;
    }

    const BlockedRegion region = normalizeBlockedRegion(request->min, request->max, request->reason);
    if (region.min.x == region.max.x || region.min.y == region.max.y || region.min.z == region.max.z) {
      response->success = false;
      response->message = "blocked region AABB must have non-zero x/y/z size";
      response->affected_cells = 0U;
      return;
    }

    if (operation == "add") {
      blocked_regions_.push_back(region);
      const SurfaceMap after_surface = extractSurfaceWithBlocked();
      response->success = true;
      response->message = "added realtime blocked region";
      response->affected_cells = static_cast<std::uint32_t>(
        after_surface.blocked_cells.size() > before_count ?
        after_surface.blocked_cells.size() - before_count : after_surface.blocked_cells.size());
      return;
    }

    if (operation == "remove") {
      const auto old_size = blocked_regions_.size();
      blocked_regions_.erase(
        std::remove_if(
          blocked_regions_.begin(), blocked_regions_.end(),
          [&](const BlockedRegion & existing) {return regionsIntersect(existing, region);}),
        blocked_regions_.end());
      for (auto it = loaded_blocked_cells_.begin(); it != loaded_blocked_cells_.end(); ) {
        const Point3 point = map_.gridToWorld(*it);
        if (pointInsideRegion(point, region)) {
          it = loaded_blocked_cells_.erase(it);
        } else {
          ++it;
        }
      }
      const SurfaceMap after_surface = extractSurfaceWithBlocked();
      response->success = true;
      response->message =
        "removed " + std::to_string(old_size - blocked_regions_.size()) + " realtime blocked regions";
      response->affected_cells = static_cast<std::uint32_t>(
        before_count > after_surface.blocked_cells.size() ?
        before_count - after_surface.blocked_cells.size() : before_count);
      return;
    }

    response->success = false;
    response->message = "operation must be add, remove, or clear";
    response->affected_cells = 0U;
  }

  void handleExportStaticCloud(
    const std::shared_ptr<tgw_planner::srv::ExportStaticCloud::Request> request,
    std::shared_ptr<tgw_planner::srv::ExportStaticCloud::Response> response)
  {
    const std::filesystem::path output =
      request->output_pcd.empty() ?
      std::filesystem::path{"./tgw_static_candidate_cloud.pcd"} :
      std::filesystem::path{request->output_pcd};
    const std::vector<GridIndex> static_cells = map_.staticCandidateVoxels();
    std::string message;
    if (!writePcd(output, static_cells, message)) {
      response->success = false;
      response->message = message;
      response->saved_points = 0U;
      return;
    }
    response->success = true;
    response->message = "exported static candidate cloud to " + output.string();
    response->saved_points = static_cast<std::uint32_t>(static_cells.size());
  }

  void handleGetSnapshot(
    const std::shared_ptr<tgw_planner::srv::GetSnapshot::Request>,
    std::shared_ptr<tgw_planner::srv::GetSnapshot::Response> response)
  {
    response->success = true;
    response->message = "snapshot built";
    if (map_.size() == 0U) {
      response->stats = buildMappingStats();
      response->stats_json = buildStatsJson();
      return;
    }

    const NavigationSnapshot snapshot = buildNavigationSnapshot();
    const std::vector<GridIndex> medial_axis =
      snapshot.clearance.medialAxisCells(medial_axis_min_clearance_m_);
    response->stats =
      buildMappingStats(&snapshot.surface, &snapshot.clearance, &snapshot.risk, medial_axis.size());
    response->stats_json =
      buildStatsJson(&snapshot.surface, &snapshot.clearance, &snapshot.risk, medial_axis.size());
    response->occupied_points = static_cast<std::uint32_t>(map_.occupiedVoxels().size());
    response->free_points = static_cast<std::uint32_t>(map_.freeVoxels().size());
    response->static_points = static_cast<std::uint32_t>(map_.staticCandidateVoxels().size());
    response->dynamic_points = static_cast<std::uint32_t>(map_.dynamicSuspectVoxels().size());
    response->surface_points = static_cast<std::uint32_t>(snapshot.surface.surface_cells.size());
    response->traversable_points = static_cast<std::uint32_t>(snapshot.surface.traversable_cells.size());
    response->blocked_points = static_cast<std::uint32_t>(snapshot.surface.blocked_cells.size());
    response->forbidden_points = static_cast<std::uint32_t>(snapshot.surface.forbidden_cells.size());
    response->risk_points = static_cast<std::uint32_t>(snapshot.risk.risks().size());
  }

  void handleLoadMap(
    const std::shared_ptr<tgw_planner::srv::LoadMap::Request> request,
    std::shared_ptr<tgw_planner::srv::LoadMap::Response> response)
  {
    const std::filesystem::path input_dir =
      request->input_dir.empty() ? std::filesystem::path{"./tgw_realtime_map"} :
      std::filesystem::path{request->input_dir};
    const std::filesystem::path occupied_path = input_dir / "occupied_cloud.pcd";
    const std::filesystem::path free_path = input_dir / "free_cloud.pcd";
    const std::filesystem::path static_path = input_dir / "static_candidate_cloud.pcd";
    const std::filesystem::path dynamic_path = input_dir / "dynamic_suspect_cloud.pcd";
    const std::filesystem::path blocked_path = input_dir / "blocked_cloud.pcd";
    const std::filesystem::path blocked_regions_path = input_dir / "blocked_regions.yaml";
    const std::filesystem::path metadata_path = input_dir / "metadata.yaml";
    const std::filesystem::path evidence_path = input_dir / "voxel_evidence.csv";

    std::string message;
    if (!validateMapMetadata(metadata_path, message)) {
      response->success = false;
      response->message = message;
      return;
    }
    const LoadedVoxelEvidence voxel_evidence = loadVoxelEvidenceCsv(evidence_path, message);
    if (!message.empty()) {
      response->success = false;
      response->message = message;
      return;
    }
    const LoadedPcdCells occupied_cells =
      voxel_evidence.exists ? LoadedPcdCells{} : loadPcdCells(occupied_path, message);
    if (!message.empty()) {
      response->success = false;
      response->message = message;
      return;
    }
    if (!voxel_evidence.exists && !occupied_cells.exists) {
      response->success = false;
      response->message = "missing required map asset: " + occupied_path.string();
      return;
    }
    const LoadedPcdCells free_cells =
      voxel_evidence.exists ? LoadedPcdCells{} : loadPcdCells(free_path, message);
    const LoadedPcdCells static_cells =
      voxel_evidence.exists ? LoadedPcdCells{} : loadPcdCells(static_path, message);
    const LoadedPcdCells dynamic_cells =
      voxel_evidence.exists ? LoadedPcdCells{} : loadPcdCells(dynamic_path, message);
    const LoadedPcdCells blocked_cells = loadPcdCells(blocked_path, message);
    std::vector<BlockedRegion> loaded_regions = loadBlockedRegionsYaml(blocked_regions_path, message);
    if (!message.empty()) {
      response->success = false;
      response->message = message;
      return;
    }

    map_.clear();
    blocked_regions_ = std::move(loaded_regions);
    loaded_blocked_cells_.clear();
    if (voxel_evidence.exists) {
      for (const auto & entry : voxel_evidence.voxels) {
        map_.setVoxelState(entry.first, entry.second);
      }
      response->free_points = static_cast<std::uint32_t>(map_.freeVoxels().size());
      response->occupied_points = static_cast<std::uint32_t>(map_.occupiedVoxels().size());
      response->static_points = static_cast<std::uint32_t>(map_.staticCandidateVoxels().size());
      response->dynamic_points = static_cast<std::uint32_t>(map_.dynamicSuspectVoxels().size());
    } else {
      response->free_points = loadLayer(free_cells, false, true, false, false);
      response->occupied_points = loadLayer(occupied_cells, true, false, false, false);
      response->static_points = loadLayer(static_cells, true, false, true, false);
      response->dynamic_points = loadLayer(dynamic_cells, true, false, false, true);
    }
    for (const auto & entry : blocked_cells.cells) {
      loaded_blocked_cells_.insert(entry.first);
    }
    response->blocked_points = static_cast<std::uint32_t>(loaded_blocked_cells_.size());
    response->loaded_voxel_evidence = voxel_evidence.exists;
    response->voxel_count = static_cast<std::uint64_t>(map_.size());
    response->success = true;
    response->message = "loaded realtime map package from " + input_dir.string();
    publishSnapshot();
  }

  void handleSaveMap(
    const std::shared_ptr<tgw_planner::srv::SaveMap::Request> request,
    std::shared_ptr<tgw_planner::srv::SaveMap::Response> response)
  {
    const std::filesystem::path output_dir =
      request->output_dir.empty() ? std::filesystem::path{"./tgw_realtime_map"} :
      std::filesystem::path{request->output_dir};
    const std::filesystem::path occupied_path = output_dir / "occupied_cloud.pcd";
    const std::filesystem::path free_path = output_dir / "free_cloud.pcd";
    const std::filesystem::path static_path = output_dir / "static_candidate_cloud.pcd";
    const std::filesystem::path dynamic_path = output_dir / "dynamic_suspect_cloud.pcd";
    const std::filesystem::path blocked_path = output_dir / "blocked_cloud.pcd";
    const std::filesystem::path blocked_regions_path = output_dir / "blocked_regions.yaml";
    const std::filesystem::path metadata_path = output_dir / "metadata.yaml";
    const std::filesystem::path evidence_path = output_dir / "voxel_evidence.csv";
    const std::filesystem::path stats_path = output_dir / "stats.json";

    const std::vector<GridIndex> occupied_cells = map_.occupiedVoxels();
    const std::vector<GridIndex> free_cells = map_.freeVoxels();
    const std::vector<GridIndex> static_cells = map_.staticCandidateVoxels();
    const std::vector<GridIndex> dynamic_cells = map_.dynamicSuspectVoxels();

    std::string message;
    const NavigationSnapshot snapshot = buildNavigationSnapshot();
    const std::vector<GridIndex> blocked_cells = setToVector(snapshot.surface.blocked_cells);

    if (!writePcd(occupied_path, occupied_cells, message) ||
      !writePcd(free_path, free_cells, message) ||
      !writePcd(static_path, static_cells, message) ||
      !writePcd(dynamic_path, dynamic_cells, message) ||
      !writePcd(blocked_path, blocked_cells, message) ||
      !writeVoxelEvidenceCsv(evidence_path, message))
    {
      response->success = false;
      response->message = message;
      return;
    }

    const std::vector<GridIndex> medial_axis =
      snapshot.clearance.medialAxisCells(medial_axis_min_clearance_m_);
    if (!writeTextFile(
        stats_path,
        buildStatsJson(&snapshot.surface, &snapshot.clearance, &snapshot.risk, medial_axis.size()),
        message) ||
      !writeTextFile(metadata_path, buildMapMetadataYaml(), message) ||
      !writeTextFile(blocked_regions_path, buildBlockedRegionsYaml(), message))
    {
      response->success = false;
      response->message = message;
      return;
    }

    response->success = true;
    response->message = "saved realtime map package to " + output_dir.string();
    response->occupied_pcd = occupied_path.string();
    response->free_pcd = free_path.string();
    response->static_pcd = static_path.string();
    response->dynamic_pcd = dynamic_path.string();
    response->blocked_pcd = blocked_path.string();
    response->voxel_evidence_csv = evidence_path.string();
    response->metadata_yaml = metadata_path.string();
    response->blocked_regions_yaml = blocked_regions_path.string();
    response->stats_json = stats_path.string();
    response->occupied_points = static_cast<std::uint32_t>(occupied_cells.size());
    response->free_points = static_cast<std::uint32_t>(free_cells.size());
    response->static_points = static_cast<std::uint32_t>(static_cells.size());
    response->dynamic_points = static_cast<std::uint32_t>(dynamic_cells.size());
    response->blocked_points = static_cast<std::uint32_t>(blocked_cells.size());
    response->voxel_evidence_rows = static_cast<std::uint64_t>(map_.size());
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ProbabilisticVoxelMap map_;
  RaycastIntegrator integrator_;
  SurfaceExtractor surface_extractor_;
  RobotFootprint footprint_;
  RiskFieldOptions risk_options_;
  SurfacePlannerOptions planner_options_;
  PathValidationOptions validation_options_;

  std::string points_topic_;
  std::string pose_topic_;
  std::string map_frame_;
  std::string sensor_frame_override_;
  bool use_tf_{true};
  bool assume_cloud_in_map_frame_{false};
  bool mapping_enabled_{true};
  int max_points_per_scan_{120000};
  int publish_period_ms_{1000};
  double medial_axis_min_clearance_m_{0.20};
  double max_snap_distance_m_{0.75};
  int view_id_{0};
  std::vector<BlockedRegion> blocked_regions_;
  std::unordered_set<GridIndex, tgw_planner::core::GridIndexHash> loaded_blocked_cells_;

  bool latest_pose_valid_{false};
  double latest_pose_stamp_sec_{0.0};
  Pose3 latest_pose_;
  RaycastStats last_raycast_stats_;
  std::uint64_t received_clouds_{0};
  std::uint64_t integrated_clouds_{0};
  std::uint64_t dropped_clouds_{0};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr free_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dynamic_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surface_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr traversable_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr boundary_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dropoff_boundary_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr wall_boundary_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clearance_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr medial_axis_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr risk_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr blocked_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr forbidden_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_compat_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr planned_path_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr start_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
  rclcpp::Publisher<tgw_planner::msg::PlannerStats>::SharedPtr planner_stats_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planner_stats_json_pub_;
  rclcpp::Publisher<tgw_planner::msg::MappingStats>::SharedPtr mapping_stats_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_json_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_srv_;
  rclcpp::Service<tgw_planner::srv::SaveMap>::SharedPtr save_map_srv_;
  rclcpp::Service<tgw_planner::srv::LoadMap>::SharedPtr load_map_srv_;
  rclcpp::Service<tgw_planner::srv::ExportStaticCloud>::SharedPtr export_static_cloud_srv_;
  rclcpp::Service<tgw_planner::srv::GetSnapshot>::SharedPtr get_snapshot_srv_;
  rclcpp::Service<tgw_planner::srv::PlanPath>::SharedPtr plan_srv_;
  rclcpp::Service<tgw_planner::srv::PlanPath>::SharedPtr plan_compat_srv_;
  rclcpp::Service<tgw_planner::srv::SetBlockedRegion>::SharedPtr set_blocked_region_srv_;
  rclcpp::Service<tgw_planner::srv::SetBlockedRegion>::SharedPtr set_blocked_region_compat_srv_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TgwRealtimeMappingNode>());
  rclcpp::shutdown();
  return 0;
}
