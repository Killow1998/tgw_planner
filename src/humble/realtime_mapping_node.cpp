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
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
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
#include "tgw_planner/msg/planner_stats.hpp"
#include "tgw_planner/srv/export_static_cloud.hpp"
#include "tgw_planner/srv/plan_path.hpp"
#include "tgw_planner/srv/save_map.hpp"

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
    integrator_ = RaycastIntegrator(mapping_options, selfFilterBox());
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
    forbidden_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/tgw_map/forbidden_cloud", latched_qos);
    planned_path_pub_ = create_publisher<nav_msgs::msg::Path>("/tgw_map/planned_path", latched_qos);
    stats_json_pub_ = create_publisher<std_msgs::msg::String>("/tgw_map/stats_json", latched_qos);

    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "/tgw_mapping/start",
      std::bind(&TgwRealtimeMappingNode::handleStart, this, std::placeholders::_1, std::placeholders::_2));
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "/tgw_mapping/stop",
      std::bind(&TgwRealtimeMappingNode::handleStop, this, std::placeholders::_1, std::placeholders::_2));
    clear_srv_ = create_service<std_srvs::srv::Trigger>(
      "/tgw_mapping/clear",
      std::bind(&TgwRealtimeMappingNode::handleClear, this, std::placeholders::_1, std::placeholders::_2));
    save_map_srv_ = create_service<tgw_planner::srv::SaveMap>(
      "/tgw_mapping/save_map",
      std::bind(&TgwRealtimeMappingNode::handleSaveMap, this, std::placeholders::_1, std::placeholders::_2));
    export_static_cloud_srv_ = create_service<tgw_planner::srv::ExportStaticCloud>(
      "/tgw_mapping/export_static_pcd",
      std::bind(
        &TgwRealtimeMappingNode::handleExportStaticCloud, this,
        std::placeholders::_1, std::placeholders::_2));
    plan_srv_ = create_service<tgw_planner::srv::PlanPath>(
      "/tgw_map/plan_path",
      std::bind(&TgwRealtimeMappingNode::handlePlanPath, this, std::placeholders::_1, std::placeholders::_2));

    publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(std::max(100, publish_period_ms_)),
      std::bind(&TgwRealtimeMappingNode::publishSnapshot, this));

    RCLCPP_INFO(
      get_logger(),
      "[RealtimeMapping] started points_topic=%s pose_topic=%s map_frame=%s use_tf=%s",
      points_topic_.c_str(), pose_topic_.c_str(), map_frame_.c_str(), use_tf_ ? "true" : "false");
  }

private:
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
    options.enable_self_filter = declare_parameter<bool>("enable_self_filter", options.enable_self_filter);
    options.enable_dynamic_filter =
      declare_parameter<bool>("enable_dynamic_filter", options.enable_dynamic_filter);
    return options;
  }

  SelfFilterBox selfFilterBox()
  {
    SelfFilterBox box;
    box.min_x = declare_parameter<double>("self_filter_min_x", box.min_x);
    box.max_x = declare_parameter<double>("self_filter_max_x", box.max_x);
    box.min_y = declare_parameter<double>("self_filter_min_y", box.min_y);
    box.max_y = declare_parameter<double>("self_filter_max_y", box.max_y);
    box.min_z = declare_parameter<double>("self_filter_min_z", box.min_z);
    box.max_z = declare_parameter<double>("self_filter_max_z", box.max_z);
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
    options.swept_sample_step_m =
      declare_parameter<double>("planner_swept_sample_step_m", options.swept_sample_step_m);
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

  void publishSnapshot()
  {
    if (map_.size() == 0U) {
      publishStatsJson();
      return;
    }

    const SurfaceMap surface = surface_extractor_.extract(map_);
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
    forbidden_pub_->publish(makeCloud(setToVector(surface.forbidden_cells), 10.0F));
    publishStatsJson(&surface, &clearance, &risk, medial_axis.size());
  }

  NavigationSnapshot buildNavigationSnapshot() const
  {
    NavigationSnapshot snapshot;
    snapshot.surface = surface_extractor_.extract(map_);
    snapshot.clearance.compute(
      snapshot.surface.traversable_cells, snapshot.surface.boundary_cells, map_.resolution());
    snapshot.risk = tgw_planner::core::RiskField(risk_options_);
    snapshot.risk.compute(snapshot.surface, snapshot.clearance);
    snapshot.resolution_m = map_.resolution();
    return snapshot;
  }

  bool snapToTraversable(
    const NavigationSnapshot & snapshot, const Point3 & point, GridIndex & snapped,
    double & snap_distance_m) const
  {
    const GridIndex seed = map_.worldToGrid(point);
    double best_distance_cells = std::numeric_limits<double>::infinity();
    bool found = false;
    const int max_radius_cells = std::max(1, static_cast<int>(std::ceil(1.0 / map_.resolution())));
    for (int radius = 0; radius <= max_radius_cells; ++radius) {
      for (int dx = -radius; dx <= radius; ++dx) {
        for (int dy = -radius; dy <= radius; ++dy) {
          for (int dz = -radius; dz <= radius; ++dz) {
            if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != radius) {
              continue;
            }
            const GridIndex candidate{seed.x + dx, seed.y + dy, seed.z + dz};
            if (snapshot.surface.traversable_cells.find(candidate) ==
              snapshot.surface.traversable_cells.end())
            {
              continue;
            }
            if (snapshot.surface.forbidden_cells.find(candidate) !=
              snapshot.surface.forbidden_cells.end())
            {
              continue;
            }
            const double distance_cells =
              std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz));
            if (distance_cells < best_distance_cells) {
              best_distance_cells = distance_cells;
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
    response->stats.blocked_cells = static_cast<std::uint32_t>(snapshot.surface.forbidden_cells.size());
    response->stats.risk_cells = static_cast<std::uint32_t>(snapshot.surface.boundary_cells.size());

    GridIndex start;
    GridIndex goal;
    const Point3 start_point{
      request->start.pose.position.x, request->start.pose.position.y, request->start.pose.position.z};
    const Point3 goal_point{
      request->goal.pose.position.x, request->goal.pose.position.y, request->goal.pose.position.z};
    if (!snapToTraversable(snapshot, start_point, start, response->stats.start_snap_distance_m)) {
      response->success = false;
      response->message = "failed to snap start to realtime traversable surface";
      response->stats.success = false;
      response->stats.failure_reason = response->message;
      return;
    }
    if (!snapToTraversable(snapshot, goal_point, goal, response->stats.goal_snap_distance_m)) {
      response->success = false;
      response->message = "failed to snap goal to realtime traversable surface";
      response->stats.success = false;
      response->stats.failure_reason = response->message;
      return;
    }

    const auto search_started = std::chrono::steady_clock::now();
    const SurfaceAstarPlanner planner(planner_options_);
    const auto result = planner.plan(snapshot, start, goal);
    const auto search_finished = std::chrono::steady_clock::now();

    response->success = result.success;
    response->message = result.message;
    response->path = makePathMessage(result.path);
    response->stats.success = result.success;
    response->stats.failure_reason = result.metrics.failure_reason;
    response->stats.search_time_ms =
      std::chrono::duration<double, std::milli>(search_finished - search_started).count();
    response->stats.total_plan_time_ms =
      std::chrono::duration<double, std::milli>(search_finished - started).count();
    response->stats.expanded_nodes = result.metrics.expanded_nodes;
    response->stats.generated_nodes = result.metrics.generated_nodes;
    response->stats.path_waypoints = static_cast<std::uint32_t>(result.path.size());
    response->stats.path_length_m = result.metrics.path_length_m;
    response->stats.path_vertical_gain_m = verticalGain(result.path);
    response->stats.path_vertical_loss_m = verticalLoss(result.path);
    response->stats.min_path_clearance_m = result.metrics.min_path_clearance_m;
    response->stats.mean_path_clearance_m = result.metrics.mean_path_clearance_m;
    response->stats.low_clearance_samples = result.metrics.low_clearance_samples;

    if (!result.success) {
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
      response->success = false;
      response->message = "final path validation failed: " + validation.failure_reason;
      response->stats.success = false;
      response->stats.failure_reason = response->message;
      return;
    }

    planned_path_pub_->publish(response->path);
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
    out << "\"last_scan_miss_updates\":" << last_raycast_stats_.miss_updates;
    if (surface != nullptr) {
      out << ",\"surface_cells\":" << surface->surface_cells.size();
      out << ",\"traversable_cells\":" << surface->traversable_cells.size();
      out << ",\"boundary_cells\":" << surface->boundary_cells.size();
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

  void publishStatsJson(
    const SurfaceMap * surface = nullptr, const ClearanceField * clearance = nullptr,
    const tgw_planner::core::RiskField * risk = nullptr, std::size_t medial_axis_cells = 0U)
  {
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

  void handleClear(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    map_.clear();
    response->success = true;
    response->message = "realtime map cleared";
    publishStatsJson();
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
    const std::filesystem::path stats_path = output_dir / "stats.json";

    const std::vector<GridIndex> occupied_cells = map_.occupiedVoxels();
    const std::vector<GridIndex> free_cells = map_.freeVoxels();
    const std::vector<GridIndex> static_cells = map_.staticCandidateVoxels();
    const std::vector<GridIndex> dynamic_cells = map_.dynamicSuspectVoxels();

    std::string message;
    if (!writePcd(occupied_path, occupied_cells, message) ||
      !writePcd(free_path, free_cells, message) ||
      !writePcd(static_path, static_cells, message) ||
      !writePcd(dynamic_path, dynamic_cells, message))
    {
      response->success = false;
      response->message = message;
      return;
    }

    const NavigationSnapshot snapshot = buildNavigationSnapshot();
    const std::vector<GridIndex> medial_axis =
      snapshot.clearance.medialAxisCells(medial_axis_min_clearance_m_);
    if (!writeTextFile(
        stats_path,
        buildStatsJson(&snapshot.surface, &snapshot.clearance, &snapshot.risk, medial_axis.size()),
        message))
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
    response->stats_json = stats_path.string();
    response->occupied_points = static_cast<std::uint32_t>(occupied_cells.size());
    response->free_points = static_cast<std::uint32_t>(free_cells.size());
    response->static_points = static_cast<std::uint32_t>(static_cells.size());
    response->dynamic_points = static_cast<std::uint32_t>(dynamic_cells.size());
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
  int view_id_{0};

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
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr forbidden_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_json_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_srv_;
  rclcpp::Service<tgw_planner::srv::SaveMap>::SharedPtr save_map_srv_;
  rclcpp::Service<tgw_planner::srv::ExportStaticCloud>::SharedPtr export_static_cloud_srv_;
  rclcpp::Service<tgw_planner::srv::PlanPath>::SharedPtr plan_srv_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TgwRealtimeMappingNode>());
  rclcpp::shutdown();
  return 0;
}
