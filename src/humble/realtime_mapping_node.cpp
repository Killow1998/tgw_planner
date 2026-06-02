#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
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
#include "tgw_planner/core/mapping_options.hpp"
#include "tgw_planner/core/probabilistic_voxel_map.hpp"
#include "tgw_planner/core/raycast_integrator.hpp"
#include "tgw_planner/core/surface_extractor.hpp"

namespace
{
using tgw_planner::core::GridIndex;
using tgw_planner::core::MappingOptions;
using tgw_planner::core::Point3;
using tgw_planner::core::Pose3;
using tgw_planner::core::ProbabilisticVoxelMap;
using tgw_planner::core::RaycastIntegrator;
using tgw_planner::core::RaycastStats;
using tgw_planner::core::ScanInput;
using tgw_planner::core::SelfFilterBox;
using tgw_planner::core::SurfaceExtractionOptions;
using tgw_planner::core::SurfaceExtractor;
using tgw_planner::core::SurfaceMap;
using tgw_planner::core::ClearanceField;

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

    points_topic_ = declare_parameter<std::string>("points_topic", "/tgw_mapping/points");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/tgw_mapping/pose");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    sensor_frame_override_ = declare_parameter<std::string>("sensor_frame", "");
    use_tf_ = declare_parameter<bool>("use_tf", true);
    assume_cloud_in_map_frame_ = declare_parameter<bool>("assume_cloud_in_map_frame", false);
    mapping_enabled_ = declare_parameter<bool>("start_enabled", true);
    max_points_per_scan_ = declare_parameter<int>("max_points_per_scan", 120000);
    publish_period_ms_ = declare_parameter<int>("publish_period_ms", 1000);

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
    forbidden_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/tgw_map/forbidden_cloud", latched_qos);
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

  std::vector<GridIndex> setToVector(const std::unordered_set<GridIndex, tgw_planner::core::GridIndexHash> & set) const
  {
    return {set.begin(), set.end()};
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
    forbidden_pub_->publish(makeCloud(setToVector(surface.forbidden_cells), 10.0F));
    publishStatsJson(&surface, &clearance);
  }

  void publishStatsJson(
    const SurfaceMap * surface = nullptr, const ClearanceField * clearance = nullptr)
  {
    std_msgs::msg::String msg;
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
    }
    out << "}";
    msg.data = out.str();
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

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ProbabilisticVoxelMap map_;
  RaycastIntegrator integrator_;
  SurfaceExtractor surface_extractor_;

  std::string points_topic_;
  std::string pose_topic_;
  std::string map_frame_;
  std::string sensor_frame_override_;
  bool use_tf_{true};
  bool assume_cloud_in_map_frame_{false};
  bool mapping_enabled_{true};
  int max_points_per_scan_{120000};
  int publish_period_ms_{1000};
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
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr forbidden_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_json_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_srv_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TgwRealtimeMappingNode>());
  rclcpp::shutdown();
  return 0;
}
