#include <algorithm>
#include <cstddef>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"

#include "tgw_planner/core/n3map_reader.hpp"

namespace
{
using tgw_planner::core::N3MapReadResult;
using tgw_planner::core::N3MapReader;
using tgw_planner::core::N3NavResource;

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
  const N3MapReadResult & result, const std::string & pbstream_path)
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
  json << "}";
  return json.str();
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
    declare_parameter<int>("max_trajectory_points", 200000);

    const auto latched_qos = rclcpp::QoS(1).transient_local().reliable();
    trajectory_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tgw_experience/trajectory_cloud", latched_qos);
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
      publishStats(result, pbstream_path);
      RCLCPP_ERROR(get_logger(), "%s", result.message.c_str());
      return;
    }

    const N3MapReadResult result = reader_.readPbstream(pbstream_path);
    publishStats(result, pbstream_path);
    if (!result.success) {
      RCLCPP_ERROR(
        get_logger(), "pbstream load failed: [%s] %s",
        result.error_code.c_str(), result.message.c_str());
      return;
    }

    publishTrajectory(result.resource);
    RCLCPP_INFO(
      get_logger(),
      "loaded n3map experience resource: keyframes=%zu dense_trajectory=%zu frame=%s body=%s",
      result.resource.keyframes.size(),
      result.resource.dense_trajectory.size(),
      result.resource.map_frame.c_str(),
      result.resource.body_frame.c_str());
  }

  void publishStats(const N3MapReadResult & result, const std::string & pbstream_path)
  {
    std_msgs::msg::String msg;
    msg.data = toStatsJson(result, pbstream_path);
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

  N3MapReader reader_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_json_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TgwExperiencePlannerNode>());
  rclcpp::shutdown();
  return 0;
}
