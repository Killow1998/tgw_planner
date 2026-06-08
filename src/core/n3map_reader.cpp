#include "tgw_planner/core/n3map_reader.hpp"

#include <cstddef>
#include <string>
#include <utility>

#include "n3mapping/n3map_nav_resource_reader.h"

namespace tgw_planner::core
{
namespace
{

Pose3 toTgwPose(const Eigen::Isometry3d & pose)
{
  Pose3 out;
  const Eigen::Vector3d translation = pose.translation();
  out.translation = {translation.x(), translation.y(), translation.z()};
  const Eigen::Matrix3d rotation = pose.rotation();
  out.rotation = {
    rotation(0, 0), rotation(0, 1), rotation(0, 2),
    rotation(1, 0), rotation(1, 1), rotation(1, 2),
    rotation(2, 0), rotation(2, 1), rotation(2, 2)};
  return out;
}

std::string normalizeReaderError(const std::string & error)
{
  if (error == "pbstream_missing_dense_trajectory") {
    return error;
  }
  if (error.find("open") != std::string::npos) {
    return "pbstream_open_failed";
  }
  if (error.find("parse") != std::string::npos) {
    return "pbstream_parse_failed";
  }
  return "pbstream_parse_failed";
}

N3NavResource convertResource(const n3mapping::N3NavResource & n3_resource)
{
  N3NavResource out;
  out.map_frame = n3_resource.map_frame;
  out.body_frame = n3_resource.body_frame;
  out.version = n3_resource.version;
  out.dense_trajectory_source = n3_resource.dense_trajectory_source;
  out.dense_trajectory_degraded = n3_resource.dense_trajectory_degraded;
  out.has_native_dense_trajectory = n3_resource.has_native_dense_trajectory;
  out.dense_trajectory_from_keyframe_fallback =
    n3_resource.dense_trajectory_from_keyframe_fallback;
  out.nav_cloud_filter_applied = n3_resource.nav_cloud_filter_applied;
  out.nav_cloud_filter_policy = n3_resource.nav_cloud_filter_policy;
  out.descriptors_recomputed_from_filtered_cloud =
    n3_resource.descriptors_recomputed_from_filtered_cloud;
  out.nav_filter_raw_points = n3_resource.nav_filter_raw_points;
  out.nav_filter_kept_points = n3_resource.nav_filter_kept_points;
  out.nav_filter_removed_points = n3_resource.nav_filter_removed_points;

  out.keyframes.reserve(n3_resource.keyframes.size());
  for (const auto & keyframe : n3_resource.keyframes) {
    N3KeyframeLite converted;
    converted.id = keyframe.id;
    converted.timestamp = keyframe.timestamp;
    converted.pose_optimized = toTgwPose(keyframe.pose_optimized);
    if (keyframe.cloud) {
      converted.cloud_body.reserve(keyframe.cloud->points.size());
      for (const auto & point : keyframe.cloud->points) {
        converted.cloud_body.push_back({point.x, point.y, point.z, point.intensity});
      }
    }
    out.keyframes.push_back(std::move(converted));
  }

  out.dense_trajectory.reserve(n3_resource.dense_optimized_trajectory.size());
  for (const auto & pose : n3_resource.dense_optimized_trajectory) {
    N3TrajectoryPose converted;
    converted.seq = pose.seq;
    converted.timestamp = pose.timestamp;
    converted.pose_world_lidar = toTgwPose(pose.pose_world_lidar);
    out.dense_trajectory.push_back(converted);
  }
  return out;
}

}  // namespace

N3MapReadResult N3MapReader::readPbstream(const std::string & pbstream_path) const
{
  n3mapping::N3NavReaderOptions options;
  options.allow_keyframe_fallback = false;

  n3mapping::N3NavResource n3_resource;
  std::string error;
  if (!n3mapping::readN3NavResource(pbstream_path, options, &n3_resource, &error)) {
    N3MapReadResult result;
    result.error_code = normalizeReaderError(error);
    result.message = error.empty() ? result.error_code : error;
    return result;
  }

  return validate(convertResource(n3_resource));
}

N3MapReadResult N3MapReader::validate(const N3NavResource & resource) const
{
  N3MapReadResult result;
  result.resource = resource;
  if (resource.map_frame.empty() || resource.body_frame.empty()) {
    result.error_code = "pbstream_invalid_frame_contract";
    result.message = "map_frame and body_frame metadata are required";
    return result;
  }
  if (resource.dense_trajectory.empty()) {
    result.error_code = "pbstream_missing_dense_trajectory";
    result.message = "dense optimized lidar/body trajectory is required";
    return result;
  }
  if (resource.dense_trajectory_from_keyframe_fallback) {
    result.error_code = "pbstream_dense_trajectory_from_fallback_not_allowed";
    result.message = "keyframe fallback dense trajectory is not allowed";
    return result;
  }
  if (resource.dense_trajectory_degraded) {
    result.error_code = "pbstream_dense_trajectory_degraded_not_allowed";
    result.message = "degraded dense trajectory is not allowed";
    return result;
  }
  if (!resource.has_native_dense_trajectory) {
    result.error_code = "pbstream_missing_dense_trajectory";
    result.message = "native dense optimized lidar/body trajectory is required";
    return result;
  }
  if (resource.keyframes.empty()) {
    result.error_code = "pbstream_missing_keyframes";
    result.message = "keyframe clouds are required for conservative expansion";
    return result;
  }
  for (std::size_t i = 0; i < resource.keyframes.size(); ++i) {
    if (resource.keyframes[i].cloud_body.empty()) {
      result.error_code = "pbstream_missing_keyframe_clouds";
      result.message = "keyframe " + std::to_string(resource.keyframes[i].id) +
        " has no cloud_body points";
      return result;
    }
  }
  result.success = true;
  return result;
}

}  // namespace tgw_planner::core
