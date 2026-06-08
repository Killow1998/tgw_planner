#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "tgw_planner/core/planning_types.hpp"

namespace tgw_planner::core
{

struct N3TrajectoryPose
{
  std::uint64_t seq{0};
  double timestamp{0.0};
  Pose3 pose_world_lidar;
};

struct N3KeyframeLite
{
  std::int64_t id{0};
  double timestamp{0.0};
  Pose3 pose_optimized;
  std::vector<PointXYZI> cloud_body;
};

struct N3NavResource
{
  std::string map_frame{"map"};
  std::string body_frame{"body"};
  std::string version;
  std::string dense_trajectory_source;
  bool dense_trajectory_degraded{false};
  bool has_native_dense_trajectory{false};
  bool dense_trajectory_from_keyframe_fallback{false};
  bool nav_cloud_filter_applied{false};
  std::string nav_cloud_filter_policy;
  bool descriptors_recomputed_from_filtered_cloud{false};
  std::uint64_t nav_filter_raw_points{0};
  std::uint64_t nav_filter_kept_points{0};
  std::uint64_t nav_filter_removed_points{0};
  std::vector<N3TrajectoryPose> dense_trajectory;
  std::vector<N3KeyframeLite> keyframes;
};

struct N3MapReadResult
{
  bool success{false};
  std::string error_code;
  std::string message;
  N3NavResource resource;
};

// TGW prefers explicit failure over uncertain recovery.
// The mainline planner does not silently fall back to global_map.pcd or any
// realtime/terrain/stair compatibility path when pbstream intake is unavailable.
class N3MapReader
{
public:
  N3MapReader() = default;

  N3MapReadResult readPbstream(const std::string & pbstream_path) const;
  N3MapReadResult validate(const N3NavResource & resource) const;
};

}  // namespace tgw_planner::core
