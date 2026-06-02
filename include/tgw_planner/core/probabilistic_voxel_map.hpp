#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "tgw_planner/core/grid_index.hpp"
#include "tgw_planner/core/mapping_options.hpp"
#include "tgw_planner/core/planning_types.hpp"

namespace tgw_planner::core
{

struct VoxelState
{
  float log_odds{0.0F};

  std::uint16_t hit_count{0};
  std::uint16_t miss_count{0};
  std::uint16_t ray_pass_count{0};

  double first_seen_time{0.0};
  double last_seen_time{0.0};
  double last_hit_time{0.0};
  double last_miss_time{0.0};

  std::uint16_t distinct_view_count{0};
  int last_view_id{-1};

  bool occupied{false};
  bool free{false};
  bool dynamic_suspect{false};
  bool static_candidate{false};
};

class ProbabilisticVoxelMap
{
public:
  explicit ProbabilisticVoxelMap(MappingOptions options = {});

  GridIndex worldToGrid(const Point3 & point) const;
  Point3 gridToWorld(const GridIndex & idx) const;

  void updateHit(const GridIndex & idx, double stamp_sec, int view_id);
  void updateMiss(const GridIndex & idx, double stamp_sec, int view_id);
  void setVoxelState(const GridIndex & idx, const VoxelState & state);

  bool isOccupied(const GridIndex & idx) const;
  bool isFree(const GridIndex & idx) const;
  bool isUnknown(const GridIndex & idx) const;

  float probability(const GridIndex & idx) const;
  const VoxelState * lookup(const GridIndex & idx) const;
  VoxelState * lookupMutable(const GridIndex & idx);

  std::vector<GridIndex> occupiedVoxels() const;
  std::vector<GridIndex> freeVoxels() const;
  std::vector<GridIndex> staticCandidateVoxels() const;
  std::vector<GridIndex> dynamicSuspectVoxels() const;

  void decayDynamic(double now_sec);
  void clear();

  const MappingOptions & options() const;
  double resolution() const;
  std::size_t size() const;

private:
  VoxelState & touch(const GridIndex & idx, double stamp_sec, int view_id);
  void refreshClassification(VoxelState & state) const;

  MappingOptions options_;
  float hit_log_odds_{0.0F};
  float miss_log_odds_{0.0F};
  float occupied_threshold_log_odds_{0.0F};
  float free_threshold_log_odds_{0.0F};
  std::unordered_map<GridIndex, VoxelState, GridIndexHash> voxels_;
};

}  // namespace tgw_planner::core
