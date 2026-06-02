#include "tgw_planner/core/probabilistic_voxel_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tgw_planner::core
{
namespace
{
float probabilityToLogOdds(double probability)
{
  const double p = std::clamp(probability, 1.0e-6, 1.0 - 1.0e-6);
  return static_cast<float>(std::log(p / (1.0 - p)));
}

float logOddsToProbability(float log_odds)
{
  return 1.0F / (1.0F + std::exp(-log_odds));
}

void saturatingIncrement(std::uint16_t & value)
{
  if (value < std::numeric_limits<std::uint16_t>::max()) {
    ++value;
  }
}

double clearEvidenceRatio(const VoxelState & state)
{
  const double clear_evidence = static_cast<double>(state.miss_count);
  const double total_evidence =
    clear_evidence + static_cast<double>(state.hit_count);
  if (total_evidence <= 0.0) {
    return 0.0;
  }
  return clear_evidence / total_evidence;
}
}  // namespace

ProbabilisticVoxelMap::ProbabilisticVoxelMap(MappingOptions options)
: options_(options)
{
  options_.resolution_m = std::max(0.01, options_.resolution_m);
  hit_log_odds_ = probabilityToLogOdds(options_.p_hit);
  miss_log_odds_ = probabilityToLogOdds(options_.p_miss);
  occupied_threshold_log_odds_ = probabilityToLogOdds(options_.p_occupied_threshold);
  free_threshold_log_odds_ = probabilityToLogOdds(options_.p_free_threshold);
}

GridIndex ProbabilisticVoxelMap::worldToGrid(const Point3 & point) const
{
  return {
    static_cast<int>(std::floor(point.x / options_.resolution_m)),
    static_cast<int>(std::floor(point.y / options_.resolution_m)),
    static_cast<int>(std::floor(point.z / options_.resolution_m))};
}

Point3 ProbabilisticVoxelMap::gridToWorld(const GridIndex & idx) const
{
  return {
    (static_cast<double>(idx.x) + 0.5) * options_.resolution_m,
    (static_cast<double>(idx.y) + 0.5) * options_.resolution_m,
    (static_cast<double>(idx.z) + 0.5) * options_.resolution_m};
}

void ProbabilisticVoxelMap::updateHit(const GridIndex & idx, double stamp_sec, int view_id)
{
  VoxelState & state = touch(idx, stamp_sec, view_id);
  state.log_odds = std::clamp(
    state.log_odds + hit_log_odds_,
    static_cast<float>(options_.log_odds_min), static_cast<float>(options_.log_odds_max));
  saturatingIncrement(state.hit_count);
  state.last_hit_time = stamp_sec;
  refreshClassification(state);
}

void ProbabilisticVoxelMap::updateMiss(const GridIndex & idx, double stamp_sec, int view_id)
{
  VoxelState & state = touch(idx, stamp_sec, view_id);
  state.log_odds = std::clamp(
    state.log_odds + miss_log_odds_,
    static_cast<float>(options_.log_odds_min), static_cast<float>(options_.log_odds_max));
  saturatingIncrement(state.miss_count);
  saturatingIncrement(state.ray_pass_count);
  state.last_miss_time = stamp_sec;
  refreshClassification(state);
}

void ProbabilisticVoxelMap::setVoxelState(const GridIndex & idx, const VoxelState & state)
{
  voxels_[idx] = state;
}

bool ProbabilisticVoxelMap::isOccupied(const GridIndex & idx) const
{
  const VoxelState * state = lookup(idx);
  return state != nullptr && state->occupied;
}

bool ProbabilisticVoxelMap::isFree(const GridIndex & idx) const
{
  const VoxelState * state = lookup(idx);
  return state != nullptr && state->free;
}

bool ProbabilisticVoxelMap::isUnknown(const GridIndex & idx) const
{
  return lookup(idx) == nullptr;
}

float ProbabilisticVoxelMap::probability(const GridIndex & idx) const
{
  const VoxelState * state = lookup(idx);
  return state == nullptr ? 0.5F : logOddsToProbability(state->log_odds);
}

const VoxelState * ProbabilisticVoxelMap::lookup(const GridIndex & idx) const
{
  const auto it = voxels_.find(idx);
  return it == voxels_.end() ? nullptr : &it->second;
}

VoxelState * ProbabilisticVoxelMap::lookupMutable(const GridIndex & idx)
{
  const auto it = voxels_.find(idx);
  return it == voxels_.end() ? nullptr : &it->second;
}

std::vector<GridIndex> ProbabilisticVoxelMap::occupiedVoxels() const
{
  std::vector<GridIndex> out;
  out.reserve(voxels_.size());
  for (const auto & entry : voxels_) {
    if (entry.second.occupied) {
      out.push_back(entry.first);
    }
  }
  return out;
}

std::vector<GridIndex> ProbabilisticVoxelMap::freeVoxels() const
{
  std::vector<GridIndex> out;
  out.reserve(voxels_.size());
  for (const auto & entry : voxels_) {
    if (entry.second.free) {
      out.push_back(entry.first);
    }
  }
  return out;
}

std::vector<GridIndex> ProbabilisticVoxelMap::staticCandidateVoxels() const
{
  std::vector<GridIndex> out;
  out.reserve(voxels_.size());
  for (const auto & entry : voxels_) {
    if (entry.second.static_candidate) {
      out.push_back(entry.first);
    }
  }
  return out;
}

std::vector<GridIndex> ProbabilisticVoxelMap::dynamicSuspectVoxels() const
{
  std::vector<GridIndex> out;
  out.reserve(voxels_.size());
  for (const auto & entry : voxels_) {
    if (entry.second.dynamic_suspect) {
      out.push_back(entry.first);
    }
  }
  return out;
}

void ProbabilisticVoxelMap::decayDynamic(double now_sec)
{
  if (!options_.enable_dynamic_filter) {
    return;
  }

  for (auto & entry : voxels_) {
    VoxelState & state = entry.second;
    const double lifetime = std::max(0.0, state.last_seen_time - state.first_seen_time);
    const double missed_recently =
      state.last_miss_time > state.last_hit_time && now_sec - state.last_miss_time < 5.0;
    const double clear_ratio = clearEvidenceRatio(state);
    state.dynamic_suspect =
      missed_recently &&
      lifetime < options_.min_static_lifetime_sec &&
      clear_ratio >= options_.dynamic_clear_ratio_threshold &&
      state.hit_count < options_.min_static_hits;
    if (state.dynamic_suspect) {
      state.static_candidate = false;
    }
  }
}

void ProbabilisticVoxelMap::clear()
{
  voxels_.clear();
}

const MappingOptions & ProbabilisticVoxelMap::options() const
{
  return options_;
}

const ProbabilisticVoxelMap::VoxelStorage & ProbabilisticVoxelMap::voxels() const
{
  return voxels_;
}

double ProbabilisticVoxelMap::resolution() const
{
  return options_.resolution_m;
}

std::size_t ProbabilisticVoxelMap::size() const
{
  return voxels_.size();
}

VoxelState & ProbabilisticVoxelMap::touch(const GridIndex & idx, double stamp_sec, int view_id)
{
  VoxelState & state = voxels_[idx];
  if (state.hit_count == 0U && state.miss_count == 0U) {
    state.first_seen_time = stamp_sec;
  }
  state.last_seen_time = stamp_sec;
  if (state.last_view_id != view_id) {
    saturatingIncrement(state.distinct_view_count);
    state.last_view_id = view_id;
  }
  return state;
}

void ProbabilisticVoxelMap::refreshClassification(VoxelState & state) const
{
  state.occupied = state.log_odds >= occupied_threshold_log_odds_;
  state.free = state.log_odds <= free_threshold_log_odds_;

  const double lifetime = std::max(0.0, state.last_seen_time - state.first_seen_time);
  const double clear_ratio = clearEvidenceRatio(state);
  state.dynamic_suspect =
    options_.enable_dynamic_filter &&
    state.hit_count < options_.min_static_hits &&
    clear_ratio >= options_.dynamic_clear_ratio_threshold &&
    state.last_miss_time > state.last_hit_time;

  state.static_candidate =
    state.occupied &&
    !state.dynamic_suspect &&
    state.hit_count >= options_.min_static_hits &&
    state.distinct_view_count >= options_.min_distinct_views &&
    lifetime >= options_.min_static_lifetime_sec;
}

}  // namespace tgw_planner::core
