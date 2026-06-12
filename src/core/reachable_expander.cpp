#include "tgw_planner/core/reachable_expander.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

namespace tgw_planner::core
{
namespace
{
double elapsedMs(const std::chrono::steady_clock::time_point start)
{
  return std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
}

struct QueueItem
{
  GridIndex cell;
  int steps{0};
  double height_m{0.0};
};

struct AnchorHeightRange
{
  std::vector<int> z_values;
};

struct CellOffset
{
  int dx{0};
  int dy{0};
  int dz{0};
};

struct AnchorRowKey
{
  int x{0};
  int z{0};

  bool operator==(const AnchorRowKey & other) const
  {
    return x == other.x && z == other.z;
  }
};

struct AnchorRowKeyHash
{
  std::size_t operator()(const AnchorRowKey & key) const
  {
    std::size_t seed = std::hash<int>{}(key.x);
    seed ^= std::hash<int>{}(key.z) + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

using AnchorColumns = std::unordered_map<GridIndex, AnchorHeightRange, GridIndexHash>;
using AnchorIntervals =
  std::unordered_map<AnchorRowKey, std::vector<std::pair<int, int>>, AnchorRowKeyHash>;
using ComponentMap = std::unordered_map<GridIndex, int, GridIndexHash>;

double fallbackHeight(const GridIndex & cell, double resolution_m)
{
  return (static_cast<double>(cell.z) + 0.5) * resolution_m;
}

double cellHeight(
  const std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> & cells,
  const GridIndex & cell,
  double resolution_m)
{
  const auto it = cells.find(cell);
  if (it == cells.end()) {
    return fallbackHeight(cell, resolution_m);
  }
  return std::isfinite(it->second.height_m) ? it->second.height_m :
         fallbackHeight(cell, resolution_m);
}

bool containsSurfaceCell(
  const std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> & base_cells,
  const std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> & overlay_cells,
  const GridIndex & cell)
{
  return base_cells.find(cell) != base_cells.end() ||
         overlay_cells.find(cell) != overlay_cells.end();
}

bool hasBodyObstruction(
  const std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> & base_cells,
  const std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> & overlay_cells,
  const GridIndex & cell,
  int body_clearance_cells)
{
  for (int dz = 1; dz <= body_clearance_cells; ++dz) {
    if (containsSurfaceCell(base_cells, overlay_cells, {cell.x, cell.y, cell.z + dz})) {
      return true;
    }
  }
  return false;
}

bool isBodyClear(
  const std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> & base_cells,
  const std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> & overlay_cells,
  const SurfaceCell & candidate,
  int body_clearance_cells)
{
  return !candidate.body_obstructed &&
         !hasBodyObstruction(base_cells, overlay_cells, candidate.cell, body_clearance_cells);
}

bool collectHoleFillSupport(
  const std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> & surface_cells,
  const std::unordered_set<GridIndex, GridIndexHash> & traversable_cells,
  const GridIndex & candidate,
  const ReachableExpanderOptions & options,
  double * average_height_m)
{
  int count = 0;
  double min_height = std::numeric_limits<double>::infinity();
  double max_height = -std::numeric_limits<double>::infinity();
  double total_height = 0.0;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      for (int dz = -options.vertical_tolerance_cells; dz <= options.vertical_tolerance_cells;
        ++dz)
      {
        const GridIndex neighbor{candidate.x + dx, candidate.y + dy, candidate.z + dz};
        if (traversable_cells.find(neighbor) == traversable_cells.end()) {
          continue;
        }
        const double height = cellHeight(surface_cells, neighbor, options.resolution_m);
        min_height = std::min(min_height, height);
        max_height = std::max(max_height, height);
        total_height += height;
        ++count;
      }
    }
  }
  if (count < options.min_hole_fill_neighbors) {
    return false;
  }

  if ((max_height - min_height) > options.max_hole_fill_height_spread_m) {
    return false;
  }

  *average_height_m = total_height / static_cast<double>(count);
  return true;
}

GridIndex columnKey(const GridIndex & cell)
{
  return {cell.x, cell.y, 0};
}

AnchorColumns buildAnchorColumns(
  const std::unordered_set<GridIndex, GridIndexHash> & anchors,
  const std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> & cells,
  const ReachableExpanderOptions & options)
{
  AnchorColumns columns;
  const int radius = std::max(0, options.experience_anchor_radius_cells);
  if (anchors.empty()) {
    return columns;
  }

  std::vector<std::pair<int, int>> row_offsets;
  row_offsets.reserve(static_cast<std::size_t>(2 * radius + 1));
  for (int dx = -radius; dx <= radius; ++dx) {
    const int dy_limit = static_cast<int>(
      std::floor(std::sqrt(static_cast<double>(radius * radius - dx * dx))));
    row_offsets.push_back({dx, dy_limit});
  }

  AnchorIntervals intervals;
  intervals.reserve(anchors.size() * row_offsets.size());
  for (const GridIndex & anchor : anchors) {
    (void)cellHeight(cells, anchor, options.resolution_m);
    for (const auto & row_offset : row_offsets) {
      const int dx = row_offset.first;
      const int dy_limit = row_offset.second;
      intervals[{anchor.x + dx, anchor.z}].push_back(
        {anchor.y - dy_limit, anchor.y + dy_limit});
    }
  }

  columns.reserve(intervals.size() * static_cast<std::size_t>(std::max(1, radius)));
  for (auto & entry : intervals) {
    std::vector<std::pair<int, int>> & ranges = entry.second;
    std::sort(ranges.begin(), ranges.end());
    int start = ranges.front().first;
    int end = ranges.front().second;
    const auto flush_range = [&columns, &entry](int range_start, int range_end) {
      for (int y = range_start; y <= range_end; ++y) {
        columns[{entry.first.x, y, 0}].z_values.push_back(entry.first.z);
      }
    };
    for (std::size_t index = 1; index < ranges.size(); ++index) {
      if (ranges[index].first <= end + 1) {
        end = std::max(end, ranges[index].second);
        continue;
      }
      flush_range(start, end);
      start = ranges[index].first;
      end = ranges[index].second;
    }
    flush_range(start, end);
  }

  for (auto & entry : columns) {
    auto & z_values = entry.second.z_values;
    std::sort(z_values.begin(), z_values.end());
    z_values.erase(std::unique(z_values.begin(), z_values.end()), z_values.end());
  }
  return columns;
}

bool isInsideAnchorEnvelope(
  const AnchorColumns & anchors,
  const GridIndex & cell,
  double,
  double height_tolerance_m,
  int vertical_tolerance_cells)
{
  (void)height_tolerance_m;
  const auto it = anchors.find(columnKey(cell));
  if (it == anchors.end()) {
    return false;
  }
  for (const int anchor_z : it->second.z_values) {
    if (std::abs(cell.z - anchor_z) <= vertical_tolerance_cells) {
      return true;
    }
  }
  return false;
}

void labelSupportComponent(
  const std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> & cells,
  const GridIndex & seed,
  int component_id,
  const ReachableExpanderOptions & options,
  ComponentMap & components)
{
  if (components.find(seed) != components.end()) {
    return;
  }
  const auto seed_it = cells.find(seed);
  if (seed_it == cells.end()) {
    return;
  }

  std::queue<GridIndex> queue;
  components[seed] = component_id;
  queue.push(seed);
  while (!queue.empty()) {
    const GridIndex current = queue.front();
    queue.pop();
    const double current_height = cellHeight(cells, current, options.resolution_m);
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -options.vertical_tolerance_cells; dz <= options.vertical_tolerance_cells;
          ++dz)
        {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const GridIndex neighbor{current.x + dx, current.y + dy, current.z + dz};
          if (components.find(neighbor) != components.end()) {
            continue;
          }
          const auto it = cells.find(neighbor);
          if (it == cells.end()) {
            continue;
          }
          if (std::abs(it->second.height_m - current_height) >
            options.max_expansion_step_height_m)
          {
            continue;
          }
          components[neighbor] = component_id;
          queue.push(neighbor);
        }
      }
    }
  }
}

ComponentMap buildAnchoredSupportComponents(
  const std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> & cells,
  const std::unordered_set<GridIndex, GridIndexHash> & observed_seed_cells,
  const ReachableExpanderOptions & options,
  std::size_t * component_count)
{
  ComponentMap components;
  components.reserve(std::min(cells.size(), observed_seed_cells.size() * 128U));
  int next_component = 0;
  for (const GridIndex & seed : observed_seed_cells) {
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -options.vertical_tolerance_cells; dz <= options.vertical_tolerance_cells;
          ++dz)
        {
          const GridIndex start{seed.x + dx, seed.y + dy, seed.z + dz};
          if (components.find(start) != components.end()) {
            continue;
          }
          if (cells.find(start) == cells.end()) {
            continue;
          }
          labelSupportComponent(cells, start, next_component, options, components);
          ++next_component;
        }
      }
    }
  }
  *component_count = static_cast<std::size_t>(next_component);
  return components;
}

int supportComponentForCell(
  const ComponentMap & components,
  const GridIndex & cell,
  const ReachableExpanderOptions & options)
{
  const auto exact_it = components.find(cell);
  if (exact_it != components.end()) {
    return exact_it->second;
  }
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -options.vertical_tolerance_cells; dz <= options.vertical_tolerance_cells;
        ++dz)
      {
        const auto it = components.find({cell.x + dx, cell.y + dy, cell.z + dz});
        if (it != components.end()) {
          return it->second;
        }
      }
    }
  }
  return -1;
}

std::vector<CellOffset> buildExpansionOffsets(int radius, int vertical_tolerance_cells)
{
  std::vector<CellOffset> offsets;
  offsets.reserve(
    static_cast<std::size_t>(2 * radius + 1) *
    static_cast<std::size_t>(2 * radius + 1) *
    static_cast<std::size_t>(2 * vertical_tolerance_cells + 1));
  for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dz = -vertical_tolerance_cells; dz <= vertical_tolerance_cells; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) {
          continue;
        }
        offsets.push_back({dx, dy, dz});
      }
    }
  }
  return offsets;
}

std::vector<CellOffset> buildHoleCandidateOffsets()
{
  std::vector<CellOffset> offsets;
  offsets.reserve(8U);
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      offsets.push_back({dx, dy, 0});
    }
  }
  return offsets;
}

void assignSurfaceLayerIds(
  ReachableExpansionResult & result,
  const ReachableExpanderOptions &)
{
  for (auto & entry : result.surface_cells) {
    SurfaceCell & cell = entry.second;
    if (cell.label == SurfaceLabel::TrajectoryBridge || cell.support_component_id < 0) {
      cell.surface_layer_id = -1;
      continue;
    }
    cell.surface_layer_id = cell.support_component_id;
  }
}

void keepOnlyTraversableSurfaceCells(ReachableExpansionResult & result)
{
  std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> compact;
  compact.reserve(result.traversable_cells.size());
  for (const GridIndex & cell : result.traversable_cells) {
    const auto surface_it = result.surface_cells.find(cell);
    if (surface_it != result.surface_cells.end()) {
      compact.emplace(cell, surface_it->second);
    }
  }
  result.surface_cells = std::move(compact);
}
}  // namespace

ReachableExpander::ReachableExpander(ReachableExpanderOptions options)
: options_(options)
{
  if (options_.resolution_m <= 0.0) {
    options_.resolution_m = 0.10;
  }
  options_.expansion_radius_cells = std::max(0, options_.expansion_radius_cells);
  options_.max_expansion_steps = std::max(0, options_.max_expansion_steps);
  options_.vertical_tolerance_cells = std::max(0, options_.vertical_tolerance_cells);
  options_.max_expansion_step_height_m = std::max(0.0, options_.max_expansion_step_height_m);
  options_.body_clearance_cells = std::max(0, options_.body_clearance_cells);
  if (options_.experience_anchor_radius_cells < 0) {
    options_.experience_anchor_radius_cells =
      options_.expansion_radius_cells * options_.max_expansion_steps;
  }
  options_.experience_anchor_height_tolerance_m =
    std::max(0.0, options_.experience_anchor_height_tolerance_m);
  options_.experience_anchor_vertical_tolerance_cells =
    std::max(0, options_.experience_anchor_vertical_tolerance_cells);
  options_.hole_fill_iterations = std::max(0, options_.hole_fill_iterations);
  options_.min_hole_fill_neighbors = std::max(1, options_.min_hole_fill_neighbors);
  options_.max_hole_fill_height_spread_m =
    std::max(0.0, options_.max_hole_fill_height_spread_m);
}

ReachableExpansionResult ReachableExpander::expand(
  const std::unordered_set<GridIndex, GridIndexHash> & observed_seed_cells,
  const std::unordered_set<GridIndex, GridIndexHash> & bridge_seed_cells,
  const std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> & geometry_cells) const
{
  const std::unordered_map<GridIndex, BridgeCellMetadata, GridIndexHash> bridge_metadata;
  return expand(observed_seed_cells, bridge_seed_cells, bridge_metadata, geometry_cells);
}

ReachableExpansionResult ReachableExpander::expand(
  const std::unordered_set<GridIndex, GridIndexHash> & observed_seed_cells,
  const std::unordered_set<GridIndex, GridIndexHash> & bridge_seed_cells,
  const std::unordered_map<GridIndex, BridgeCellMetadata, GridIndexHash> & bridge_metadata,
  const std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> & geometry_cells) const
{
  ReachableExpansionResult result;
  const std::size_t expected_surface_cells = std::min(
    geometry_cells.size(),
    std::max(
      observed_seed_cells.size() + bridge_seed_cells.size(),
      (geometry_cells.size() * 3U) / 4U));
  result.surface_cells.reserve(expected_surface_cells);
  result.traversable_cells.reserve(expected_surface_cells);
  result.reachability.reserve(expected_surface_cells);
  const std::vector<CellOffset> expansion_offsets = buildExpansionOffsets(
    options_.expansion_radius_cells, options_.vertical_tolerance_cells);
  const std::vector<CellOffset> hole_candidate_offsets = buildHoleCandidateOffsets();
  const auto t_components = std::chrono::steady_clock::now();
  const ComponentMap components = buildAnchoredSupportComponents(
    geometry_cells, observed_seed_cells, options_, &result.support_component_count);
  result.anchored_component_time_ms = elapsedMs(t_components);
  result.anchored_support_component_count = result.support_component_count;

  const auto t_wave = std::chrono::steady_clock::now();
  const auto t_seed_init = std::chrono::steady_clock::now();
  std::queue<QueueItem> queue;
  for (const GridIndex & seed : observed_seed_cells) {
    SurfaceCell & cell = result.surface_cells[seed];
    const auto geometry_it = geometry_cells.find(seed);
    if (geometry_it != geometry_cells.end()) {
      cell = geometry_it->second;
    }
    cell.cell = seed;
    cell.support = {seed.x, seed.y, seed.z - 1};
    cell.label = SurfaceLabel::ReachableSeed;
    cell.reachability = ReachabilityLabel::ProvenReachable;
    cell.support_component_id = supportComponentForCell(components, seed, options_);
    cell.confidence = 1.0;
    if (!std::isfinite(cell.height_m)) {
      cell.height_m = fallbackHeight(seed, options_.resolution_m);
    }
    result.traversable_cells.insert(seed);
    result.reachability[seed] = ReachabilityLabel::ProvenReachable;
    queue.push({seed, 0, cell.height_m});
  }
  result.proven_seed_count = result.traversable_cells.size();
  result.seed_initialization_time_ms = elapsedMs(t_seed_init);
  const auto t_anchor_envelope = std::chrono::steady_clock::now();
  const AnchorColumns anchor_columns = buildAnchorColumns(
    result.traversable_cells, result.surface_cells, options_);
  result.anchor_envelope_time_ms = elapsedMs(t_anchor_envelope);
  std::unordered_set<GridIndex, GridIndexHash> permanently_rejected_expansion_cells;
  permanently_rejected_expansion_cells.reserve(
    std::min(geometry_cells.size(), result.traversable_cells.size() * 64U + 1024U));

  const auto t_frontier = std::chrono::steady_clock::now();
  while (!queue.empty()) {
    const QueueItem current = queue.front();
    queue.pop();
    if (current.steps >= options_.max_expansion_steps) {
      continue;
    }

    for (const CellOffset & offset : expansion_offsets) {
      const GridIndex neighbor{
        current.cell.x + offset.dx,
        current.cell.y + offset.dy,
        current.cell.z + offset.dz};
      if (result.traversable_cells.find(neighbor) != result.traversable_cells.end() ||
        permanently_rejected_expansion_cells.find(neighbor) !=
        permanently_rejected_expansion_cells.end())
      {
        continue;
      }
      const auto geometry_it = geometry_cells.find(neighbor);
      if (geometry_it == geometry_cells.end()) {
        permanently_rejected_expansion_cells.insert(neighbor);
        continue;
      }
      const auto component_it = components.find(neighbor);
      if (component_it == components.end()) {
        ++result.rejected_unanchored_component_cells;
        permanently_rejected_expansion_cells.insert(neighbor);
        continue;
      }
      if (!isInsideAnchorEnvelope(
          anchor_columns, neighbor, geometry_it->second.height_m,
          options_.experience_anchor_height_tolerance_m,
          options_.experience_anchor_vertical_tolerance_cells))
      {
        ++result.anchor_envelope_rejected_count;
        permanently_rejected_expansion_cells.insert(neighbor);
        continue;
      }
      if (!isBodyClear(
          geometry_cells, result.surface_cells, geometry_it->second,
          options_.body_clearance_cells))
      {
        ++result.body_obstructed_rejected_count;
        permanently_rejected_expansion_cells.insert(neighbor);
        continue;
      }
      if (std::abs(geometry_it->second.height_m - current.height_m) >
        options_.max_expansion_step_height_m)
      {
        ++result.rejected_expansion_count;
        continue;
      }
      if (!result.traversable_cells.insert(neighbor).second) {
        continue;
      }
      SurfaceCell & cell = result.surface_cells[neighbor];
      cell = geometry_it->second;
      cell.label = SurfaceLabel::Expanded;
      cell.reachability = ReachabilityLabel::InferredReachable;
      cell.support_component_id = component_it->second;
      cell.confidence = std::max(cell.confidence, 0.5);
      result.reachability[neighbor] = ReachabilityLabel::InferredReachable;
      ++result.inferred_cell_count;
      queue.push({neighbor, current.steps + 1, cell.height_m});
    }
  }
  result.expansion_frontier_time_ms = elapsedMs(t_frontier);
  result.expansion_wave_time_ms = elapsedMs(t_wave);

  const auto t_hole_fill = std::chrono::steady_clock::now();
  if (options_.enable_hole_filling) {
    for (int iteration = 0; iteration < options_.hole_fill_iterations; ++iteration) {
      std::vector<SurfaceCell> additions;
      additions.reserve(result.traversable_cells.size() / 16U + 1U);
      std::unordered_set<GridIndex, GridIndexHash> evaluated_candidates;
      evaluated_candidates.reserve(result.traversable_cells.size() / 2U + 1U);
      for (const GridIndex & cell : result.traversable_cells) {
        for (const CellOffset & offset : hole_candidate_offsets) {
          const GridIndex candidate{cell.x + offset.dx, cell.y + offset.dy, cell.z};
          if (result.traversable_cells.find(candidate) != result.traversable_cells.end() ||
            !evaluated_candidates.insert(candidate).second)
          {
            continue;
          }
          const double candidate_height = fallbackHeight(candidate, options_.resolution_m);
          if (!isInsideAnchorEnvelope(
              anchor_columns, candidate, candidate_height,
              options_.experience_anchor_height_tolerance_m,
              options_.experience_anchor_vertical_tolerance_cells))
          {
            ++result.anchor_envelope_rejected_count;
            continue;
          }
          if (hasBodyObstruction(
              geometry_cells, result.surface_cells, candidate, options_.body_clearance_cells))
          {
            ++result.body_obstructed_rejected_count;
            continue;
          }

          double average_height = 0.0;
          if (!collectHoleFillSupport(
              result.surface_cells, result.traversable_cells, candidate, options_,
              &average_height))
          {
            continue;
          }

          const auto geometry_it = geometry_cells.find(candidate);
          int support_component_id = -1;
          if (geometry_it != geometry_cells.end()) {
            const auto component_it = components.find(candidate);
            if (component_it == components.end()) {
              ++result.rejected_unanchored_component_cells;
              continue;
            }
            support_component_id = component_it->second;
            if (!isBodyClear(
                geometry_cells, result.surface_cells, geometry_it->second,
                options_.body_clearance_cells))
            {
              ++result.body_obstructed_rejected_count;
              continue;
            }
            if (std::abs(geometry_it->second.height_m - average_height) >
              options_.max_hole_fill_height_spread_m)
            {
              continue;
            }
          }

          SurfaceCell filled;
          if (geometry_it != geometry_cells.end()) {
            filled = geometry_it->second;
          }
          filled.cell = candidate;
          filled.support = {candidate.x, candidate.y, candidate.z - 1};
          filled.label = SurfaceLabel::Expanded;
          filled.reachability = ReachabilityLabel::LowConfidenceReachable;
          filled.support_component_id = support_component_id >= 0 ?
            support_component_id : supportComponentForCell(components, candidate, options_);
          filled.height_m = geometry_it == geometry_cells.end() ?
            average_height : geometry_it->second.height_m;
          filled.confidence = std::max(filled.confidence, 0.35);
          filled.hole_filled = true;
          additions.push_back(filled);
        }
      }

      if (additions.empty()) {
        break;
      }
      for (const SurfaceCell & filled : additions) {
        if (!result.traversable_cells.insert(filled.cell).second) {
          continue;
        }
        result.surface_cells[filled.cell] = filled;
        result.reachability[filled.cell] = filled.reachability;
        ++result.inferred_cell_count;
        ++result.hole_filled_count;
      }
    }
  }
  result.hole_fill_time_ms = elapsedMs(t_hole_fill);

  const auto t_layers = std::chrono::steady_clock::now();
  assignSurfaceLayerIds(result, options_);
  result.layer_assignment_time_ms = elapsedMs(t_layers);

  const auto t_bridge = std::chrono::steady_clock::now();
  for (const GridIndex & seed : bridge_seed_cells) {
    if (result.traversable_cells.find(seed) != result.traversable_cells.end()) {
      continue;
    }
    SurfaceCell & cell = result.surface_cells[seed];
    const auto geometry_it = geometry_cells.find(seed);
    if (geometry_it != geometry_cells.end()) {
      cell = geometry_it->second;
    }
    cell.cell = seed;
    cell.support = {seed.x, seed.y, seed.z - 1};
    cell.label = SurfaceLabel::TrajectoryBridge;
    cell.reachability = ReachabilityLabel::LowConfidenceReachable;
    cell.support_component_id = -1;
    const auto metadata_it = bridge_metadata.find(seed);
    if (metadata_it != bridge_metadata.end()) {
      const BridgeCellMetadata & metadata = metadata_it->second;
      cell.bridge_id = metadata.bridge_id;
      cell.bridge_order = metadata.bridge_order;
      cell.bridge_endpoint = metadata.bridge_endpoint;
      cell.height_m = std::isfinite(metadata.height_m) ?
        metadata.height_m : fallbackHeight(seed, options_.resolution_m);
      cell.confidence = std::max(cell.confidence, metadata.confidence);
    } else {
      cell.height_m = fallbackHeight(seed, options_.resolution_m);
    }
    cell.confidence = std::max(cell.confidence, 0.30);
    result.traversable_cells.insert(seed);
    result.reachability[seed] = ReachabilityLabel::LowConfidenceReachable;
    ++result.bridge_seed_count;
  }
  result.bridge_seed_time_ms = elapsedMs(t_bridge);

  const auto t_compact = std::chrono::steady_clock::now();
  keepOnlyTraversableSurfaceCells(result);
  result.compact_time_ms = elapsedMs(t_compact);
  return result;
}

}  // namespace tgw_planner::core
