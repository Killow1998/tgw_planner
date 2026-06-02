#pragma once

namespace tgw_planner::core
{

struct MappingOptions
{
  double resolution_m{0.10};
  double max_range_m{30.0};
  double min_range_m{0.30};

  double p_hit{0.70};
  double p_miss{0.40};
  double p_occupied_threshold{0.65};
  double p_free_threshold{0.35};
  double log_odds_min{-4.0};
  double log_odds_max{4.0};

  int min_static_hits{3};
  int min_distinct_views{2};
  double min_static_lifetime_sec{1.0};
  double dynamic_clear_ratio_threshold{0.65};

  bool enable_self_filter{true};
  bool enable_dynamic_filter{true};
};

}  // namespace tgw_planner::core
