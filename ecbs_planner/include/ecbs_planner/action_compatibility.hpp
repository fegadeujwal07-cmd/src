#ifndef ECBS_PLANNER__ACTION_COMPATIBILITY_HPP_
#define ECBS_PLANNER__ACTION_COMPATIBILITY_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "ecbs_planner/passage_action.hpp"
#include "ecbs_planner/types.hpp"

namespace ecbs_planner
{

struct CompatibilitySnapshot
{
  std::unordered_map<std::string, TimedPath> paths;
  /// Live + pending reserved intervals: amr → (from_idx, to_idx).
  std::unordered_map<std::string, std::pair<int32_t, int32_t>> reserved_intervals;
  double footprint_radius_m{0.6};
  double load_length_m{1.2};
  /// Same-conflict following allowed when rear trails front by this gap.
  double following_gap_m{2.5};
};

/// Geometric movement compatibility. Conflict IDs are NOT exclusivity proofs.
bool actions_compatible(
  const PassageAction & a,
  const PassageAction & b,
  const CompatibilitySnapshot & snapshot);

/// Interval + footprint check used by TM provisional reservation.
bool intervals_compatible(
  const TimedPath & path_a,
  int32_t a_from, int32_t a_to,
  const TimedPath & path_b,
  int32_t b_from, int32_t b_to,
  double footprint_radius_m,
  double load_length_m);

}  // namespace ecbs_planner

#endif  // ECBS_PLANNER__ACTION_COMPATIBILITY_HPP_
