#ifndef ZONE_MANAGER__BEHIND_ROBOT_HPP_
#define ZONE_MANAGER__BEHIND_ROBOT_HPP_

#include <cstddef>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace zone_manager
{

/// Index of zone_id in ordered reserved sequence [current, next...].
/// nullopt if zone is not in the sequence.
std::optional<std::size_t> reserved_zone_index(
  const std::vector<std::string> & reserved_sequence,
  const std::string & zone_id);

/// Among robots that reserve conflict_zone, return those coming from behind:
/// they have a strictly larger index of conflict_zone in their reserved sequence
/// than the minimum index among the group (most-advanced robots are not returned).
///
/// Example: A=[1,2,3,4] B=[7,8,9,1], conflict_zone=1 → A idx0, B idx3 → {B}.
///
/// If all share the same index (tie), returns empty (no behind robot).
/// Robots missing conflict_zone in their sequence are treated as farthest (behind).
std::vector<std::string> select_robots_coming_from_behind(
  const std::string & conflict_zone,
  const std::unordered_map<std::string, std::vector<std::string>> &
  reserved_by_robot);

}  // namespace zone_manager

#endif  // ZONE_MANAGER__BEHIND_ROBOT_HPP_
