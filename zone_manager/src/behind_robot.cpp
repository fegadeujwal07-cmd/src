#include "zone_manager/behind_robot.hpp"

#include <algorithm>
#include <limits>
#include <sstream>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace zone_manager
{
namespace
{
rclcpp::Logger behind_logger()
{
  return rclcpp::get_logger("zone_manager.behind");
}
}  // namespace

std::optional<std::size_t> reserved_zone_index(
  const std::vector<std::string> & reserved_sequence,
  const std::string & zone_id)
{
  if (zone_id.empty()) {
    return std::nullopt;
  }
  for (std::size_t i = 0; i < reserved_sequence.size(); ++i) {
    if (reserved_sequence[i] == zone_id) {
      return i;
    }
  }
  return std::nullopt;
}

std::vector<std::string> select_robots_coming_from_behind(
  const std::string & conflict_zone,
  const std::unordered_map<std::string, std::vector<std::string>> &
  reserved_by_robot)
{
  if (conflict_zone.empty() || reserved_by_robot.size() < 2) {
    return {};
  }

  struct Entry
  {
    std::string robot_id;
    std::size_t index{0};
  };

  std::vector<Entry> entries;
  entries.reserve(reserved_by_robot.size());

  constexpr std::size_t kMissing = std::numeric_limits<std::size_t>::max();
  std::size_t min_idx = kMissing;
  std::size_t max_idx = 0;

  {
    std::ostringstream identified;
    for (const auto & [robot_id, seq] : reserved_by_robot) {
      if (robot_id.empty()) {
        continue;
      }
      if (!identified.str().empty()) {
        identified << ", ";
      }
      identified << robot_id;
    }
    RCLCPP_INFO(
      behind_logger(),
      "[Behind] Robot Identified for conflict zone=%s: [%s]",
      conflict_zone.c_str(), identified.str().c_str());
  }

  for (const auto & [robot_id, seq] : reserved_by_robot) {
    if (robot_id.empty()) {
      continue;
    }
    const auto idx_opt = reserved_zone_index(seq, conflict_zone);
    const std::size_t idx = idx_opt.value_or(kMissing);
    entries.push_back({robot_id, idx});
    min_idx = std::min(min_idx, idx);
    if (idx != kMissing) {
      max_idx = std::max(max_idx, idx);
    } else {
      max_idx = kMissing;  // missing counts as farthest behind
    }
    const std::string idx_str = idx_opt.has_value() ?
      std::to_string(*idx_opt) : std::string("missing(farthest)");
    RCLCPP_INFO(
      behind_logger(),
      "[Behind] %s reserved index for zone=%s is %s",
      robot_id.c_str(),
      conflict_zone.c_str(),
      idx_str.c_str());
  }

  if (entries.size() < 2 || min_idx == kMissing) {
    RCLCPP_INFO(
      behind_logger(),
      "[Behind] No clear behind robot for zone=%s (insufficient index data)",
      conflict_zone.c_str());
    return {};
  }

  // Everyone tied at the same index → no clear "behind" robot.
  if (min_idx == max_idx) {
    RCLCPP_INFO(
      behind_logger(),
      "[Behind] No clear behind robot for zone=%s (all reserved indices tied at %zu)",
      conflict_zone.c_str(), min_idx);
    return {};
  }

  std::vector<std::string> behind;
  for (const auto & e : entries) {
    if (e.index > min_idx) {
      behind.push_back(e.robot_id);
      const std::string idx_str = (e.index == kMissing) ?
        std::string("missing") : std::to_string(e.index);
      RCLCPP_INFO(
        behind_logger(),
        "[Behind] Why single robot pause: selected robot %s is from behind "
        "(reserved index %s > lead index %zu on zone=%s); single robot stop is required.",
        e.robot_id.c_str(),
        idx_str.c_str(),
        min_idx,
        conflict_zone.c_str());
    }
  }
  std::sort(behind.begin(), behind.end());
  return behind;
}

}  // namespace zone_manager
