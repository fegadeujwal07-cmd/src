#ifndef ZONE_MANAGER__POST_REPLAN_HPP_
#define ZONE_MANAGER__POST_REPLAN_HPP_

#include <chrono>
#include <string>
#include <unordered_set>
#include <vector>

namespace zone_manager
{

/// Fingerprint of a robot plan used to detect post-replan updates.
struct PlanSnapshot
{
  std::string current_zone;
  std::vector<std::string> next_zones;
  std::vector<std::string> zone_sequence;
  std::vector<std::string> planned_path;
};

/// Armed wait after /replan_candidate until a new MonitoredTaskDetails plan arrives.
struct PendingReplanWatch
{
  std::string zone_id;
  std::string candidate_robot_id;
  std::vector<std::string> conflicting_robots;
  PlanSnapshot snapshot_at_arm;
  std::chrono::steady_clock::time_point armed_at{};
};

/// True when the zone list that just arrived differs from the one captured
/// when the replan watch was armed.
///
/// Compares zones against zones. is_new_plan_for_candidate() below compares a
/// whole PlanSnapshot, and the two snapshots in play are never the same shape —
/// the arm-time one comes from the world model (zones, never a path) and the
/// incoming one used to come from MonitoredTaskDetails (a path, never zones) —
/// so it answered "new plan" for the first message after arming regardless of
/// whether anything had replanned. Feeding the re-analysis on that answer made
/// it re-decide the conflict against the PRE-replan zones.
bool zones_changed_since_arm(
  const PlanSnapshot & baseline,
  const std::vector<std::string> & incoming_sequence,
  const std::vector<std::string> & incoming_next);


/// True if `incoming` has a usable plan and differs from `baseline`.
bool is_new_plan_for_candidate(
  const PlanSnapshot & baseline,
  const PlanSnapshot & incoming);

/// Conflicting robots that should be resumed (not in the new pause set).
std::vector<std::string> robots_to_resume(
  const std::vector<std::string> & conflicting_robots,
  const std::vector<std::string> & pause_robot_ids);

/// True when watch age exceeds timeout_sec (timeout_sec <= 0 disables).
bool is_replan_watch_timed_out(
  const PendingReplanWatch & watch,
  std::chrono::steady_clock::time_point now,
  double timeout_sec);

}  // namespace zone_manager

#endif  // ZONE_MANAGER__POST_REPLAN_HPP_
