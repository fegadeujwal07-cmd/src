#ifndef ZONE_MANAGER__REPLAN_PRIORITY_HPP_
#define ZONE_MANAGER__REPLAN_PRIORITY_HPP_

#include <chrono>
#include <optional>
#include <string>
#include <vector>

namespace zone_manager
{

/// Snapshot used to rank robots in a zone conflict (who should be replanned).
struct RobotPriorityFacts
{
  std::string robot_id;
  /// From MonitoredTaskDetails.priority: "high" / "low" / other / empty.
  std::string task_priority;
  /// true = loaded (keep preferred over empty).
  bool loaded{false};
  /// Remaining ETA in minutes if known; nullopt if unknown.
  std::optional<double> eta_minutes;
  /// When this robot first "requested" / joined the conflict context (FIFO).
  std::chrono::steady_clock::time_point first_request_time{};
};

/// Parse ETA strings like "1.20 min" → minutes. Empty / invalid → nullopt.
std::optional<double> parse_eta_minutes(const std::string & eta);

/// Map task priority string → rank (higher = more important to keep).
/// high=2, low=1, empty/unknown=0.
int task_priority_rank(const std::string & priority);

/// True if /fleet_states load_status means loaded.
/// Single source: 4 = loaded, 2 = unloaded (empty); any other value = unloaded.
/// Does not use /Monitored_task_details load_status.
bool is_loaded_status(int16_t fleet_load_status);

/// Among conflicting robots, return the lowest-priority robot_id (replan candidate).
/// Tie-break order (high→low importance to KEEP path):
///   1) task priority  2) loaded  3) smaller ETA  4) later FIFO (first requester loses).
/// Empty input → empty string.
std::string select_lowest_priority_robot(const std::vector<RobotPriorityFacts> & robots);

}  // namespace zone_manager

#endif  // ZONE_MANAGER__REPLAN_PRIORITY_HPP_
