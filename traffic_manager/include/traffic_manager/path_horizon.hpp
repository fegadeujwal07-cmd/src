#ifndef TRAFFIC_MANAGER__PATH_HORIZON_HPP_
#define TRAFFIC_MANAGER__PATH_HORIZON_HPP_

#include <cstdint>
#include <limits>
#include <vector>

#include "rmf_fleet_msgs/msg/location.hpp"

namespace traffic_manager
{

/// Waypoints along the planned polyline covering the next `horizon_m` of travel.
struct PathHorizonSlice
{
  /// Contiguous subsequence of the full leg path sent to ECBS (inclusive indices).
  std::vector<rmf_fleet_msgs::msg::Location> waypoints;
  std::size_t first_path_index{0};
  std::size_t last_path_index{0};
  double travel_distance_m{0.0};
  bool valid{false};
};

/// Sum of segment lengths from robot pose, then along path[from_index .. ] until >= horizon_m.
/// Includes every waypoint vertex up to and including the index where the budget is reached.
PathHorizonSlice slice_path_by_travel_distance(
  const std::vector<rmf_fleet_msgs::msg::Location> & path,
  int32_t already_released_through,
  double current_x,
  double current_y,
  double horizon_m);

/// Base ECBS evaluate checks for a pose (inclusive leg indices).
struct PathBaseRange
{
  std::size_t start_index{0};
  std::size_t end_index{0};
  double travel_distance_m{0.0};
  bool valid{false};
};

/// Start at the waypoint nearest (x, y) among indices >= \p min_start_index
/// (clamped), then add segment lengths until >= horizon_m.
/// \p min_start_index prevents U-turn / return-leg poses from snapping the
/// horizon back to an earlier outbound waypoint (which stalled base extension).
/// When \p fix_start_at_min is true (newBaseRequest from awarded tip), start
/// exactly at \p min_start_index — do not nearest-scan ahead (a later return
/// leg near the live pose would skip the corridor and release to destination).
PathBaseRange base_range_from_pose(
  const std::vector<rmf_fleet_msgs::msg::Location> & path,
  double current_x,
  double current_y,
  double horizon_m,
  int32_t min_start_index = 0,
  bool fix_start_at_min = false);

/// Closest waypoint index on the leg polyline to (x, y), searching only indices
/// >= \p min_start_index within a bounded forward window. Honors
/// \p match_tolerance_m (returns -1 if best is farther). Optional
/// \p prefer_near_index + \p heading_rad prefer continuous route tracking.
/// Returns -1 if path empty or no match within tolerance.
int32_t nearest_leg_index(
  const std::vector<rmf_fleet_msgs::msg::Location> & path,
  double x,
  double y,
  double match_tolerance_m = 0.5,
  int32_t min_start_index = 0,
  int32_t prefer_near_index = -1,
  double heading_rad = std::numeric_limits<double>::quiet_NaN(),
  std::size_t max_lookahead = 40);

}  // namespace traffic_manager

#endif  // TRAFFIC_MANAGER__PATH_HORIZON_HPP_
