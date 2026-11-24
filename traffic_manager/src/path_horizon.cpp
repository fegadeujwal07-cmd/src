#include "traffic_manager/path_horizon.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace traffic_manager
{
namespace
{
double segment_length(
  const rmf_fleet_msgs::msg::Location & a,
  const rmf_fleet_msgs::msg::Location & b)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  return std::hypot(dx, dy);
}

std::size_t clamp_min_start(
  const std::vector<rmf_fleet_msgs::msg::Location> & path,
  const int32_t min_start_index)
{
  if (path.empty()) {
    return 0;
  }
  if (min_start_index <= 0) {
    return 0;
  }
  return std::min(
    static_cast<std::size_t>(min_start_index),
    path.size() - 1);
}
}  // namespace

PathHorizonSlice slice_path_by_travel_distance(
  const std::vector<rmf_fleet_msgs::msg::Location> & path,
  const int32_t already_released_through,
  const double current_x,
  const double current_y,
  const double horizon_m)
{
  PathHorizonSlice slice;
  if (path.empty() || horizon_m <= 0.0) {
    return slice;
  }

  std::size_t start_i = 0;
  if (already_released_through >= 0) {
    start_i = static_cast<std::size_t>(already_released_through) + 1;
  }
  if (start_i >= path.size()) {
    slice.valid = true;
    slice.first_path_index = path.size() - 1;
    slice.last_path_index = slice.first_path_index;
    slice.waypoints.push_back(path.back());
    slice.travel_distance_m = 0.0;
    return slice;
  }

  double accumulated =
    std::hypot(path[start_i].x - current_x, path[start_i].y - current_y);

  slice.waypoints.push_back(path[start_i]);
  slice.first_path_index = start_i;
  slice.last_path_index = start_i;
  slice.travel_distance_m = accumulated;

  if (accumulated >= horizon_m) {
    slice.valid = true;
    return slice;
  }

  for (std::size_t i = start_i + 1; i < path.size(); ++i) {
    accumulated += segment_length(path[i - 1], path[i]);
    slice.waypoints.push_back(path[i]);
    slice.last_path_index = i;
    slice.travel_distance_m = accumulated;
    if (accumulated >= horizon_m) {
      break;
    }
  }

  slice.valid = !slice.waypoints.empty();
  return slice;
}

PathBaseRange base_range_from_pose(
  const std::vector<rmf_fleet_msgs::msg::Location> & path,
  const double current_x,
  const double current_y,
  const double horizon_m,
  const int32_t min_start_index,
  const bool fix_start_at_min)
{
  PathBaseRange range;
  if (path.empty()) {
    return range;
  }
  const double budget = horizon_m > 0.0 ? horizon_m : 20.0;
  const std::size_t floor = clamp_min_start(path, min_start_index);

  std::size_t start = floor;
  if (!fix_start_at_min) {
    auto dist2 = [&](std::size_t i) {
        const double dx = path[i].x - current_x;
        const double dy = path[i].y - current_y;
        return dx * dx + dy * dy;
      };
    // Bounded forward scan from floor — avoid snapping to a later loop/return.
    constexpr std::size_t kMaxLookahead = 40;
    const std::size_t end =
      std::min(path.size(), floor + kMaxLookahead);
    double best_d2 = dist2(floor);
    for (std::size_t i = floor + 1; i < end; ++i) {
      const double d2 = dist2(i);
      if (d2 < best_d2) {
        best_d2 = d2;
        start = i;
      }
    }
  }

  range.start_index = start;
  range.end_index = start;
  for (std::size_t i = start + 1; i < path.size(); ++i) {
    range.travel_distance_m += segment_length(path[i - 1], path[i]);
    range.end_index = i;
    if (range.travel_distance_m >= budget) {
      break;
    }
  }
  range.valid = true;
  return range;
}

int32_t nearest_leg_index(
  const std::vector<rmf_fleet_msgs::msg::Location> & path,
  const double x,
  const double y,
  const double match_tolerance_m,
  const int32_t min_start_index,
  const int32_t prefer_near_index,
  const double heading_rad,
  const std::size_t max_lookahead)
{
  if (path.empty()) {
    return -1;
  }
  const std::size_t floor = clamp_min_start(path, min_start_index);
  std::size_t search_start = floor;
  if (prefer_near_index >= 0) {
    search_start = std::max(
      floor, static_cast<std::size_t>(prefer_near_index));
  }
  const std::size_t search_end = std::min(
    path.size(),
    search_start + std::max<std::size_t>(1, max_lookahead));

  int32_t best = -1;
  double best_score = std::numeric_limits<double>::infinity();
  const bool use_heading = std::isfinite(heading_rad);

  for (std::size_t i = search_start; i < search_end; ++i) {
    const double d = std::hypot(path[i].x - x, path[i].y - y);
    if (match_tolerance_m > 0.0 && d > match_tolerance_m) {
      continue;
    }
    double score = d;
    if (use_heading && i + 1 < path.size()) {
      const double dx = path[i + 1].x - path[i].x;
      const double dy = path[i + 1].y - path[i].y;
      if (dx * dx + dy * dy > 1e-6) {
        constexpr double kPi = 3.14159265358979323846;
        const double path_yaw = std::atan2(dy, dx);
        double dyaw = path_yaw - heading_rad;
        while (dyaw > kPi) {
          dyaw -= 2.0 * kPi;
        }
        while (dyaw < -kPi) {
          dyaw += 2.0 * kPi;
        }
        // Prefer continuous forward progress: heading mismatch adds cost.
        score += 0.5 * std::abs(dyaw);
      }
    }
    // Prefer nearer to prefer_near_index when distances are close.
    if (prefer_near_index >= 0) {
      score += 0.01 * std::abs(static_cast<double>(i) - prefer_near_index);
    }
    if (score < best_score) {
      best_score = score;
      best = static_cast<int32_t>(i);
    }
  }

  // If bounded window found nothing within tolerance, fall back to floor-only
  // local check (do not scan the entire remaining loop).
  if (best < 0 && match_tolerance_m > 0.0) {
    const double d = std::hypot(path[floor].x - x, path[floor].y - y);
    if (d <= match_tolerance_m) {
      return static_cast<int32_t>(floor);
    }
    return -1;
  }
  if (best < 0) {
    return static_cast<int32_t>(floor);
  }
  return best;
}

}  // namespace traffic_manager
