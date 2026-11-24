#include "ecbs_planner/action_compatibility.hpp"

#include <algorithm>
#include <cmath>

#include "ecbs_planner/conflict_store.hpp"

namespace ecbs_planner
{
namespace
{

bool segment_near(
  double ax, double ay, double bx, double by,
  double cx, double cy, double dx, double dy,
  double eps)
{
  return segments_conflict(ax, ay, bx, by, cx, cy, dx, dy, eps);
}

}  // namespace

bool intervals_compatible(
  const TimedPath & path_a,
  const int32_t a_from, const int32_t a_to,
  const TimedPath & path_b,
  const int32_t b_from, const int32_t b_to,
  const double footprint_radius_m,
  const double load_length_m)
{
  if (path_a.waypoints.empty() || path_b.waypoints.empty()) {
    return true;
  }
  if (a_from > a_to || b_from > b_to) {
    return true;
  }
  const double eps = footprint_radius_m + 0.5 * load_length_m;
  const int32_t a0 = std::max(0, a_from);
  const int32_t a1 = std::min(
    a_to, static_cast<int32_t>(path_a.waypoints.size()) - 1);
  const int32_t b0 = std::max(0, b_from);
  const int32_t b1 = std::min(
    b_to, static_cast<int32_t>(path_b.waypoints.size()) - 1);

  for (int32_t i = a0; i < a1; ++i) {
    const auto & a = path_a.waypoints[static_cast<std::size_t>(i)];
    const auto & b = path_a.waypoints[static_cast<std::size_t>(i + 1)];
    for (int32_t j = b0; j < b1; ++j) {
      const auto & c = path_b.waypoints[static_cast<std::size_t>(j)];
      const auto & d = path_b.waypoints[static_cast<std::size_t>(j + 1)];
      if (segment_near(a.x, a.y, b.x, b.y, c.x, c.y, d.x, d.y, eps)) {
        return false;
      }
    }
  }
  // Stopping envelopes: endpoints within footprint.
  const auto & ae = path_a.waypoints[static_cast<std::size_t>(a1)];
  const auto & be = path_b.waypoints[static_cast<std::size_t>(b1)];
  if (std::hypot(ae.x - be.x, ae.y - be.y) <= eps) {
    return false;
  }
  return true;
}

bool actions_compatible(
  const PassageAction & a,
  const PassageAction & b,
  const CompatibilitySnapshot & snapshot)
{
  if (a.amr_id == b.amr_id) {
    // Same robot: sequential only if intervals don't overlap wrongly —
    // earlier action must finish before later starts (handled by deps).
    return a.to_index <= b.from_index || b.to_index <= a.from_index;
  }
  const auto pa = snapshot.paths.find(a.amr_id);
  const auto pb = snapshot.paths.find(b.amr_id);
  if (pa == snapshot.paths.end() || pb == snapshot.paths.end()) {
    // Without geometry, fail closed if they share any reserved conflict id
    // AND also if either has a reserved interval conflict in snapshot.
    if (!a.reserved_conflict_ids.empty() && !b.reserved_conflict_ids.empty()) {
      for (const auto & c : a.reserved_conflict_ids) {
        if (std::find(
            b.reserved_conflict_ids.begin(),
            b.reserved_conflict_ids.end(), c) != b.reserved_conflict_ids.end())
        {
          // Same conflict record may allow following — without paths, reject.
          return false;
        }
      }
    }
    return true;
  }
  // Different conflict IDs can still collide in space — always check geometry.
  if (!intervals_compatible(
      pa->second, a.from_index, a.to_index,
      pb->second, b.from_index, b.to_index,
      snapshot.footprint_radius_m, snapshot.load_length_m))
  {
    return false;
  }
  // Peer live/pending reservations.
  const auto ra = snapshot.reserved_intervals.find(a.amr_id);
  const auto rb = snapshot.reserved_intervals.find(b.amr_id);
  if (ra != snapshot.reserved_intervals.end() &&
    rb != snapshot.reserved_intervals.end())
  {
    if (!intervals_compatible(
        pa->second, ra->second.first, ra->second.second,
        pb->second, rb->second.first, rb->second.second,
        snapshot.footprint_radius_m, snapshot.load_length_m))
    {
      return false;
    }
  }
  return true;
}

}  // namespace ecbs_planner
