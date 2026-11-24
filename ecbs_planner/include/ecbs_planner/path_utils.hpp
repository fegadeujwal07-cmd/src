#ifndef ECBS_PLANNER__PATH_UTILS_HPP_
#define ECBS_PLANNER__PATH_UTILS_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <unordered_set>
#include <vector>

#include "ecbs_planner/types.hpp"

namespace ecbs_planner
{

/// Quantize metres to mm — matches former hypot < 1e-3 clear-point equality.
inline std::uint64_t clear_point_key(double x, double y)
{
  const auto qx = static_cast<std::int32_t>(std::llround(x * 1000.0));
  const auto qy = static_cast<std::int32_t>(std::llround(y * 1000.0));
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(qx)) << 32) |
         static_cast<std::uint64_t>(static_cast<std::uint32_t>(qy));
}

/// Append clear points not already present (mm-grid dedup, no hypot).
inline void append_unique_clear_points(
  std::vector<ClearPoint> & dst,
  const std::vector<ClearPoint> & src)
{
  std::unordered_set<std::uint64_t> seen;
  seen.reserve(dst.size() + src.size());
  for (const auto & e : dst) {
    seen.insert(clear_point_key(e.x, e.y));
  }
  for (const auto & p : src) {
    if (seen.insert(clear_point_key(p.x, p.y)).second) {
      dst.push_back(p);
    }
  }
}

/// Hold time published to ZM: real dwell, or the largest provisional
/// wait_point.wait_sec (metadata that never shifts timing).
inline double published_wait_sec(const TimedPoint & wp)
{
  double w = wp.wait_duration_sec;
  for (const auto & dep : wp.waiting_for) {
    w = std::max(w, dep.wait_point.wait_duration_sec);
  }
  return w;
}

/// Stamp 0..n-1 route_occurrence when the path looks unstamped.
/// Preserves existing monotonic stamps (e.g. after rolling erase).
inline void ensure_route_occurrences(TimedPath & path)
{
  if (path.waypoints.empty()) {
    return;
  }
  bool stamped = path.waypoints.size() == 1;
  if (path.waypoints.size() > 1) {
    stamped = true;
    for (std::size_t i = 1; i < path.waypoints.size(); ++i) {
      if (path.waypoints[i].route_occurrence <=
        path.waypoints[i - 1].route_occurrence)
      {
        stamped = false;
        break;
      }
    }
    // All zeros → treat as unstamped even if "monotonic".
    if (stamped && path.waypoints.front().route_occurrence == 0 &&
      path.waypoints.back().route_occurrence == 0)
    {
      stamped = false;
    }
  }
  if (stamped) {
    return;
  }
  for (std::size_t i = 0; i < path.waypoints.size(); ++i) {
    path.waypoints[i].route_occurrence = static_cast<uint32_t>(i);
  }
}

/// Local vector index for an immutable route_occurrence, or nullopt if absent.
inline std::optional<std::size_t> local_index_for_occurrence(
  const TimedPath & path, uint32_t occurrence)
{
  for (std::size_t i = 0; i < path.waypoints.size(); ++i) {
    if (path.waypoints[i].route_occurrence == occurrence) {
      return i;
    }
  }
  return std::nullopt;
}

/// Map a TM/global grant tip onto the current (possibly trimmed) vector.
/// Returns local index to continue from; nullopt if tip is past the cache end
/// (no continuation on this cache entry). Tips behind the remaining front
/// map to local 0 (already passed).
inline std::optional<int32_t> local_index_for_grant_tip(
  const TimedPath & path, int32_t grant_through_index)
{
  if (path.waypoints.empty()) {
    return std::nullopt;
  }
  if (grant_through_index < 0) {
    return 0;
  }
  const uint32_t tip = static_cast<uint32_t>(grant_through_index);
  if (const auto local = local_index_for_occurrence(path, tip)) {
    return static_cast<int32_t>(*local);
  }
  const uint32_t front = path.waypoints.front().route_occurrence;
  const uint32_t back = path.waypoints.back().route_occurrence;
  if (tip < front) {
    return 0;
  }
  if (tip > back) {
    return std::nullopt;
  }
  // Sparse gap: nearest occurrence at or before tip.
  int32_t best = 0;
  for (std::size_t i = 0; i < path.waypoints.size(); ++i) {
    if (path.waypoints[i].route_occurrence <= tip) {
      best = static_cast<int32_t>(i);
    }
  }
  return best;
}

inline uint32_t occurrence_at_local(const TimedPath & path, std::size_t local)
{
  if (path.waypoints.empty()) {
    return 0;
  }
  local = std::min(local, path.waypoints.size() - 1);
  return path.waypoints[local].route_occurrence;
}

/// Nearest waypoint to (x,y) over the whole path (squared distance, no
/// hypot). A local minimum on the way (curve, noisy pose) does not stop the
/// search. Equally near waypoints (a place passed twice): the one closest to
/// \p hint (last known index).
inline std::size_t nearest_waypoint_index(
  const TimedPath & path,
  double x,
  double y,
  std::size_t hint = 0)
{
  if (path.waypoints.empty()) {
    return 0;
  }
  const std::size_t n = path.waypoints.size();
  hint = std::min(hint, n - 1);

  auto dist2 = [&](std::size_t i) {
    const double dx = path.waypoints[i].x - x;
    const double dy = path.waypoints[i].y - y;
    return dx * dx + dy * dy;
  };
  auto gap = [hint](std::size_t i) {return i > hint ? i - hint : hint - i;};

  constexpr double kTieM2 = 1e-6;
  std::size_t best = hint;
  double best_d2 = dist2(hint);
  for (std::size_t i = 0; i < n; ++i) {
    const double d2 = dist2(i);
    if (d2 < best_d2 - kTieM2 || (d2 <= best_d2 + kTieM2 && gap(i) < gap(best))) {
      best_d2 = d2;
      best = i;
    }
  }
  return best;
}

/// Path length (m) along waypoints from \p from_i to \p to_i inclusive span.
inline double path_length_between(
  const TimedPath & path, std::size_t from_i, std::size_t to_i)
{
  if (path.waypoints.empty() || from_i >= to_i) {
    return 0.0;
  }
  const std::size_t n = path.waypoints.size();
  to_i = std::min(to_i, n - 1);
  from_i = std::min(from_i, to_i);
  double m = 0.0;
  for (std::size_t i = from_i + 1; i <= to_i; ++i) {
    m += std::hypot(
      path.waypoints[i].x - path.waypoints[i - 1].x,
      path.waypoints[i].y - path.waypoints[i - 1].y);
  }
  return m;
}

/// Waypoint to hold at before an overlap starting at \p overlap_lo:
/// the last waypoint at least \p safety_m (along the path) before it.
/// Never earlier than \p min_i (path start / robot position): if there is
/// not enough path behind, hold at \p min_i.
inline std::size_t safety_wait_index(
  const TimedPath & path,
  std::size_t overlap_lo,
  std::size_t min_i,
  double safety_m)
{
  if (path.waypoints.empty()) {
    return 0;
  }
  const std::size_t n = path.waypoints.size();
  min_i = std::min(min_i, n - 1);
  overlap_lo = std::min(overlap_lo, n - 1);
  if (overlap_lo <= min_i) {
    return min_i;
  }
  double back_m = 0.0;
  for (std::size_t i = overlap_lo; i > min_i; --i) {
    const auto & a = path.waypoints[i - 1];
    const auto & b = path.waypoints[i];
    back_m += std::hypot(b.x - a.x, b.y - a.y);
    if (back_m >= safety_m - 1e-9) {
      return i - 1;
    }
  }
  return min_i;
}

/// True if (x, y) is within \p dist_m of any of \p cells.
inline bool near_cells(
  double x, double y, const std::vector<ClearPoint> & cells, double dist_m)
{
  const double d2 = dist_m * dist_m;
  for (const auto & c : cells) {
    const double dx = c.x - x;
    const double dy = c.y - y;
    if (dx * dx + dy * dy <= d2) {
      return true;
    }
  }
  return false;
}

/// Squared distance from (x, y) to segment a–b.
inline double dist2_to_segment(
  double x, double y, const TimedPoint & a, const TimedPoint & b)
{
  const double vx = b.x - a.x;
  const double vy = b.y - a.y;
  const double len2 = vx * vx + vy * vy;
  double t = len2 > 1e-12 ? ((x - a.x) * vx + (y - a.y) * vy) / len2 : 0.0;
  t = std::clamp(t, 0.0, 1.0);
  const double dx = a.x + t * vx - x;
  const double dy = a.y + t * vy - y;
  return dx * dx + dy * dy;
}

/// True if (x, y) is within \p dist_m of \p path (its waypoints or the
/// straight segments between them, so sparse waypoints on curves count).
inline bool near_path(double x, double y, const TimedPath & path, double dist_m)
{
  const double d2 = dist_m * dist_m;
  const auto & w = path.waypoints;
  if (w.size() == 1) {
    return dist2_to_segment(x, y, w[0], w[0]) <= d2;
  }
  for (std::size_t i = 1; i < w.size(); ++i) {
    if (dist2_to_segment(x, y, w[i - 1], w[i]) <= d2) {
      return true;
    }
  }
  return false;
}

/// safety_wait_index, then further back (never before \p min_i) while the
/// wait point is within \p clearance_m of \p away (the other robot's path):
/// on curves and side-by-side lanes the point safety_m back along the path
/// can still be beside it.
inline std::size_t clear_wait_index(
  const TimedPath & path,
  std::size_t overlap_lo,
  std::size_t min_i,
  double safety_m,
  const TimedPath & away,
  double clearance_m)
{
  std::size_t i = safety_wait_index(path, overlap_lo, min_i, safety_m);
  min_i = std::min(min_i, i);
  while (i > min_i && !away.waypoints.empty() &&
    near_path(path.waypoints[i].x, path.waypoints[i].y, away, clearance_m))
  {
    --i;
  }
  return i;
}

/// Compute the heading unit vector entering waypoint \p i on \p path
/// (from i-1 → i, or +x if no predecessor).
inline void heading_at(
  const TimedPath & path, std::size_t i, double & hx, double & hy)
{
  hx = 1.0; hy = 0.0;
  if (i == 0 || path.waypoints.size() < 2) {
    return;
  }
  const auto & a = path.waypoints[i - 1];
  const auto & b = path.waypoints[i];
  const double len = std::hypot(b.x - a.x, b.y - a.y);
  if (len > 1e-9) {
    hx = (b.x - a.x) / len;
    hy = (b.y - a.y) / len;
  }
}

/// True if an AMR body (half_diag radius circle approximation) at waypoint
/// \p i of \p path would overlap the blocker AMR's body anywhere along
/// \p blocker_path within \p body_reach_m.  Using the conservative
/// half-diagonal of the rectangular body (sqrt((L/2)^2+(W/2)^2)) as the
/// effective collision radius gives a safe, rotation-independent check.
/// Two bodies touch when centre-to-centre distance < 2 * half_diag.
inline bool body_overlap_at(
  const TimedPath & path,
  std::size_t i,
  const TimedPath & blocker_path,
  double half_diag)
{
  if (blocker_path.waypoints.empty() || i >= path.waypoints.size()) {
    return false;
  }
  const double touch_dist = 2.0 * half_diag;
  const double touch_d2   = touch_dist * touch_dist;
  const double wx = path.waypoints[i].x;
  const double wy = path.waypoints[i].y;
  // Check against every waypoint and the segment to the next one.
  const auto & bw = blocker_path.waypoints;
  if (bw.size() == 1) {
    const double dx = bw[0].x - wx;
    const double dy = bw[0].y - wy;
    return (dx * dx + dy * dy) < touch_d2;
  }
  for (std::size_t k = 1; k < bw.size(); ++k) {
    if (dist2_to_segment(wx, wy, bw[k - 1], bw[k]) < touch_d2) {
      return true;
    }
  }
  return false;
}

/// clear_wait_index with explicit AMR body dimensions:
/// 1. Start at safety_m before the overlap (same as before).
/// 2. If the AMR body (half_diag = sqrt((L/2)^2+(W/2)^2)) at that point
///    overlaps the blocker's body, step back one waypoint at a time.
/// 3. Stop stepping as soon as bodies are clear, or \p min_i is reached.
/// Falls back to the simple clearance check if half_diag <= 0.
inline std::size_t clear_wait_index(
  const TimedPath & path,
  std::size_t overlap_lo,
  std::size_t min_i,
  double safety_m,
  const TimedPath & away,
  double clearance_m,
  double amr_half_diag)
{
  std::size_t i = safety_wait_index(path, overlap_lo, min_i, safety_m);
  min_i = std::min(min_i, i);
  if (amr_half_diag <= 0.0) {
    // Fallback: original clearance-from-path logic.
    while (i > min_i && !away.waypoints.empty() &&
      near_path(path.waypoints[i].x, path.waypoints[i].y, away, clearance_m))
    {
      --i;
    }
    return i;
  }
  // Body-overlap check: step back only while bodies would physically touch.
  while (i > min_i && body_overlap_at(path, i, away, amr_half_diag)) {
    --i;
  }
  return i;
}

/// Array index of a given base end on \p path, searching from \p from_i.
/// Matches nav-graph index first, else XY within \p tol_m.
inline std::optional<std::size_t> locate_granted_base(
  const TimedPath & path,
  const GrantedBase & base,
  std::size_t from_i,
  double tol_m = 0.5)
{
  const std::size_t n = path.waypoints.size();
  if (base.index.has_value()) {
    for (std::size_t i = from_i; i < n; ++i) {
      const auto & wp = path.waypoints[i];
      if (wp.index.has_value() && *wp.index == *base.index) {
        return i;
      }
    }
  }
  std::optional<std::size_t> best;
  double best_d2 = tol_m * tol_m;
  for (std::size_t i = from_i; i < n; ++i) {
    const double dx = path.waypoints[i].x - base.x;
    const double dy = path.waypoints[i].y - base.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 <= best_d2) {
      best_d2 = d2;
      best = i;
    }
  }
  return best;
}

}  // namespace ecbs_planner

#endif  // ECBS_PLANNER__PATH_UTILS_HPP_
