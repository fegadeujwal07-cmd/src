#ifndef ECBS_PLANNER__PATH_TOPOLOGY_HPP_
#define ECBS_PLANNER__PATH_TOPOLOGY_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "ecbs_planner/path_utils.hpp"
#include "ecbs_planner/schedule_cache.hpp"
#include "ecbs_planner/wait_reason.hpp"

namespace ecbs_planner
{

/// Path-geometry overlap class (no zone IDs).
enum class PathTopology
{
  Opposite,
  BackToBack,
  Plus,
  TJunction,
  Unknown,
};

inline const char * path_topology_wait_reason(PathTopology t)
{
  switch (t) {
    case PathTopology::Opposite: return kWaitReasonOpposite;
    case PathTopology::BackToBack: return kWaitReasonBackToBack;
    case PathTopology::Plus: return kWaitReasonPlus;
    case PathTopology::TJunction: return kWaitReasonTJunction;
    case PathTopology::Unknown: return kWaitReasonCorridor;
  }
  return kWaitReasonCorridor;
}

/// Unit heading of \p path into waypoint \p i (segment i-1→i). Returns false
/// if undefined.
inline bool path_heading(
  const TimedPath & path, std::size_t i, double & hx, double & hy)
{
  if (i == 0 || i >= path.waypoints.size()) {
    return false;
  }
  const double dx = path.waypoints[i].x - path.waypoints[i - 1].x;
  const double dy = path.waypoints[i].y - path.waypoints[i - 1].y;
  const double len = std::hypot(dx, dy);
  if (len < 1e-9) {
    return false;
  }
  hx = dx / len;
  hy = dy / len;
  return true;
}

inline double heading_dot(
  double ax, double ay, double bx, double by)
{
  return ax * bx + ay * by;
}

/// True if every waypoint of \p rem_self lies near \p rem_other's polyline
/// in order (self finishes along other's remaining path) — path-geometry
/// analogue of zone_manager::remaining_blocks_other.
inline bool remaining_path_blocks_other(
  const std::vector<TimedPoint> & rem_self,
  const std::vector<TimedPoint> & rem_other,
  double eps_m)
{
  if (rem_self.empty() || rem_other.size() < 2) {
    return false;
  }
  std::size_t j = 0;
  for (const auto & p : rem_self) {
    bool found = false;
    for (; j + 1 < rem_other.size(); ++j) {
      const double d0 = std::hypot(
        p.x - rem_other[j].x, p.y - rem_other[j].y);
      const double d1 = std::hypot(
        p.x - rem_other[j + 1].x, p.y - rem_other[j + 1].y);
      // Distance to segment j→j+1 (endpoint min is enough for coarse match).
      if (std::min(d0, d1) <= eps_m) {
        found = true;
        break;
      }
      const double seg = std::hypot(
        rem_other[j + 1].x - rem_other[j].x,
        rem_other[j + 1].y - rem_other[j].y);
      if (seg > 1e-9) {
        const double t = std::clamp(
          ((p.x - rem_other[j].x) * (rem_other[j + 1].x - rem_other[j].x) +
            (p.y - rem_other[j].y) * (rem_other[j + 1].y - rem_other[j].y)) /
            (seg * seg),
          0.0, 1.0);
        const double mx = rem_other[j].x + t * (rem_other[j + 1].x - rem_other[j].x);
        const double my = rem_other[j].y + t * (rem_other[j + 1].y - rem_other[j].y);
        if (std::hypot(p.x - mx, p.y - my) <= eps_m) {
          found = true;
          break;
        }
      }
    }
    if (!found) {
      return false;
    }
  }
  return true;
}

inline std::vector<TimedPoint> path_suffix(
  const TimedPath & path, std::size_t after_i)
{
  std::vector<TimedPoint> out;
  if (after_i + 1 >= path.waypoints.size()) {
    return out;
  }
  out.assign(
    path.waypoints.begin() + static_cast<std::ptrdiff_t>(after_i + 1),
    path.waypoints.end());
  return out;
}

/// Heading along an overlap span (mid segment), not only the approach into lo.
inline bool overlap_span_heading(
  const TimedPath & path, std::size_t lo, std::size_t hi,
  double & hx, double & hy)
{
  lo = std::min(lo, path.waypoints.size() - 1);
  hi = std::min(hi, path.waypoints.size() - 1);
  if (hi > lo) {
    return path_heading(path, hi, hx, hy) || path_heading(path, lo + 1, hx, hy);
  }
  return path_heading(path, lo > 0 ? lo : 1, hx, hy);
}

/// Classify overlap of \p self[\p clo..\p chi] with \p other[\p other_lo..\p other_hi].
inline PathTopology classify_path_overlap(
  const TimedPath & self,
  std::size_t clo,
  std::size_t chi,
  const TimedPath & other,
  std::size_t other_lo,
  std::size_t other_hi,
  double /*eps_m*/ = 1.0)
{
  if (self.waypoints.size() < 2 || other.waypoints.size() < 2) {
    return PathTopology::Unknown;
  }
  clo = std::min(clo, self.waypoints.size() - 1);
  chi = std::min(chi, self.waypoints.size() - 1);
  other_lo = std::min(other_lo, other.waypoints.size() - 1);
  other_hi = std::min(other_hi, other.waypoints.size() - 1);

  // 1) Travel along the shared corridor — catches parallel approaches that
  //    reverse on the overlap (orders.txt amr1↔amr2 at y≈72).
  double along_sx = 1.0, along_sy = 0.0, along_ox = 1.0, along_oy = 0.0;
  const bool have_along =
    overlap_span_heading(self, clo, chi, along_sx, along_sy) &&
    overlap_span_heading(other, other_lo, other_hi, along_ox, along_oy);
  if (have_along && heading_dot(along_sx, along_sy, along_ox, along_oy) < -0.5) {
    return PathTopology::Opposite;
  }

  // 2) Approach into the overlap (legacy).
  double shx = 1.0, shy = 0.0, ohx = 1.0, ohy = 0.0;
  const std::size_t si = clo > 0 ? clo : 1;
  const std::size_t oi = other_lo > 0 ? other_lo : 1;
  path_heading(self, si, shx, shy);
  path_heading(other, oi, ohx, ohy);
  const double approach = heading_dot(shx, shy, ohx, ohy);

  if (approach < -0.5) {
    return PathTopology::Opposite;
  }
  // Same approach alone is not enough for B2B when the shared span is a
  // cross/merge (both turned onto a perpendicular corridor).
  if (approach > 0.5 && have_along &&
    heading_dot(along_sx, along_sy, along_ox, along_oy) > 0.5)
  {
    return PathTopology::BackToBack;
  }

  const bool self_past = chi + 1 < self.waypoints.size();
  const bool other_past = other_hi + 1 < other.waypoints.size();
  if (self_past && other_past) {
    double esx = 1.0, esy = 0.0, eox = 1.0, eoy = 0.0;
    path_heading(self, chi + 1, esx, esy);
    path_heading(other, other_hi + 1, eox, eoy);
    const double exit_dot = heading_dot(esx, esy, eox, eoy);
    // Same exit lane → merge (T); divergent → crossing (+).
    if (exit_dot > 0.5) {
      return PathTopology::TJunction;
    }
    return PathTopology::Plus;
  }
  // Only one continues past the shared cell → through vs branch (T).
  return PathTopology::TJunction;
}

inline bool point_near_polyline(
  const TimedPoint & p, const std::vector<TimedPoint> & poly, double eps_m)
{
  for (std::size_t i = 0; i + 1 < poly.size(); ++i) {
    const double d0 = std::hypot(p.x - poly[i].x, p.y - poly[i].y);
    const double d1 = std::hypot(p.x - poly[i + 1].x, p.y - poly[i + 1].y);
    if (std::min(d0, d1) <= eps_m) {
      return true;
    }
    const double seg = std::hypot(
      poly[i + 1].x - poly[i].x, poly[i + 1].y - poly[i].y);
    if (seg > 1e-9) {
      const double t = std::clamp(
        ((p.x - poly[i].x) * (poly[i + 1].x - poly[i].x) +
          (p.y - poly[i].y) * (poly[i + 1].y - poly[i].y)) / (seg * seg),
        0.0, 1.0);
      const double mx = poly[i].x + t * (poly[i + 1].x - poly[i].x);
      const double my = poly[i].y + t * (poly[i + 1].y - poly[i].y);
      if (std::hypot(p.x - mx, p.y - my) <= eps_m) {
        return true;
      }
    }
  }
  if (poly.size() == 1) {
    return std::hypot(p.x - poly[0].x, p.y - poly[0].y) <= eps_m;
  }
  return false;
}

/// Length of \p rem that is NOT near \p other_rem (exclusive free exit).
inline double exclusive_remaining_length(
  const std::vector<TimedPoint> & rem,
  const std::vector<TimedPoint> & other_rem,
  double eps_m)
{
  if (rem.size() < 2) {
    return 0.0;
  }
  double L = 0.0;
  for (std::size_t i = 1; i < rem.size(); ++i) {
    if (point_near_polyline(rem[i], other_rem, eps_m) &&
      point_near_polyline(rem[i - 1], other_rem, eps_m))
    {
      continue;  // still on shared corridor
    }
    L += std::hypot(rem[i].x - rem[i - 1].x, rem[i].y - rem[i - 1].y);
  }
  return L;
}

/// True when \p self can hold strictly before overlap entry \p clo without
/// sitting on \p other's path (joiner/clearable yield). False → cannot yield
/// clear (through, or total head-on).
inline bool can_hold_clear_of_peer(
  const TimedPath & self,
  std::size_t clo,
  const TimedPath & other,
  double eps_m,
  double safety_m = 0.0,
  double amr_half_diag = 0.0)
{
  if (self.waypoints.size() < 2 || other.waypoints.empty() || clo == 0) {
    return false;
  }
  clo = std::min(clo, self.waypoints.size() - 1);
  const std::size_t wait_i = clear_wait_index(
    self, clo, 0, safety_m, other, eps_m, amr_half_diag);
  if (wait_i >= clo) {
    return false;
  }
  const auto & w = self.waypoints[wait_i];
  if (near_path(w.x, w.y, other, eps_m)) {
    return false;
  }
  if (amr_half_diag > 0.0 && body_overlap_at(self, wait_i, other, amr_half_diag)) {
    return false;
  }
  return true;
}

/// ZM C3 + through-vs-crosser when prefix is inconclusive (shared corridor
/// with opposite travel then diverge — orders.txt amr1 through / amr2 cross).
/// Sets \p self_blocks / \p other_blocks for WaitDependency flags.
inline void path_easiness_flags(
  const std::vector<TimedPoint> & rem_self,
  const std::vector<TimedPoint> & rem_other,
  double eps_m,
  bool & self_blocks,
  bool & other_blocks)
{
  self_blocks = remaining_path_blocks_other(rem_self, rem_other, eps_m);
  other_blocks = remaining_path_blocks_other(rem_other, rem_self, eps_m);
  if (self_blocks != other_blocks) {
    return;  // clear ZM C3 winner
  }
  self_blocks = false;
  other_blocks = false;
  // Need a real shared remaining corridor.
  bool share = false;
  for (const auto & p : rem_self) {
    if (point_near_polyline(p, rem_other, eps_m)) {
      share = true;
      break;
    }
  }
  if (!share || rem_self.size() < 2 || rem_other.size() < 2) {
    return;
  }
  // Through traffic = longer exclusive free exit after the shared cells.
  // Crosser / shorter exit yields (self_blocks) so the through robot GO's.
  const double self_ex = exclusive_remaining_length(rem_self, rem_other, eps_m);
  const double other_ex = exclusive_remaining_length(rem_other, rem_self, eps_m);
  constexpr double k_eps_m = 0.5;
  if (self_ex + k_eps_m < other_ex) {
    self_blocks = true;
  } else if (other_ex + k_eps_m < self_ex) {
    other_blocks = true;
  }
}

/// Path-level easiness: C3, then clearable-hold joiner (orders amr4 south of
/// 48,72), then exclusive-exit. Joiner who can hold clear of peer yields;
/// through who cannot hold clear of peer without deadlocking GO's.
inline void path_easiness_flags(
  const TimedPath & self,
  std::size_t clo,
  const TimedPath & other,
  std::size_t other_lo,
  double eps_m,
  bool & self_blocks,
  bool & other_blocks,
  double safety_m = 0.0,
  double amr_half_diag = 0.0)
{
  (void)safety_m;
  (void)amr_half_diag;
  const auto rem_self = path_suffix(self, clo);
  const auto rem_other = path_suffix(other, other_lo);
  self_blocks = remaining_path_blocks_other(rem_self, rem_other, eps_m);
  other_blocks = remaining_path_blocks_other(rem_other, rem_self, eps_m);
  if (self_blocks != other_blocks) {
    return;
  }
  self_blocks = false;
  other_blocks = false;

  // Approach headings into the overlap.
  double shx = 1.0, shy = 0.0, ohx = 1.0, ohy = 0.0;
  const bool have_h =
    path_heading(self, clo > 0 ? clo : 1, shx, shy) &&
    path_heading(other, other_lo > 0 ? other_lo : 1, ohx, ohy);
  const bool collinear_opposite =
    have_h && heading_dot(shx, shy, ohx, ohy) < -0.5;

  // Same-lane head-on: exclusive-exit must not FREE the longer-tail robot
  // (E7 mutual / fail-closed). Leave flags false → caller ties to WAIT.
  if (collinear_opposite) {
    return;
  }

  path_easiness_flags(rem_self, rem_other, eps_m, self_blocks, other_blocks);
}

/// Easiness: if letting \p self go first would leave self on other's remaining
/// path, self should wait. Returns true when \p self is the easiness loser.
inline bool t_junction_self_should_wait(
  const TimedPath & self,
  std::size_t chi,
  const TimedPath & other,
  std::size_t other_hi,
  double eps_m)
{
  const auto rem_self = path_suffix(self, chi);
  const auto rem_other = path_suffix(other, other_hi);
  bool self_blocks = false;
  bool other_blocks = false;
  path_easiness_flags(rem_self, rem_other, eps_m, self_blocks, other_blocks);
  if (self_blocks && !other_blocks) {
    return true;
  }
  if (other_blocks && !self_blocks) {
    return false;
  }
  return true;  // tie → fail-closed hold
}

}  // namespace ecbs_planner

#endif  // ECBS_PLANNER__PATH_TOPOLOGY_HPP_
