#include "ecbs_planner/ecbs_solver.hpp"

#include <algorithm>       // std::max
#include <limits>          // infinity for earliest conflict search
#include <unordered_map>   // per-AMR wait totals / mutable others
#include <unordered_set>   // which others we modified
#include <vector>

#include "ecbs_planner/path_utils.hpp"

#include <stdexcept>       // load_solver_config_yaml errors
#include <type_traits>     // std::decay_t (load_solver_config_yaml)
#include <yaml-cpp/yaml.h>  // load_solver_config_yaml

namespace ecbs_planner
{
namespace
{

/// True if time intervals [a0,a1) and [b0,b1) overlap.
bool intervals_overlap(double a0, double a1, double b0, double b1)
{
  return a0 < b1 && b0 < a1;
}

/// Attach or merge a WaitDependency on the waypoint where we dwell.
/// Sets a per-overlap safe wait_point from the hold waypoint.
void add_wait_dependency(
  TimedPoint & wait_wp,
  const std::string & blocker_amr,
  const std::vector<ClearPoint> & clear_pts)
{
  PathWaitPoint safe;
  safe.x = wait_wp.x;
  safe.y = wait_wp.y;
  safe.index = wait_wp.index;
  safe.wait_duration_sec = wait_wp.wait_duration_sec;

  for (auto & dep : wait_wp.waiting_for) {
    if (dep.amr_id == blocker_amr) {
      append_unique_clear_points(dep.clear_points, clear_pts);
      // Refresh safe wait point (duration may have grown). Keep a larger
      // provisional wait_sec already stored for this same hold point.
      if (dep.wait_point.index == safe.index &&
        dep.wait_point.x == safe.x && dep.wait_point.y == safe.y)
      {
        safe.wait_duration_sec = std::max(
          safe.wait_duration_sec, dep.wait_point.wait_duration_sec);
      }
      dep.wait_point = safe;
      return;
    }
  }
  WaitDependency dep;
  dep.amr_id = blocker_amr;
  dep.wait_point = safe;
  dep.clear_points = clear_pts;
  wait_wp.waiting_for.push_back(std::move(dep));
}

/// Provisional wait_sec for ZM/viz on the dependency wait_point only.
/// TimedPoint.wait_duration_sec / times are untouched: shifting them would
/// extend occupancy and recreate the conflict.
void set_provisional_wait(
  TimedPath & path,
  std::size_t wait_wp,
  const std::string & blocker_amr,
  double wait_sec)
{
  if (wait_wp >= path.waypoints.size() || wait_sec <= 1e-3) {
    return;
  }
  auto & wp = path.waypoints[wait_wp];
  for (auto & dep : wp.waiting_for) {
    if (dep.amr_id == blocker_amr) {
      dep.wait_point.wait_duration_sec = std::max(
        dep.wait_point.wait_duration_sec, wait_sec);
      dep.wait_point.x = wp.x;
      dep.wait_point.y = wp.y;
      dep.wait_point.index = wp.index;
    }
  }
}

/// Hold at wait_at (before overlapped path), then shift later arrivals.
void delay_at_wait_wp(
  TimedPath & path,
  std::size_t wait_at,
  double delta_t,
  const std::string & blocker_amr,
  const std::vector<ClearPoint> & clear_pts)
{
  if (delta_t <= 1e-9 || path.waypoints.empty() ||
    wait_at >= path.waypoints.size())
  {
    return;
  }

  path.waypoints[wait_at].wait_duration_sec += delta_t;
  add_wait_dependency(path.waypoints[wait_at], blocker_amr, clear_pts);
  // Keep each dep's wait_point duration in sync after the hold grows.
  for (auto & dep : path.waypoints[wait_at].waiting_for) {
    if (dep.amr_id == blocker_amr) {
      dep.wait_point.wait_duration_sec =
        path.waypoints[wait_at].wait_duration_sec;
      dep.wait_point.x = path.waypoints[wait_at].x;
      dep.wait_point.y = path.waypoints[wait_at].y;
      dep.wait_point.index = path.waypoints[wait_at].index;
    }
  }

  // Shift every waypoint after the hold (includes overlapped corridor + beyond).
  for (std::size_t k = wait_at + 1; k < path.waypoints.size(); ++k) {
    path.waypoints[k].t_sec += delta_t;
  }

  // If waiting at wp 0, also shift arrival at 0 so the hold starts immediately.
  if (wait_at == 0) {
    path.waypoints[0].t_sec += delta_t;
  }
}

/// Occupancy end at waypoint i (anonymous copy of EcbsSolver::depart_sec).
double depart_at(const TimedPath & path, std::size_t i)
{
  if (path.waypoints.empty() || i >= path.waypoints.size()) {
    return 0.0;
  }
  const auto & wp = path.waypoints[i];
  if (i + 1 < path.waypoints.size()) {
    return std::max(path.waypoints[i + 1].t_sec, wp.t_sec + wp.wait_duration_sec);
  }
  return wp.t_sec + std::max(wp.wait_duration_sec, 0.1);
}

/// Latest depart time over waypoints [lo, hi] inclusive.
double corridor_clear_sec(
  const TimedPath & path,
  std::size_t lo,
  std::size_t hi)
{
  double clear = 0.0;
  if (path.waypoints.empty() || lo > hi || hi >= path.waypoints.size()) {
    return clear;
  }
  for (std::size_t k = lo; k <= hi; ++k) {
    clear = std::max(clear, depart_at(path, k));
  }
  return clear;
}

}  // namespace

/// Store solver config.
EcbsSolver::EcbsSolver(SolverConfig config)
: config_(std::move(config))
{
}

/// Same nav-graph index, or XY within eps on the same level (robots on
/// different graph points still touch when that close).
bool EcbsSolver::same_cell(const TimedPoint & a, const TimedPoint & b, double eps_m)
{
  if (a.index.has_value() && b.index.has_value() && *a.index == *b.index) {
    return true;
  }
  if (!a.level_name.empty() && !b.level_name.empty() && a.level_name != b.level_name) {
    return false;
  }
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::hypot(dx, dy) <= eps_m;
}

/// When occupancy at waypoint i ends (for conflict interval checks).
double EcbsSolver::depart_sec(const TimedPath & path, std::size_t i)
{
  if (path.waypoints.empty() || i >= path.waypoints.size()) {
    return 0.0;
  }
  const auto & wp = path.waypoints[i];
  if (i + 1 < path.waypoints.size()) {
    return std::max(path.waypoints[i + 1].t_sec, wp.t_sec + wp.wait_duration_sec);
  }
  return wp.t_sec + std::max(wp.wait_duration_sec, 0.1);
}

std::vector<ClearPoint> EcbsSolver::clear_points_from_range(
  const TimedPath & blocker,
  std::size_t lo,
  std::size_t hi)
{
  std::vector<ClearPoint> pts;
  if (blocker.waypoints.empty() || lo > hi || hi >= blocker.waypoints.size()) {
    return pts;
  }
  for (std::size_t k = lo; k <= hi; ++k) {
    pts.push_back({blocker.waypoints[k].x, blocker.waypoints[k].y});
  }
  return pts;
}

/// Expand contiguous shared cells around the time-conflict; set wait before corridor.
void EcbsSolver::enrich_overlap_and_wait(
  const TimedPath & a,
  const TimedPath & b,
  ConflictHit & hit,
  double eps_m)
{
  const std::size_t i = hit.self_wp;
  const std::size_t j = hit.other_wp;

  hit.self_overlap_lo = i;
  hit.self_overlap_hi = i;
  hit.other_overlap_lo = j;
  hit.other_overlap_hi = j;
  hit.opposite = false;

  // --- Opposite / head-on: A's previous matches B's next (and vice versa) ---
  std::size_t a_lo = i;
  std::size_t a_hi = i;
  std::size_t b_lo = j;
  std::size_t b_hi = j;
  bool opp = false;

  // Expand "before" on A ↔ "after" on B
  while (a_lo > 0 && b_hi + 1 < b.waypoints.size() &&
    same_cell(a.waypoints[a_lo - 1], b.waypoints[b_hi + 1], eps_m))
  {
    --a_lo;
    ++b_hi;
    opp = true;
  }
  // Expand "after" on A ↔ "before" on B
  while (a_hi + 1 < a.waypoints.size() && b_lo > 0 &&
    same_cell(a.waypoints[a_hi + 1], b.waypoints[b_lo - 1], eps_m))
  {
    ++a_hi;
    --b_lo;
    opp = true;
  }

  const std::size_t opp_span = a_hi - a_lo;

  // --- Same direction / following: indices move together ---
  std::size_t s_lo = i;
  std::size_t s_hi = i;
  std::size_t o_lo = j;
  std::size_t o_hi = j;
  while (s_lo > 0 && o_lo > 0 &&
    same_cell(a.waypoints[s_lo - 1], b.waypoints[o_lo - 1], eps_m))
  {
    --s_lo;
    --o_lo;
  }
  while (s_hi + 1 < a.waypoints.size() && o_hi + 1 < b.waypoints.size() &&
    same_cell(a.waypoints[s_hi + 1], b.waypoints[o_hi + 1], eps_m))
  {
    ++s_hi;
    ++o_hi;
  }
  const std::size_t same_span = s_hi - s_lo;

  // Prefer the wider corridor; opposite wins ties if it expanded at all.
  if (opp && opp_span >= same_span) {
    hit.self_overlap_lo = a_lo;
    hit.self_overlap_hi = a_hi;
    hit.other_overlap_lo = b_lo;
    hit.other_overlap_hi = b_hi;
    hit.opposite = true;
  } else {
    hit.self_overlap_lo = s_lo;
    hit.self_overlap_hi = s_hi;
    hit.other_overlap_lo = o_lo;
    hit.other_overlap_hi = o_hi;
    hit.opposite = false;
  }

  // Wait *before* the overlapped path (first shared cell), not at the conflict.
  hit.self_wait_wp =
    (hit.self_overlap_lo == 0) ? 0 : (hit.self_overlap_lo - 1);
  hit.other_wait_wp =
    (hit.other_overlap_lo == 0) ? 0 : (hit.other_overlap_lo - 1);

  // Clear times: other must free the whole shared corridor before we enter it.
  hit.other_clear_sec = corridor_clear_sec(
    b, hit.other_overlap_lo, hit.other_overlap_hi);
  hit.self_clear_sec = corridor_clear_sec(
    a, hit.self_overlap_lo, hit.self_overlap_hi);
}

/// Scan all waypoint pairs; return earliest-in-time vertex conflict + corridor.
std::optional<ConflictHit> EcbsSolver::earliest_conflict(
  const TimedPath & a,
  const TimedPath & b,
  double eps_m)
{
  if (a.waypoints.empty() || b.waypoints.empty()) {
    return std::nullopt;
  }

  // Precompute depart times once (depend only on each path index).
  std::vector<double> a_dep(a.waypoints.size());
  std::vector<double> b_dep(b.waypoints.size());
  for (std::size_t i = 0; i < a.waypoints.size(); ++i) {
    a_dep[i] = depart_sec(a, i);
  }
  for (std::size_t j = 0; j < b.waypoints.size(); ++j) {
    b_dep[j] = depart_sec(b, j);
  }

  std::size_t best_i = 0;
  std::size_t best_j = 0;
  double best_t = std::numeric_limits<double>::infinity();
  bool found = false;

  for (std::size_t i = 0; i < a.waypoints.size(); ++i) {
    const double a0 = a.waypoints[i].t_sec;
    const double a1 = a_dep[i];
    for (std::size_t j = 0; j < b.waypoints.size(); ++j) {
      // Cheap time window before expensive same_cell / hypot.
      const double b0 = b.waypoints[j].t_sec;
      const double b1 = b_dep[j];
      if (!intervals_overlap(a0, a1, b0, b1)) {
        continue;
      }
      if (!same_cell(a.waypoints[i], b.waypoints[j], eps_m)) {
        continue;
      }
      const double overlap_start = std::max(a0, b0);
      if (overlap_start < best_t) {
        best_t = overlap_start;
        best_i = i;
        best_j = j;
        found = true;
      }
    }
  }

  if (!found) {
    return std::nullopt;
  }

  // Enrich only the true earliest hit (not every provisional minimum).
  ConflictHit hit;
  hit.other_amr = b.amr_id;
  hit.self_wp = best_i;
  hit.other_wp = best_j;
  hit.overlap_start_sec = best_t;
  hit.other_clear_sec = b_dep[best_j];
  hit.self_clear_sec = a_dep[best_i];
  enrich_overlap_and_wait(a, b, hit, eps_m);
  return hit;
}

/// Simple wait apply without dependency metadata (kept for completeness).
void EcbsSolver::apply_wait(TimedPath & path, std::size_t wp_index, double delta_t)
{
  if (path.waypoints.empty() || wp_index >= path.waypoints.size() || delta_t <= 0.0) {
    return;
  }
  path.waypoints[wp_index].wait_duration_sec += delta_t;
  for (std::size_t k = wp_index + 1; k < path.waypoints.size(); ++k) {
    path.waypoints[k].t_sec += delta_t;
  }
}

namespace
{

/// Point and waypoint index at \p along_m from \p from_i along \p path.
void point_at_along(
  const TimedPath & path,
  std::size_t from_i,
  double along_m,
  double & ox, double & oy, std::size_t & oi)
{
  const std::size_t n = path.waypoints.size();
  from_i = std::min(from_i, n - 1);
  oi = from_i;
  ox = path.waypoints[from_i].x;
  oy = path.waypoints[from_i].y;
  if (along_m <= 1e-9) {
    return;
  }
  double left = along_m;
  for (std::size_t i = from_i + 1; i < n; ++i) {
    const auto & a = path.waypoints[i - 1];
    const auto & b = path.waypoints[i];
    const double seg = std::hypot(b.x - a.x, b.y - a.y);
    if (seg <= 1e-12) {
      oi = i;
      ox = b.x;
      oy = b.y;
      continue;
    }
    if (left <= seg + 1e-9) {
      const double t = std::min(1.0, std::max(0.0, left / seg));
      ox = a.x + t * (b.x - a.x);
      oy = a.y + t * (b.y - a.y);
      // Landed on b → hold index is i; otherwise the previous wp.
      oi = (t >= 1.0 - 1e-9) ? i : i - 1;
      return;
    }
    left -= seg;
    oi = i;
    ox = b.x;
    oy = b.y;
  }
}

/// Along-segment t of the closest point to (cx,cy), or nullopt if farther
/// than reach_m from the segment.
std::optional<double> segment_reach_t(
  double ax, double ay, double bx, double by,
  double cx, double cy, double reach_m)
{
  const double dx = bx - ax;
  const double dy = by - ay;
  const double len2 = dx * dx + dy * dy;
  double t = 0.0;
  if (len2 > 1e-18) {
    t = ((cx - ax) * dx + (cy - ay) * dy) / len2;
    t = std::min(1.0, std::max(0.0, t));
  }
  const double px = ax + t * dx;
  const double py = ay + t * dy;
  const double d2 = (px - cx) * (px - cx) + (py - cy) * (py - cy);
  if (d2 > reach_m * reach_m + 1e-12) {
    return std::nullopt;
  }
  return t;
}

}  // namespace

std::optional<ReservedHit> wait_before_reserved(
  const TimedPath & path,
  std::size_t from_i,
  std::size_t to_i,
  const std::vector<ClearPoint> & reserved,
  const TimedPath & owner_path,
  double footprint_m,
  double safety_m,
  double amr_half_diag)
{
  if (path.waypoints.empty() || reserved.empty()) {
    return std::nullopt;
  }
  const std::size_t n = path.waypoints.size();
  from_i = std::min(from_i, n - 1);
  to_i = std::min(to_i, n - 1);
  const auto & robot = path.waypoints[from_i];

  ReservedHit hit;
  for (const auto & c : reserved) {
    const std::vector<ClearPoint> one{c};
    if (!near_cells(robot.x, robot.y, one, footprint_m)) {
      hit.cells.push_back(c);
    }
  }
  // Conflict if path comes within footprint OR body-touch of a reserved cell
  // (size conflict even when centres are just outside clearance). Check
  // segments too — sparse waypoints can skip past a tip ~2 m off the path
  // (live amr1 tip at y≈70 vs amr2 westbound samples on y=72).
  const double body_touch =
    amr_half_diag > 0.0 ? 2.0 * amr_half_diag : footprint_m;
  const double detect_m = std::max(footprint_m, body_touch);
  const double detect_d2 = detect_m * detect_m;
  bool found = false;
  for (std::size_t i = from_i + 1; i <= to_i; ++i) {
    if (near_cells(path.waypoints[i].x, path.waypoints[i].y, hit.cells, detect_m)) {
      hit.hit_i = i;
      found = true;
      break;
    }
    const auto & a = path.waypoints[i - 1];
    const auto & b = path.waypoints[i];
    for (const auto & c : hit.cells) {
      if (dist2_to_segment(c.x, c.y, a, b) <= detect_d2) {
        hit.hit_i = i;
        found = true;
        break;
      }
    }
    if (found) {
      break;
    }
  }
  if (!found) {
    return std::nullopt;
  }
  // Path point nearest the first reserved cell within kReachM, then wait
  // exactly safety_m before that (continuous XY). Avoids nearest-wp snap
  // stacking (~1 m + 2.5 m ≈ 5 m before a crossing).
  constexpr double kReachM = 0.5;
  double reach_along = -1.0;
  double along = 0.0;
  for (std::size_t i = from_i + 1; i <= to_i; ++i) {
    const auto & a = path.waypoints[i - 1];
    const auto & b = path.waypoints[i];
    const double seg = std::hypot(b.x - a.x, b.y - a.y);
    double best_t = 2.0;
    for (const auto & c : hit.cells) {
      if (const auto t = segment_reach_t(
          a.x, a.y, b.x, b.y, c.x, c.y, kReachM))
      {
        best_t = std::min(best_t, *t);
      }
    }
    if (best_t <= 1.0) {
      reach_along = along + best_t * seg;
      break;
    }
    along += seg;
  }
  if (reach_along < 0.0) {
    reach_along = 0.0;
    for (std::size_t i = from_i + 1; i <= hit.hit_i && i < n; ++i) {
      reach_along += std::hypot(
        path.waypoints[i].x - path.waypoints[i - 1].x,
        path.waypoints[i].y - path.waypoints[i - 1].y);
    }
  }
  const double wait_along = std::max(0.0, reach_along - safety_m);
  point_at_along(path, from_i, wait_along, hit.wait_x, hit.wait_y, hit.wait_i);
  // A wait is always a real path waypoint (never mid-segment): point_at_along
  // leaves wait_i on the waypoint at or before the safety_m point, so the stop
  // is >= safety_m before the conflict.
  hit.wait_x = path.waypoints[hit.wait_i].x;
  hit.wait_y = path.waypoints[hit.wait_i].y;
  hit.has_exact_wait = false;
  // Body dash: step further back only while L×W still overlaps.
  if (amr_half_diag > 0.0) {
    while (hit.wait_i > from_i &&
      body_overlap_at(path, hit.wait_i, owner_path, amr_half_diag))
    {
      --hit.wait_i;
      hit.wait_x = path.waypoints[hit.wait_i].x;
      hit.wait_y = path.waypoints[hit.wait_i].y;
    }
  } else if (!owner_path.waypoints.empty()) {
    while (hit.wait_i > from_i &&
      near_path(
        path.waypoints[hit.wait_i].x, path.waypoints[hit.wait_i].y,
        owner_path, footprint_m))
    {
      --hit.wait_i;
      hit.wait_x = path.waypoints[hit.wait_i].x;
      hit.wait_y = path.waypoints[hit.wait_i].y;
    }
  }
  return hit;
}

void attach_reserved_waits(
  TimedPath & path, ScheduleCache & cache, const SolverConfig & config)
{
  if (path.waypoints.empty()) {
    return;
  }
  std::unordered_set<std::string> blockers;
  bool changed = false;
  const auto own = cache.reserved(path.amr_id);
  const double half_diag = std::hypot(
    config.amr_length_m * 0.5, config.amr_width_m * 0.5);
  for (const auto & kv : cache.reserved_except(path.amr_id)) {
    if (waits_for(cache, kv.first, path.amr_id)) {
      continue;
    }
    TimedPath owner_path;
    if (auto p = cache.get(kv.first)) {
      owner_path = std::move(*p);
    }
    const auto hit = wait_before_reserved(
      path, 0, path.waypoints.size() - 1,
      blocking_cells(kv.second, own, config.clearance_m), owner_path,
      config.clearance_m, config.safety_distance_m, half_diag);
    if (!hit) {
      continue;
    }
    blockers.insert(kv.first);
    bool already = false;
    for (const auto & wp : path.waypoints) {
      for (const auto & d : wp.waiting_for) {
        already = already || d.amr_id == kv.first;
      }
    }
    if (!already) {
      add_wait_dependency(path.waypoints[hit->wait_i], kv.first, hit->cells);
      if (hit->has_exact_wait) {
        for (auto & d : path.waypoints[hit->wait_i].waiting_for) {
          if (d.amr_id == kv.first) {
            d.wait_point.x = hit->wait_x;
            d.wait_point.y = hit->wait_y;
          }
        }
      }
      changed = true;
    }
  }
  if (changed) {
    cache.upsert(path);
  }
  cache.set_reserve_waits(path.amr_id, std::move(blockers));
}

namespace
{

void commit_paths(
  TimedPath & working,
  std::unordered_map<std::string, TimedPath> & others_mut,
  const std::unordered_set<std::string> & modified_others,
  ScheduleCache & cache,
  SolveResult & result)
{
  cache.upsert(working);
  cache.add_plan_holds(working);
  result.path = working;
  result.updated_others.clear();
  for (auto & kv : others_mut) {
    if (modified_others.count(kv.first) > 0) {
      cache.upsert(kv.second);
      cache.add_plan_holds(kv.second);
      result.updated_others.push_back(kv.second);
    }
  }
}

}  // namespace

/// Resolve loop: apply waits, then re-scan every cached AMR.
/// Commit once when nothing conflicts. FailReplan on max_wait or max_iters.
SolveResult EcbsSolver::solve(const TimedPath & candidate, ScheduleCache & cache) const
{
  SolveResult result;
  result.path = candidate;

  if (candidate.waypoints.empty()) {
    result.status = SolveStatus::Ok;
    result.reason = "empty path";
    cache.upsert(result.path);
    cache.add_plan_holds(result.path);
    return result;
  }

  // Keep prior granted/reserved until pose vacates (update_pose) or evaluate
  // commits a superseding base — do not wipe corridor authority on replan (C2).

  TimedPath working = candidate;
  std::unordered_map<std::string, TimedPath> others_mut;
  for (const auto & other : cache.all_except(candidate.amr_id)) {
    others_mut.emplace(other.amr_id, other);
  }

  double total_wait_self = 0.0;
  std::unordered_map<std::string, double> total_wait_other;
  std::unordered_set<std::string> modified_others;
  int iteration = 0;

  while (iteration < config_.max_iters) {
    ++iteration;

    std::optional<ConflictHit> earliest;
    TimedPath * earliest_other = nullptr;
    double earliest_t = std::numeric_limits<double>::infinity();

    for (auto & kv : others_mut) {
      auto hit = earliest_conflict(working, kv.second, config_.clearance_m);
      if (!hit.has_value()) {
        continue;
      }
      if (hit->overlap_start_sec < earliest_t) {
        earliest_t = hit->overlap_start_sec;
        earliest = hit;
        earliest_other = &kv.second;
      }
    }

    if (!earliest.has_value() || earliest_other == nullptr) {
      commit_paths(working, others_mut, modified_others, cache, result);
      attach_reserved_waits(result.path, cache, config_);
      if (!result.updated_others.empty()) {
        result.status = SolveStatus::UpdatedOther;
        result.reason = total_wait_self > 0.0 ?
          "mutual / both-side waits applied; ScheduleCache updated" :
          "wait applied on other AMR(s); ScheduleCache updated";
      } else {
        result.status = SolveStatus::Ok;
        result.reason = total_wait_self > 0.0 ?
          "wait applied on candidate" : "conflict-free";
      }
      return result;
    }

    result.conflicting_amrs.push_back(earliest->other_amr);

    // Wait points: safety_distance_m before the overlap (or path start).
    // A cached AMR whose given base covers its overlap start is never
    // time-shifted there (it is inside the corridor); it holds at the first
    // waypoint of the next base when the overlap runs past the given base,
    // else at the given base end.

    // --- Leader / follower detection -----------------------------------------
    // If the OTHER AMR's heading at the conflict has a positive dot product
    // with SELF's heading, OTHER is approaching from the same / behind
    // direction → OTHER is the follower → SELF (the leader) must NOT wait.
    // The follower's (OTHER's) wait point is pegged to the leader's live
    // pose so it advances as the leader moves (Option-A dynamic wait).
    bool other_is_follower = false;
    bool self_is_follower = false;
    if (cache.pose(working.amr_id).has_value() || cache.pose(earliest_other->amr_id).has_value()) {
      const std::size_t si = earliest->self_overlap_lo;
      const std::size_t oi = earliest->other_overlap_lo;
      double shx = 1.0, shy = 0.0, ohx = 1.0, ohy = 0.0;
      heading_at(working, si > 0 ? si : 1, shx, shy);
      heading_at(*earliest_other, oi > 0 ? oi : 1, ohx, ohy);
      const bool same_dir = (shx * ohx + shy * ohy) > 0.0;
      if (same_dir) {
        const double self_t = working.waypoints[si].t_sec;
        const double other_t = earliest_other->waypoints[oi].t_sec;
        if (self_t <= other_t + 1e-3) {
          other_is_follower = true;
        } else {
          self_is_follower = true;
        }
      }
    }

    // Compute AMR half-diagonal for body-overlap check.
    const double half_diag = std::hypot(
      config_.amr_length_m * 0.5, config_.amr_width_m * 0.5);

    earliest->self_wait_wp = clear_wait_index(
      working, earliest->self_overlap_lo, 0, config_.safety_distance_m,
      *earliest_other, config_.clearance_m, half_diag);
    earliest->other_wait_wp = clear_wait_index(
      *earliest_other, earliest->other_overlap_lo, 0, config_.safety_distance_m,
      working, config_.clearance_m, half_diag);

    // If other is the follower, dynamically anchor its wait to leader's live
    // pose so the wait travels forward as the leader (self) advances.
    if (other_is_follower) {
      const auto leader_pose = cache.pose(working.amr_id);
      if (leader_pose) {
        // Find the point on other's path nearest to the leader's live position.
        const std::size_t nearest_to_leader =
          nearest_waypoint_index(*earliest_other, leader_pose->x, leader_pose->y);
        // Place other's wait safety_distance_m before that point.
        earliest->other_wait_wp = safety_wait_index(
          *earliest_other, nearest_to_leader, 0, config_.safety_distance_m);
        // Apply body-overlap check and step back if needed.
        while (earliest->other_wait_wp > 0 &&
          body_overlap_at(*earliest_other, earliest->other_wait_wp, working, half_diag))
        {
          --earliest->other_wait_wp;
        }
      }
    }
    bool other_owns_base = false;
    // Other shares cells with the candidate only inside its given base:
    // it has priority there and gets no wait at all; only the candidate waits.
    bool other_inside_base = false;
    if (const auto base = cache.granted_base(earliest_other->amr_id)) {
      if (base->task_id.empty() || earliest_other->task_id.empty() ||
        base->task_id == earliest_other->task_id)
      {
        if (const auto g = locate_granted_base(*earliest_other, *base, 0)) {
          if (earliest->other_overlap_lo < *g) {
            earliest->other_wait_wp = earliest->other_overlap_hi > *g ?
              std::min(*g + 1, earliest_other->waypoints.size() - 1) : *g;
            other_owns_base = true;
            other_inside_base = true;
            for (std::size_t i = *g + 1;
              other_inside_base && i < earliest_other->waypoints.size(); ++i)
            {
              for (const auto & w : working.waypoints) {
                if (same_cell(earliest_other->waypoints[i], w, config_.clearance_m)) {
                  other_inside_base = false;
                  break;
                }
              }
            }
          } else {
            earliest->other_wait_wp = std::min(earliest->other_wait_wp, *g);
          }
        }
      }
    }
    // Same for a stretch reserved on WAIT (robot → its wait point): every
    // cell the other shares with the candidate lies in it → no wait for it.
    if (!other_inside_base) {
      std::vector<ClearPoint> cells;
      for (const auto & c : cache.reserved(earliest_other->amr_id)) {
        cells.push_back({c.x, c.y});
      }
      bool shared_any = false;
      bool all_reserved = !cells.empty();
      for (const auto & o : earliest_other->waypoints) {
        if (!all_reserved) {
          break;
        }
        for (const auto & w : working.waypoints) {
          if (same_cell(o, w, config_.clearance_m)) {
            shared_any = true;
            all_reserved = near_cells(o.x, o.y, cells, config_.position_epsilon_m);
            break;
          }
        }
      }
      if (shared_any && all_reserved) {
        other_owns_base = true;
        other_inside_base = true;
      }
    }

    // Wait until the other clears the *whole overlapped corridor*, then enter
    // at the first shared cell (arrival time at overlap_lo).
    const double self_arrive_overlap =
      working.waypoints[earliest->self_overlap_lo].t_sec;
    const double other_arrive_overlap =
      earliest_other->waypoints[earliest->other_overlap_lo].t_sec;

    const double wait_self =
      earliest->other_clear_sec + config_.safety_margin_sec - self_arrive_overlap;
    const double wait_other =
      earliest->self_clear_sec + config_.safety_margin_sec - other_arrive_overlap;

    // Leader/follower rule: the leader AMR must NEVER be given a wait —
    // only the follower AMR waits.
    const bool self_can =
      !other_is_follower &&
      wait_self > 1e-3 &&
      (total_wait_self + wait_self) <= config_.max_wait_sec;
    const bool other_can = !other_owns_base &&
      !self_is_follower &&
      wait_other > 1e-3 &&
      (total_wait_other[earliest->other_amr] + wait_other) <= config_.max_wait_sec;

    if (!self_can && !other_can) {
      result.status = SolveStatus::FailReplan;
      result.reason = (other_is_follower || self_is_follower) ?
        "follower AMR cannot clear conflict by waiting within max_wait" :
        "neither AMR can clear conflict by waiting within max_wait";
      result.conflicting_amrs = {candidate.amr_id, earliest->other_amr};
      return result;
    }

    const auto clear_pts_other = EcbsSolver::clear_points_from_range(
      *earliest_other,
      earliest->other_overlap_lo,
      earliest->other_overlap_hi);
    const auto clear_pts_self = EcbsSolver::clear_points_from_range(
      working,
      earliest->self_overlap_lo,
      earliest->self_overlap_hi);

    // Dependencies on both paths (wait_point + clear_pts) — always mutual,
    // EXCEPT when leader/follower: the leader gets no wait tag.
    if (!other_is_follower && earliest->self_wait_wp < working.waypoints.size()) {
      add_wait_dependency(
        working.waypoints[earliest->self_wait_wp],
        earliest_other->amr_id,
        clear_pts_other);
    }
    if (!self_is_follower && !other_inside_base && earliest->other_wait_wp < earliest_other->waypoints.size()) {
      add_wait_dependency(
        earliest_other->waypoints[earliest->other_wait_wp],
        working.amr_id,
        clear_pts_self);
    }
    modified_others.insert(earliest->other_amr);

    // Mutual provisional holds (not FCFS): both AMRs get wait_sec at their
    // wait_points when possible. Conflict clear still uses a time shift so
    // the committed schedule can become conflict-free; the other side also
    // receives at least its computed wait_other (or a capped provisional).
    const double room_self =
      config_.max_wait_sec - total_wait_self;

    if (self_can) {
      delay_at_wait_wp(
        working, earliest->self_wait_wp, wait_self,
        earliest_other->amr_id, clear_pts_other);
      total_wait_self += wait_self;

      const double self_clear = corridor_clear_sec(
        working, earliest->self_overlap_lo, earliest->self_overlap_hi);
      const double other_arrive =
        earliest_other->waypoints[earliest->other_overlap_lo].t_sec;
      const double delta_other =
        self_clear + config_.safety_margin_sec - other_arrive;
      // At least wait_other when other_can; else whatever clears conflict.
      double apply_other = 0.0;
      if (other_can) {
        apply_other = std::max(delta_other, wait_other);
      } else if (delta_other > 1e-3) {
        apply_other = delta_other;
      }
      if (!other_owns_base && apply_other > 1e-3 &&
        (total_wait_other[earliest->other_amr] + apply_other) <=
          config_.max_wait_sec)
      {
        delay_at_wait_wp(
          *earliest_other, earliest->other_wait_wp, apply_other,
          working.amr_id, clear_pts_self);
        total_wait_other[earliest->other_amr] += apply_other;
      } else if (!other_inside_base) {
        // Other cannot be time-shifted within max_wait: still hand ZM its
        // hold time (if it were the one to yield), capped at max_wait.
        set_provisional_wait(
          *earliest_other, earliest->other_wait_wp, working.amr_id,
          std::min(std::max(wait_other, apply_other), config_.max_wait_sec));
      }
    } else if (other_can) {
      delay_at_wait_wp(
        *earliest_other, earliest->other_wait_wp, wait_other,
        working.amr_id, clear_pts_self);
      total_wait_other[earliest->other_amr] += wait_other;

      set_provisional_wait(
        working, earliest->self_wait_wp, earliest_other->amr_id,
        std::min(std::max(0.0, wait_self), room_self));
    }

    // Re-scan working against every entry in others_mut with these times.
    // Commit happens only on the no-conflict path above.
    continue;
  }

  result.status = SolveStatus::FailReplan;
  result.reason = "exceeded max ECBS iterations while conflicts remain";
  return result;
}

//==============================================================================
SolverConfig load_solver_config_yaml(
  const std::string & yaml_path, const SolverConfig & fallback)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error(
      "cannot read ECBS params '" + yaml_path + "': " + e.what());
  }
  const YAML::Node params = root["evaluate_path_server"]["ros__parameters"];
  if (!params || !params.IsMap()) {
    throw std::runtime_error(
      "ECBS params '" + yaml_path +
      "' has no evaluate_path_server.ros__parameters block");
  }

  SolverConfig cfg = fallback;
  const auto read = [&](const char * key, auto & value) {
      const YAML::Node node = params[key];
      if (!node || node.IsNull()) {
        return;  // keep the fallback
      }
      try {
        value = node.as<std::decay_t<decltype(value)>>();
      } catch (const YAML::Exception &) {
        throw std::runtime_error(
          "ECBS params '" + yaml_path + "': '" + key + "' is not a number");
      }
    };
  read("safety_margin_sec", cfg.safety_margin_sec);
  read("max_wait_sec", cfg.max_wait_sec);
  read("max_iters", cfg.max_iters);
  read("position_epsilon_m", cfg.position_epsilon_m);
  read("safety_distance_m", cfg.safety_distance_m);
  read("evaluate_lookahead_margin_m", cfg.evaluate_lookahead_margin_m);
  read("clearance_m", cfg.clearance_m);
  read("amr_length_m", cfg.amr_length_m);
  read("amr_width_m", cfg.amr_width_m);
  read("junction_release_m", cfg.junction_release_m);
  return cfg;
}

}  // namespace ecbs_planner
