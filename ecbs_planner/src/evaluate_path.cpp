#include "ecbs_planner/evaluate_path.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <ecbs_planner/msg/clear_point.hpp>
#include <ecbs_planner/msg/evaluate_path_waypoint.hpp>
#include <ecbs_planner/msg/from_zone_manager.hpp>
#include <ecbs_planner/msg/path_point.hpp>
#include <ecbs_planner/msg/to_zone_manager.hpp>
#include <ecbs_planner/msg/wait_dependency.hpp>
#include <ecbs_planner/msg/wait_point.hpp>
#include <ecbs_planner/msg/register_update.hpp>
#include <ecbs_planner/path_topology.hpp>
#include <ecbs_planner/path_utils.hpp>
#include <ecbs_planner/replan_candidate.hpp>
#include <ecbs_planner/wait_reason.hpp>

#include "builtin_interfaces/msg/time.hpp"

#include "monitor/eta_path_calculator.hpp"

#include <utility>
#include <vector>

namespace ecbs_planner
{
namespace
{

// Forward decls — used by conflict collection before their definitions.
std::size_t outside_given_base(
  const TimedPath & path,
  std::size_t wait_i,
  std::optional<std::size_t> granted_i);
bool apply_junction_hard_rule(
  const TimedPath & path,
  std::size_t start,
  double safety_m,
  const NavGraphJunctions * junctions,
  std::size_t & wait_i,
  PathWaitPoint & wp);
WaitDependency dep_from_msg(const msg::WaitDependency & m);


bool waypoint_has_wait(const TimedPoint & wp)
{
  return wp.wait_duration_sec > 1e-6 || !wp.waiting_for.empty();
}

std::string normalize_mode(
  const std::string & mode, bool has_waypoints, bool has_updates)
{
  if (mode == "register" || mode == "evaluate") {
    return mode;
  }
  if (has_updates || has_waypoints) {
    return "register";
  }
  return "evaluate";
}

double effective_query_horizon_m(double query_horizon_m)
{
  return (query_horizon_m > 0.0) ? query_horizon_m : 20.0;
}

/// Inclusive array indices covering up to horizon_m metres from start_i.
void horizon_index_range(
  const TimedPath & path,
  std::size_t start_i,
  double horizon_m,
  std::size_t & out_lo,
  std::size_t & out_hi,
  double & out_travelled_m)
{
  out_lo = start_i;
  out_hi = start_i;
  out_travelled_m = 0.0;
  if (path.waypoints.empty() || start_i >= path.waypoints.size()) {
    return;
  }
  horizon_m = effective_query_horizon_m(horizon_m);
  for (std::size_t i = start_i + 1; i < path.waypoints.size(); ++i) {
    const auto & a = path.waypoints[i - 1];
    const auto & b = path.waypoints[i];
    out_travelled_m += std::hypot(b.x - a.x, b.y - a.y);
    out_hi = i;
    if (out_travelled_m >= horizon_m) {
      break;
    }
  }
}

/// Prefer graph/index field; else fall back to array position.
int64_t waypoint_index_value(const TimedPoint & wp, std::size_t array_i)
{
  if (wp.index.has_value()) {
    return static_cast<int64_t>(*wp.index);
  }
  return static_cast<int64_t>(array_i);
}

msg::WaitPoint to_msg_wait_point(const PathWaitPoint & p)
{
  msg::WaitPoint out;
  out.x = p.x;
  out.y = p.y;
  out.index = p.index.has_value() ?
    static_cast<int64_t>(*p.index) : static_cast<int64_t>(-1);
  out.wait_sec = p.wait_duration_sec;
  return out;
}

PathWaitPoint from_msg_wait_point(const msg::WaitPoint & p)
{
  PathWaitPoint out;
  out.x = p.x;
  out.y = p.y;
  if (p.index >= 0) {
    out.index = static_cast<uint64_t>(p.index);
  }
  out.wait_duration_sec = p.wait_sec;
  return out;
}

PathWaitPoint safe_wait_from_wp(const TimedPoint & wp)
{
  PathWaitPoint p;
  p.x = wp.x;
  p.y = wp.y;
  p.index = wp.index;
  p.wait_duration_sec = wp.wait_duration_sec;
  return p;
}

bool wait_point_is_set(const PathWaitPoint & p)
{
  return p.wait_duration_sec > 1e-6 || p.index.has_value() ||
    std::abs(p.x) > 1e-9 || std::abs(p.y) > 1e-9;
}

msg::WaitDependency to_msg_dep(const WaitDependency & dep)
{
  msg::WaitDependency d;
  d.amr_id = dep.amr_id;
  d.wait_point = to_msg_wait_point(dep.wait_point);
  for (const auto & c : dep.clear_points) {
    msg::ClearPoint cp;
    cp.x = c.x;
    cp.y = c.y;
    d.clear_points.push_back(cp);
  }
  d.topology = dep.topology;
  d.self_blocks_other = dep.self_blocks_other;
  d.other_blocks_self = dep.other_blocks_self;
  return d;
}

const char * topo_string(PathTopology t)
{
  switch (t) {
    case PathTopology::Opposite: return kTopoOpposite;
    case PathTopology::BackToBack: return kTopoSameWay;
    case PathTopology::Plus: return kTopoCross;
    case PathTopology::TJunction: return kTopoMerge;
    case PathTopology::Unknown: return "";
  }
  return "";
}

/// Classify overlap + easiness flags for TM (timed conflicts).
void fill_dep_geometry(
  WaitDependency & dep,
  const TimedPath & self,
  std::size_t clo,
  std::size_t chi,
  const TimedPath & other,
  std::size_t other_lo,
  std::size_t other_hi,
  double eps_m,
  double safety_m = 0.0,
  double amr_half_diag = 0.0)
{
  dep.topology = topo_string(
    classify_path_overlap(self, clo, chi, other, other_lo, other_hi, eps_m));
  path_easiness_flags(
    self, clo, other, other_lo, eps_m,
    dep.self_blocks_other, dep.other_blocks_self,
    safety_m, amr_half_diag);
}

void set_dep_ownership(WaitDependency & dep)
{
  dep.topology = kTopoOwnership;
  dep.self_blocks_other = false;
  dep.other_blocks_self = false;
}

void merge_dep(
  std::unordered_map<std::string, WaitDependency> & by_amr,
  const WaitDependency & dep)
{
  if (dep.amr_id.empty()) {
    return;
  }
  auto & dst = by_amr[dep.amr_id];
  dst.amr_id = dep.amr_id;
  if (!wait_point_is_set(dst.wait_point) && wait_point_is_set(dep.wait_point)) {
    dst.wait_point = dep.wait_point;
  } else if (dep.wait_point.wait_duration_sec > dst.wait_point.wait_duration_sec) {
    dst.wait_point = dep.wait_point;
  }
  append_unique_clear_points(dst.clear_points, dep.clear_points);
  if (dst.topology.empty()) {
    dst.topology = dep.topology;
  } else if (!dep.topology.empty() &&
    wait_reason_rank(wait_reason_from_topology(dep.topology)) >
    wait_reason_rank(wait_reason_from_topology(dst.topology)))
  {
    dst.topology = dep.topology;
  }
  dst.self_blocks_other = dst.self_blocks_other || dep.self_blocks_other;
  dst.other_blocks_self = dst.other_blocks_self || dep.other_blocks_self;
}

void fill_identity(
  const srv::EvaluatePath::Request & req,
  srv::EvaluatePath::Response & res)
{
  res.amr_id = req.amr_id;
  res.task_id = req.task_id;
  res.path_revision = req.path_revision;
  res.waiting_for.clear();
  res.horizon_m = 0.0;
  res.dep_from_index = -1;
  res.dep_to_index = -1;
  res.success = false;
  res.status = "BLOCKED";
  res.message.clear();
  res.wait_reason.clear();
}

void respond_free(
  srv::EvaluatePath::Response & res,
  double query_horizon_m,
  const std::string & message)
{
  res.success = true;
  res.status = "FREE";
  res.horizon_m = effective_query_horizon_m(query_horizon_m);
  res.dep_from_index = -1;
  res.dep_to_index = -1;
  res.waiting_for.clear();
  res.wait_reason.clear();
  res.message = message;
}

void respond_wait(
  srv::EvaluatePath::Response & res,
  double blocked_m,
  int64_t dep_from,
  int64_t dep_to,
  const std::unordered_map<std::string, WaitDependency> & deps,
  const std::string & message)
{
  res.success = true;
  res.status = "WAIT";
  res.horizon_m = blocked_m;
  res.dep_from_index = dep_from;
  res.dep_to_index = dep_to;
  res.waiting_for.clear();
  res.wait_reason.clear();
  for (const auto & kv : deps) {
    res.waiting_for.push_back(to_msg_dep(kv.second));
    strengthen_wait_reason(
      res.wait_reason, wait_reason_from_topology(kv.second.topology));
  }
  if (res.wait_reason.empty()) {
    res.wait_reason = infer_wait_reason_from_message(message);
  }
  res.message = message;
}

/// Build lean WAIT from waits already on path in [lo, hi].
/// If \p only_at_base_end (evaluate / base-ownership): ignore holds with
/// array index i < hi; only a wait at the base end is reported to ZM.
bool collect_registered_waits(
  const TimedPath & path,
  std::size_t lo,
  std::size_t hi,
  int64_t & dep_from,
  int64_t & dep_to,
  std::unordered_map<std::string, WaitDependency> & deps,
  bool only_at_base_end)
{
  bool any = false;
  dep_from = -1;
  dep_to = -1;
  if (path.waypoints.empty()) {
    return false;
  }
  hi = std::min(hi, path.waypoints.size() - 1);
  for (std::size_t i = lo; i <= hi; ++i) {
    const auto & wp = path.waypoints[i];
    if (!waypoint_has_wait(wp)) {
      continue;
    }
    if (only_at_base_end && i < hi) {
      continue;
    }
    any = true;
    const int64_t idx = waypoint_index_value(wp, i);
    if (dep_from < 0 || idx < dep_from) {
      dep_from = idx;
    }
    if (dep_to < 0 || idx > dep_to) {
      dep_to = idx;
    }
    for (const auto & dep : wp.waiting_for) {
      WaitDependency copy = dep;
      if (!wait_point_is_set(copy.wait_point)) {
        copy.wait_point = safe_wait_from_wp(wp);
      }
      if (only_at_base_end) {
        const double hold = copy.wait_point.wait_duration_sec > 1e-6 ?
          copy.wait_point.wait_duration_sec : wp.wait_duration_sec;
        copy.wait_point = safe_wait_from_wp(path.waypoints[hi]);
        copy.wait_point.wait_duration_sec = hold;
      }
      merge_dep(deps, copy);
    }
  }
  return any;
}

/// Detect conflicts vs cached AMRs for the evaluate base [lo, hi].
/// Only overlaps starting at or before \p look_hi (horizon + look-ahead
/// margin) count; later ones are left to a later evaluate.
///
/// Base-ownership applies only to a base already given (\p granted_i):
/// - Conflict fully inside the given base → no wait for the owner.
/// - Any other conflict (no base given yet, or spills past it) → WAIT.
///   Wait point:
///   * overlap starts inside the already given base (\p granted_i) → first
///     waypoint of the next base;
///   * overlap after the given base → that base end (never inside it);
///   * no base given → the waypoint \p safety_m before the overlap, never
///     behind the robot (\p lo) and never past \p hi.
/// Cosine between \p path's way into its waypoint \p i and \p other's path
/// where it is nearest that waypoint (0 if either has no direction there).
double heading_cos_at(const TimedPath & path, std::size_t i, const TimedPath & other)
{
  const auto & o = other.waypoints;
  if (i == 0 || i >= path.waypoints.size() || o.size() < 2) {
    return 0.0;
  }
  const auto & w = path.waypoints[i];
  std::size_t best = 1;
  double best_d2 = dist2_to_segment(w.x, w.y, o[0], o[1]);
  for (std::size_t j = 2; j < o.size(); ++j) {
    const double d2 = dist2_to_segment(w.x, w.y, o[j - 1], o[j]);
    if (d2 < best_d2) {
      best_d2 = d2;
      best = j;
    }
  }
  const auto & p = path.waypoints[i - 1];
  const double ax = w.x - p.x;
  const double ay = w.y - p.y;
  const double bx = o[best].x - o[best - 1].x;
  const double by = o[best].y - o[best - 1].y;
  const double len = std::hypot(ax, ay) * std::hypot(bx, by);
  return len > 1e-12 ? (ax * bx + ay * by) / len : 0.0;
}

/// True if \p other's path, where it is nearest \p path's waypoint \p i,
/// runs the same way as \p path does into that waypoint.
bool same_heading_at(const TimedPath & path, std::size_t i, const TimedPath & other)
{
  return heading_cos_at(path, i, other) > 0.0;
}

/// True if \p other's path comes at \p path head-on at \p path's waypoint
/// \p i (from its first segment if \p i is the start).
bool head_on_at(const TimedPath & path, std::size_t i, const TimedPath & other)
{
  return heading_cos_at(path, std::max<std::size_t>(i, 1), other) < -0.5;
}

/// True if \p leader (at waypoint \p from) is on \p follower's way ahead of
/// it and drives on the same way: the follower waits behind it, the leader
/// never waits for it.
bool leads(
  const TimedPath & leader, std::size_t from, const TimedPath & follower, double eps_m)
{
  if (from + 1 >= leader.waypoints.size() || follower.waypoints.size() < 2) {
    return false;
  }
  const auto & at = leader.waypoints[from];
  return near_path(at.x, at.y, follower, eps_m) &&
         nearest_waypoint_index(follower, at.x, at.y) > 0 &&
         same_heading_at(leader, from + 1, follower);
}

bool collect_cache_conflicts(
  const TimedPath & self,
  std::size_t lo,
  std::size_t hi,
  std::size_t look_hi,
  std::optional<std::size_t> granted_i,
  double safety_m,
  const ScheduleCache & cache,
  double eps_m,
  int64_t & dep_from,
  int64_t & dep_to,
  std::unordered_map<std::string, WaitDependency> & deps,
  double amr_half_diag = 0.0,
  const NavGraphJunctions * junctions = nullptr)
{
  bool any = false;
  if (self.waypoints.empty()) {
    return false;
  }
  hi = std::min(hi, self.waypoints.size() - 1);
  // A wait gives the robot up to it: never past this horizon, nor past a
  // longer base already given. A short base (e.g. one given while the robot
  // had nowhere to go) does not pull the wait back from the conflict.
  const std::size_t wait_cap = granted_i ? std::max(*granted_i, hi) : hi;

  for (const auto & other : cache.all_except(self.amr_id)) {
    // ZM registered `other` waiting for self → self is GO for this pair.
    if (cache.registered_waiter(other.amr_id, self.amr_id)) {
      continue;
    }
    // One of the pair stands waiting before the other's base or body: its
    // times are stale (that wait shifts none), so they say nothing about
    // meeting. The reserved stretch / standing pose decides instead.
    if (cache.reserve_waits(other.amr_id).count(self.amr_id) > 0 ||
      cache.reserve_waits(self.amr_id).count(other.amr_id) > 0)
    {
      continue;
    }
    if (leads(self, lo, other, eps_m)) {
      continue;
    }
    const auto hit = EcbsSolver::earliest_conflict(self, other, eps_m);
    // Body-size-only clashes (no path/time hit) are handled via reserved
    // stretch / standing pose (wait_before_reserved detect_m), not here —
    // otherwise a far-future parallel path makes the owner WAIT for a waiter.
    if (!hit.has_value()) {
      continue;
    }
    const std::size_t clo = hit->self_overlap_lo;
    const std::size_t chi = hit->self_overlap_hi;
    const std::size_t other_lo = hit->other_overlap_lo;
    const std::size_t other_hi = hit->other_overlap_hi;

    // Ignore overlaps fully behind the robot or starting past the look-ahead.
    if (chi < lo || clo > look_hi) {
      continue;
    }

    // --- Leader / follower rule -------------------------------------------
    // Compute heading of self and other at the conflict point.
    // If other's heading has a positive dot product with self's heading,
    // other is approaching from the same / behind direction → other is the
    // follower → self (the leader) must NOT receive a wait here.
    // The follower (other) will get its own wait when it calls evaluate.
    {
      double shx = 1.0, shy = 0.0, ohx = 1.0, ohy = 0.0;
      const std::size_t si = clo > 0 ? clo : 1;
      heading_at(self,  si,                          shx, shy);
      heading_at(other, other_lo > 0 ? other_lo : 1, ohx, ohy);
      const bool same_direction = (shx * ohx + shy * ohy) > 0.0;
      if (same_direction) {
        const double self_t = self.waypoints[clo].t_sec;
        const double other_t = other.waypoints[other_lo].t_sec;
        const bool self_is_leader = (self_t <= other_t + 1e-3) ||
          leads(self, lo, other, eps_m);
        if (self_is_leader) {
          continue;  // Self is the leader; it does not wait for the follower.
        }
      }
    }

    // Fully inside the base already given → owner has priority.
    if (granted_i && chi <= *granted_i) {
      continue;
    }
    // Starts inside it and runs on: crossing or same way → the owner is in
    // first and has priority (the other AMR waits before its reserved
    // stretch); head-on → wait at the base end. Never inside a base already
    // given, never past its end (a WAIT gives the robot up to its wait
    // point: one past the end would move the base and the wait on by one at
    // every evaluate).
    std::size_t wait_i = wait_cap;
    if (granted_i && clo < *granted_i) {
      if (!head_on_at(self, clo, other)) {
        continue;  // Owner priority (path or body dash inside given base).
      }
      // Head-on into a given base: hold at base end (never past it).
      wait_i = outside_given_base(self, clo, granted_i);
    } else {
      wait_i = std::min(
        clear_wait_index(self, clo, lo, safety_m, other, eps_m, amr_half_diag),
        wait_cap);
      // C5: do not raise wait_i to granted_i past the safety stop.
    }

    any = true;
    const int64_t from_i = waypoint_index_value(self.waypoints[clo], clo);
    const int64_t to_i = waypoint_index_value(self.waypoints[chi], chi);
    if (dep_from < 0 || from_i < dep_from) {
      dep_from = from_i;
    }
    if (dep_to < 0 || to_i > dep_to) {
      dep_to = to_i;
    }

    WaitDependency dep;
    dep.amr_id = other.amr_id;
    dep.wait_point = safe_wait_from_wp(self.waypoints[wait_i]);
    // Never leave the wait on a T/cross centre — always safety_m before it.
    apply_junction_hard_rule(
      self, lo, safety_m, junctions, wait_i, dep.wait_point);
    dep.clear_points = EcbsSolver::clear_points_from_range(
      other, other_lo, other_hi);
    fill_dep_geometry(
      dep, self, clo, chi, other, other_lo, other_hi,
      eps_m, safety_m, amr_half_diag);
    merge_dep(deps, dep);
  }
  return any;
}

void respond_from_path_status(
  srv::EvaluatePath::Response & res,
  const TimedPath & path,
  std::size_t lo,
  std::size_t hi,
  std::size_t look_hi,
  std::optional<std::size_t> granted_i,
  double safety_m,
  double query_horizon_m,
  const ScheduleCache & cache,
  double eps_m,
  const std::string & message_prefix,
  double amr_half_diag = 0.0,
  const NavGraphJunctions * junctions = nullptr)
{
  std::unordered_map<std::string, WaitDependency> deps;
  int64_t dep_from = -1;
  int64_t dep_to = -1;

  bool has_waits = collect_registered_waits(
    path, lo, hi, dep_from, dep_to, deps, true /* only_at_base_end */);
  // ZM-registered waits must apply anywhere in the look-ahead — not only at
  // the base-end index. Otherwise register(wait beyond hi) → FREE, the robot
  // drives past the wait, then reserved-base creates a mutual deadlock.
  {
    std::unordered_map<std::string, WaitDependency> reg_deps;
    int64_t rf = -1;
    int64_t rt = -1;
    if (collect_registered_waits(
        path, lo, look_hi, rf, rt, reg_deps, false /* any wait in window */))
    {
      for (auto & kv : reg_deps) {
        // A wait not yet registered by ZM is still a hold: never FREE past it.
        // Stale ones were dropped earlier (passed / yielding / behind robot).
        if (!cache.registered_waiter(path.amr_id, kv.first) &&
          !cache.has_hold(path.amr_id, kv.first))
        {
          continue;
        }
        merge_dep(deps, kv.second);
        has_waits = true;
        if (rf >= 0 && (dep_from < 0 || rf < dep_from)) {
          dep_from = rf;
        }
        if (rt >= 0 && (dep_to < 0 || rt > dep_to)) {
          dep_to = rt;
        }
      }
    }
  }
  const bool has_conflicts = collect_cache_conflicts(
    path, lo, hi, look_hi, granted_i, safety_m, cache, eps_m,
    dep_from, dep_to, deps, amr_half_diag, junctions);

  if (!has_waits && !has_conflicts) {
    // Base ownership is checked first (respond_reserved_waits). Never wipe a
    // reserved-base WAIT with this FREE — time/path silence must not override.
    if (res.status != "WAIT") {
      respond_free(res, query_horizon_m, message_prefix.empty() ?
        "FREE; base clear or owner priority inside base — do not register" :
        message_prefix);
    }
    return;
  }

  // Merge path/time deps onto any base WAIT already on the response.
  if (res.status == "WAIT") {
    for (const auto & mdep : res.waiting_for) {
      merge_dep(deps, dep_from_msg(mdep));
    }
  }

  double blocked_m = 0.0;
  if (dep_from >= 0 && dep_to >= 0) {
    std::size_t blo = 0;
    std::size_t bhi = path.waypoints.empty() ? 0 : path.waypoints.size() - 1;
    bool found_lo = false;
    bool found_hi = false;
    for (std::size_t i = 0; i < path.waypoints.size(); ++i) {
      const int64_t idx = waypoint_index_value(path.waypoints[i], i);
      if (idx == dep_from) {
        blo = i;
        found_lo = true;
      }
      if (idx == dep_to) {
        bhi = i;
        found_hi = true;
      }
    }
    if (found_lo && found_hi) {
      blocked_m = path_length_between(path, blo, bhi);
    }
  }
  if (blocked_m <= 0.0) {
    blocked_m = path_length_between(path, lo, hi);
  }

  respond_wait(
    res, blocked_m, dep_from, dep_to, deps,
    message_prefix.empty() ?
      "WAIT; conflict past allotted base — hold at wait_point "
      "(safety distance before overlap, or first point of next base); ZM must decide then register" :
      message_prefix);
}

void attach_mutual_dep(
  TimedPoint & wait_wp,
  const std::string & blocker_amr,
  const std::vector<ClearPoint> & clear_pts)
{
  PathWaitPoint safe = safe_wait_from_wp(wait_wp);
  for (auto & dep : wait_wp.waiting_for) {
    if (dep.amr_id == blocker_amr) {
      append_unique_clear_points(dep.clear_points, clear_pts);
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

void delay_path_at(
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
  attach_mutual_dep(path.waypoints[wait_at], blocker_amr, clear_pts);
  for (auto & dep : path.waypoints[wait_at].waiting_for) {
    if (dep.amr_id == blocker_amr) {
      dep.wait_point = safe_wait_from_wp(path.waypoints[wait_at]);
    }
  }
  for (std::size_t k = wait_at + 1; k < path.waypoints.size(); ++k) {
    path.waypoints[k].t_sec += delta_t;
  }
  if (wait_at == 0) {
    path.waypoints[0].t_sec += delta_t;
  }
}

/// Undo prior ECBS/ZM holds so a new register decision is not double-applied.
void strip_existing_waits(TimedPath & path)
{
  for (std::size_t i = 0; i < path.waypoints.size(); ++i) {
    const double w = path.waypoints[i].wait_duration_sec;
    if (w > 1e-9) {
      for (std::size_t k = i + 1; k < path.waypoints.size(); ++k) {
        path.waypoints[k].t_sec -= w;
      }
      if (i == 0) {
        path.waypoints[0].t_sec -= w;
      }
      path.waypoints[i].wait_duration_sec = 0.0;
    }
    path.waypoints[i].waiting_for.clear();
  }
}

std::size_t resolve_wait_at(
  const TimedPath & path, const msg::WaitPoint & wp)
{
  // XY first — matches viz / TM release-cap. Graph index can be stale (same
  // idx as the T wait while x,y still sits one step earlier in the base).
  if (std::hypot(wp.x, wp.y) > 1e-9 || wp.index < 0) {
    return nearest_waypoint_index(path, wp.x, wp.y);
  }
  if (wp.index >= 0) {
    const auto want = static_cast<uint64_t>(wp.index);
    for (std::size_t i = 0; i < path.waypoints.size(); ++i) {
      if (path.waypoints[i].index.has_value() &&
        *path.waypoints[i].index == want)
      {
        return i;
      }
    }
    if (static_cast<std::size_t>(wp.index) < path.waypoints.size()) {
      return static_cast<std::size_t>(wp.index);
    }
  }
  return nearest_waypoint_index(path, wp.x, wp.y);
}

WaitDependency dep_from_msg(const msg::WaitDependency & m)
{
  WaitDependency d;
  d.amr_id = m.amr_id;
  d.wait_point = from_msg_wait_point(m.wait_point);
  for (const auto & c : m.clear_points) {
    ClearPoint cp;
    cp.x = c.x;
    cp.y = c.y;
    d.clear_points.push_back(cp);
  }
  d.topology = m.topology;
  d.self_blocks_other = m.self_blocks_other;
  d.other_blocks_self = m.other_blocks_self;
  return d;
}

/// Make the robot's cached path (and /to_zone_manager) match an evaluate
/// answer. Within the checked window (horizon + look-ahead margin from the
/// Upsert evaluate/PathIntent request geometry into ScheduleCache, then attach
/// only the waits this response answered. Prefer request waypoints over any
/// prior /from_zone_manager sample so BaseExtensionQuery indices match the
/// route TM/full_control execute (index divergence blocked PathRelease).
/// A responded wait with no seconds takes the seconds of the wait it replaces
/// (response is updated too). Returns false if the request has no waypoints.
bool sync_cache_with_evaluate(
  const srv::EvaluatePath::Request & req,
  srv::EvaluatePath::Response & res,
  double lookahead_margin_m,
  ScheduleCache & cache,
  std::vector<TimedPath> * committed_out)
{
  if (req.waypoints.empty()) {
    return false;
  }
  (void)lookahead_margin_m;

  // Preserve prior wait durations by blocker id when the response omits them.
  std::unordered_map<std::string, double> replaced_sec;
  if (const auto cached = cache.get(req.amr_id)) {
    for (const auto & wp : cached->waypoints) {
      for (const auto & d : wp.waiting_for) {
        auto & sec = replaced_sec[d.amr_id];
        sec = std::max(
          sec, std::max(d.wait_point.wait_duration_sec, wp.wait_duration_sec));
      }
    }
  }

  TimedPath path = timed_path_from_evaluate_request(req);
  ensure_route_occurrences(path);  // immutable indices for TM grant tips
  for (auto & wp : path.waypoints) {
    wp.waiting_for.clear();
    wp.wait_duration_sec = 0.0;
  }

  for (auto & mdep : res.waiting_for) {
    if (mdep.amr_id.empty()) {
      continue;
    }
    const std::size_t wait_at = resolve_wait_at(path, mdep.wait_point);
    WaitDependency dep = dep_from_msg(mdep);
    if (!wait_point_is_set(dep.wait_point)) {
      dep.wait_point = safe_wait_from_wp(path.waypoints[wait_at]);
    }
    const auto prev = replaced_sec.find(mdep.amr_id);
    if (dep.wait_point.wait_duration_sec <= 1e-6 && prev != replaced_sec.end() &&
      prev->second > 1e-6)
    {
      dep.wait_point.wait_duration_sec = prev->second;
      mdep.wait_point.wait_sec = prev->second;
    }
    path.waypoints[wait_at].waiting_for.push_back(std::move(dep));
  }

  cache.upsert(path);
  // Path.waiting_for is enough for BaseExtensionQuery; do not add_plan_holds
  // here — that would retain stale holds across FREE re-evals (same task_id).
  if (committed_out) {
    committed_out->push_back(path);
  }
  return true;
}

/// Shared cells of \p self with \p other by position only (times ignored):
/// first shared waypoint at or after \p from and the contiguous run after it.
bool spatial_overlap(
  const TimedPath & self,
  const TimedPath & other,
  std::size_t from,
  double eps_m,
  std::size_t & out_lo,
  std::size_t & out_hi)
{
  const double eps2 = eps_m * eps_m;
  auto shared = [&](std::size_t i) {
      for (const auto & o : other.waypoints) {
        const double dx = self.waypoints[i].x - o.x;
        const double dy = self.waypoints[i].y - o.y;
        if (dx * dx + dy * dy <= eps2) {
          return true;
        }
      }
      return false;
    };
  for (std::size_t i = from; i < self.waypoints.size(); ++i) {
    if (!shared(i)) {
      continue;
    }
    out_lo = i;
    out_hi = i;
    while (out_hi + 1 < self.waypoints.size() && shared(out_hi + 1)) {
      ++out_hi;
    }
    return true;
  }
  return false;
}

/// \p path shares cells with \p other (at or after \p from) only inside the
/// base already given (\p granted_i), none after its end: the base owner has
/// priority there and gets no wait for \p other.
bool overlap_inside_given_base(
  const TimedPath & path,
  const TimedPath & other,
  std::size_t from,
  std::optional<std::size_t> granted_i,
  double eps_m)
{
  if (!granted_i || other.waypoints.empty()) {
    return false;
  }
  std::size_t lo = 0;
  std::size_t hi = 0;
  if (!spatial_overlap(path, other, from, eps_m, lo, hi) || lo > *granted_i) {
    return false;
  }
  return !spatial_overlap(path, other, *granted_i + 1, eps_m, lo, hi);
}

/// The shared stretch of \p path and \p other (at or after \p from) starts
/// inside the base already given (\p granted_i) and \p other does not come
/// at it head-on there (crossing or same way): the base owner is in first
/// and has priority, \p other waits before its reserved stretch.
bool owns_overlap_start(
  const TimedPath & path,
  const TimedPath & other,
  std::size_t from,
  std::optional<std::size_t> granted_i,
  double eps_m)
{
  if (!granted_i || other.waypoints.empty()) {
    return false;
  }
  std::size_t lo = 0;
  std::size_t hi = 0;
  return spatial_overlap(path, other, from, eps_m, lo, hi) && lo < *granted_i &&
         !head_on_at(path, lo, other);
}

/// Keep the calculated safe wait. A stop before \p granted_i is an authority
/// conflict (C5) — do not push the wait forward to the base end past the
/// required separation. Never move the wait past the base end either.
std::size_t outside_given_base(
  const TimedPath & path,
  std::size_t wait_i,
  std::optional<std::size_t> granted_i)
{
  (void)granted_i;
  if (path.waypoints.empty()) {
    return wait_i;
  }
  return std::min(wait_i, path.waypoints.size() - 1);
}

/// Fill \p wp with the junction hard-rule wait (always safety_m before the
/// T/cross centre). Returns true if \p wp / \p wait_i were changed.
bool apply_junction_hard_rule(
  const TimedPath & path,
  std::size_t start,
  double safety_m,
  const NavGraphJunctions * junctions,
  std::size_t & wait_i,
  PathWaitPoint & wp)
{
  if (!junctions || junctions->empty() || path.waypoints.empty()) {
    return false;
  }
  auto snapped = junctions->snap_wait_point_off_junction(
    path, wait_i, wp.x, wp.y, start, safety_m);
  if (!snapped) {
    return false;
  }
  wait_i = snapped->first;
  wp.x = snapped->second.x;
  wp.y = snapped->second.y;
  wp.index = path.waypoints[wait_i].index;
  return true;
}

/// Hold of the robot on \p blocker_path for the robot on \p waiter_path, so
/// both robots of a conflict wait until ZM registers the pair. Uses the
/// blocker's existing wait for that robot (\p planned = true), else the
/// evaluate rule for the overlap (time conflict, else shared cells by
/// position): the safety point before the overlap, never inside a base
/// already given (head-on into it: its end). Waiter path unknown → hold at
/// \p lo; paths share no cell, share them only inside the blocker's given
/// base, or the blocker's base takes it into the shared stretch first
/// (crossing or same way: it has priority) → nullopt.
std::optional<WaitDependency> reciprocal_hold(
  const TimedPath & blocker_path,
  const TimedPath & waiter_path,
  std::size_t lo,
  std::optional<std::size_t> granted_i,
  double safety_m,
  double eps_m,
  bool & planned,
  std::size_t & wait_i,
  double amr_half_diag = 0.0)
{
  planned = false;
  WaitDependency dep;
  dep.amr_id = waiter_path.amr_id;
  const std::size_t n = blocker_path.waypoints.size();
  lo = std::min(lo, n - 1);
  if (overlap_inside_given_base(blocker_path, waiter_path, lo, granted_i, eps_m) ||
    owns_overlap_start(blocker_path, waiter_path, lo, granted_i, eps_m))
  {
    return std::nullopt;
  }

  // Evaluate rule for the overlap with the waiter.
  std::optional<std::size_t> rule_i;
  std::vector<ClearPoint> rule_clear;
  if (waiter_path.waypoints.empty()) {
    rule_i = lo;
  } else {
    std::size_t clo = 0;
    std::size_t chi = 0;
    bool overlap = true;
    const auto hit = EcbsSolver::earliest_conflict(blocker_path, waiter_path, eps_m);
    if (hit.has_value() && hit->self_overlap_hi >= lo) {
      clo = hit->self_overlap_lo;
      chi = hit->self_overlap_hi;
      rule_clear = EcbsSolver::clear_points_from_range(
        waiter_path, hit->other_overlap_lo, hit->other_overlap_hi);
    } else if (spatial_overlap(blocker_path, waiter_path, lo, eps_m, clo, chi)) {
      rule_clear = EcbsSolver::clear_points_from_range(blocker_path, clo, chi);
    } else {
      overlap = false;
    }
    if (overlap) {
      std::size_t at = clear_wait_index(
        blocker_path, clo, lo, safety_m, waiter_path, eps_m, amr_half_diag);
      // C5: do not push wait forward to granted_i past the safety stop.
      rule_i = std::min(std::max(at, lo), n - 1);
    }
  }

  for (std::size_t i = lo; i < n; ++i) {
    for (const auto & d : blocker_path.waypoints[i].waiting_for) {
      if (d.amr_id != waiter_path.amr_id) {
        continue;
      }
      dep = d;
      if (!wait_point_is_set(dep.wait_point)) {
        dep.wait_point = safe_wait_from_wp(blocker_path.waypoints[i]);
      }
      wait_i = outside_given_base(blocker_path, i, granted_i);
      // An existing wait further on (e.g. before the robot standing at its
      // start) is too late for a mutual hold: the pair would meet head-on.
      if (rule_i && *rule_i < wait_i) {
        wait_i = *rule_i;
        dep.clear_points = rule_clear;
      }
      planned = wait_i == i;
      if (!planned) {
        const double sec = dep.wait_point.wait_duration_sec;
        dep.wait_point = safe_wait_from_wp(blocker_path.waypoints[wait_i]);
        dep.wait_point.wait_duration_sec = sec;
      }
      return dep;
    }
  }

  if (!rule_i) {
    return std::nullopt;
  }
  wait_i = *rule_i;
  dep.clear_points = rule_clear;
  dep.wait_point = safe_wait_from_wp(blocker_path.waypoints[wait_i]);
  return dep;
}

std::size_t hold_index_on(
  const TimedPath & path, const PathWaitPoint & wp, std::size_t start);

/// Earliest waypoint (at or after \p lo) where \p path's robot already holds
/// for an AMR other than \p except.
std::optional<std::size_t> earliest_hold_index(
  const ScheduleCache & cache, const TimedPath & path, std::size_t lo,
  const std::string & except)
{
  std::optional<std::size_t> out;
  for (const auto & h : cache.holds(path.amr_id)) {
    if (h.amr_id == except) {
      continue;
    }
    const std::size_t i = hold_index_on(path, h.wait_point, lo);
    if (i >= lo && (!out || i < *out)) {
      out = i;
    }
  }
  return out;
}

/// One wait point per robot: if it already holds earlier on \p path than
/// \p wait_i, \p dep joins that wait point.
void join_earlier_hold(
  const ScheduleCache & cache, const TimedPath & path, std::size_t lo,
  WaitDependency & dep, bool & planned, std::size_t & wait_i)
{
  const auto earlier = earliest_hold_index(cache, path, lo, dep.amr_id);
  if (!earlier || *earlier >= wait_i) {
    return;
  }
  const double sec = dep.wait_point.wait_duration_sec;
  dep.wait_point = safe_wait_from_wp(path.waypoints[*earlier]);
  dep.wait_point.wait_duration_sec = sec;
  wait_i = *earlier;
  planned = false;
}

/// Remove every wait for \p waiter from \p path.
void strip_waiter(TimedPath & path, const std::string & waiter)
{
  for (auto & w : path.waypoints) {
    const auto before = w.waiting_for.size();
    w.waiting_for.erase(
      std::remove_if(
        w.waiting_for.begin(), w.waiting_for.end(),
        [&](const WaitDependency & d) {return d.amr_id == waiter;}),
      w.waiting_for.end());
    if (w.waiting_for.empty() && before > 0) {
      w.wait_duration_sec = 0.0;
    }
  }
}

/// Attach \p dep at \p wait_i on \p path (visible on /to_zone_manager).
void attach_hold(TimedPath & path, std::size_t wait_i, const WaitDependency & dep)
{
  attach_mutual_dep(path.waypoints[wait_i], dep.amr_id, dep.clear_points);
  for (auto & d : path.waypoints[wait_i].waiting_for) {
    if (d.amr_id == dep.amr_id) {
      d.wait_point = dep.wait_point;
      break;
    }
  }
}

/// \p blocker (cached, not the requester) holds for the robot on
/// \p waiter_path unless it already does or ZM registered the pair.
void hold_cached_blocker(
  ScheduleCache & cache,
  const std::string & blocker,
  const TimedPath & waiter_path,
  double safety_m,
  double eps_m,
  std::vector<TimedPath> * committed_out,
  double amr_half_diag = 0.0)
{
  const std::string & waiter = waiter_path.amr_id;
  if (blocker.empty() || blocker == waiter || is_junction_wait_id(blocker) ||
    cache.has_hold(blocker, waiter) ||
    cache.registered_waiter(blocker, waiter) || cache.registered_waiter(waiter, blocker) ||
    cache.has_right_of_way(blocker, waiter))
  {
    return;
  }
  auto bpath = cache.get(blocker);
  if (!bpath.has_value() || bpath->waypoints.empty() || leads(*bpath, 0, waiter_path, eps_m)) {
    return;
  }
  std::optional<std::size_t> granted_i;
  if (const auto base = cache.granted_base(blocker)) {
    granted_i = locate_granted_base(*bpath, *base, 0);
  }
  bool planned = false;
  std::size_t wait_i = 0;
  auto dep = reciprocal_hold(
    *bpath, waiter_path, 0, granted_i, safety_m, eps_m, planned, wait_i,
    amr_half_diag);
  if (!dep) {
    return;
  }
  join_earlier_hold(cache, *bpath, 0, *dep, planned, wait_i);
  cache.add_hold(blocker, *dep);
  if (planned) {
    return;
  }
  strip_waiter(*bpath, waiter);
  attach_hold(*bpath, wait_i, *dep);
  cache.upsert(*bpath);
  if (committed_out) {
    committed_out->push_back(*bpath);
  }
}

/// AMRs holding for \p self_path's robot that it does not yet hold for:
/// add the reverse hold on \p self_path (from \p lo). Returns true if
/// \p self_path changed.
bool hold_self_for_waiters(
  ScheduleCache & cache,
  TimedPath & self_path,
  std::size_t lo,
  std::optional<std::size_t> granted_i,
  double safety_m,
  double eps_m,
  bool attach,
  double amr_half_diag = 0.0)
{
  bool changed = false;
  if (self_path.waypoints.empty()) {
    return false;
  }
  for (const auto & waiter : cache.waiters_for(self_path.amr_id)) {
    if (cache.has_hold(self_path.amr_id, waiter) ||
      cache.has_right_of_way(self_path.amr_id, waiter))
    {
      continue;
    }
    TimedPath wpath;
    if (auto cached = cache.get(waiter)) {
      wpath = std::move(*cached);
    }
    wpath.amr_id = waiter;
    if (leads(self_path, lo, wpath, eps_m)) {
      continue;
    }
    bool planned = false;
    std::size_t wait_i = lo;
    auto dep = reciprocal_hold(
      self_path, wpath, lo, granted_i, safety_m, eps_m, planned, wait_i,
      amr_half_diag);
    if (!dep) {
      continue;
    }
    join_earlier_hold(cache, self_path, lo, *dep, planned, wait_i);
    cache.add_hold(self_path.amr_id, *dep);
    if (attach && !planned) {
      strip_waiter(self_path, waiter);
      attach_hold(self_path, wait_i, *dep);
      changed = true;
    }
  }
  return changed;
}

/// Array index of hold \p wp on \p path: nav-graph index match at or after
/// \p start, else one behind it, else the nearest waypoint by XY.
std::size_t hold_index_on(
  const TimedPath & path, const PathWaitPoint & wp, std::size_t start)
{
  const std::size_t n = path.waypoints.size();
  if (wp.index.has_value()) {
    for (std::size_t i = start; i < n; ++i) {
      if (path.waypoints[i].index.has_value() && *path.waypoints[i].index == *wp.index) {
        return i;
      }
    }
    for (std::size_t i = std::min(start, n); i-- > 0; ) {
      if (path.waypoints[i].index.has_value() && *path.waypoints[i].index == *wp.index) {
        return i;
      }
    }
  }
  return nearest_waypoint_index(path, wp.x, wp.y);
}

/// Path index of a wait for "inside given base" checks. Uses XY only — viz and
/// traffic_manager release-cap also key off wait_point x,y. A stale graph
/// index can match the base-end waypoint while XY still sits one step earlier
/// (seen when a new AMR conflict reused the T wait's index).
std::size_t hold_index_by_xy(
  const TimedPath & path, const PathWaitPoint & wp, std::size_t start)
{
  (void)start;
  if (path.waypoints.empty()) {
    return 0;
  }
  return nearest_waypoint_index(path, wp.x, wp.y);
}

/// Move wait \p at back along \p path (never before \p floor) only while an
/// L×W body at that point would still dash another AMR (\p blocker excepted:
/// the wait is meant to sit just before its route). Does not use clearance
/// footprint to push the wait further — that stays at safety_m unless bodies
/// overlap. Stays at \p at if nothing fits.
std::size_t clear_of_others(
  const ScheduleCache & cache,
  const TimedPath & path,
  std::size_t at,
  std::size_t floor,
  const std::string & blocker,
  double amr_half_diag)
{
  if (path.waypoints.empty() || at < floor || amr_half_diag <= 0.0) {
    return at;
  }
  const auto others = cache.all_except(path.amr_id);
  const double touch = 2.0 * amr_half_diag;
  for (std::size_t k = at + 1; k-- > floor; ) {
    bool dashes = false;
    for (const auto & o : others) {
      if (o.amr_id == blocker) {
        continue;
      }
      if (body_overlap_at(path, k, o, amr_half_diag)) {
        dashes = true;
        break;
      }
      // Standing / reserved pose of others (no full path): body vs point.
      if (const auto pose = cache.standing_pose(o.amr_id)) {
        const double dx = path.waypoints[k].x - pose->x;
        const double dy = path.waypoints[k].y - pose->y;
        if (dx * dx + dy * dy < touch * touch) {
          dashes = true;
          break;
        }
      }
    }
    if (!dashes) {
      return k;
    }
  }
  return at;
}

/// Fix the wait point of every hold of \p path's robot, once:
/// never inside a base already given (\p granted_i), never behind the robot
/// (\p start), not beside another AMR's reserved stretch or path, and never
/// on a T/cross centre. After this the wait point does not move with the
/// robot — except a pinned wait that still sits on a junction is corrected.
/// Junction sentinel holds (T / +) are never pinned — recomputed each evaluate.
void pin_holds(
  ScheduleCache & cache,
  const TimedPath & path,
  std::size_t start,
  std::optional<std::size_t> granted_i,
  double amr_half_diag,
  double safety_m = SolverConfig{}.safety_distance_m,
  const NavGraphJunctions * junctions = nullptr)
{
  for (const auto & dep : cache.holds(path.amr_id)) {
    if (is_junction_wait_id(dep.amr_id)) {
      continue;
    }
    // Already pinned: only correct a hard-rule violation (wait on T/cross
    // *centre*). Approach stops (≥ safety_m before) must stay put.
    if (cache.hold_pinned(path.amr_id, dep.amr_id)) {
      if (!junctions || junctions->empty()) {
        continue;
      }
      constexpr double kOnCentreM = 0.75;
      if (!junctions->near_junction(
          dep.wait_point.x, dep.wait_point.y, kOnCentreM))
      {
        continue;
      }
      std::size_t at = hold_index_on(path, dep.wait_point, start);
      PathWaitPoint wp = dep.wait_point;
      if (apply_junction_hard_rule(
          path, start, safety_m, junctions, at, wp))
      {
        WaitDependency fixed = dep;
        fixed.wait_point = wp;
        cache.force_hold_wait_point(path.amr_id, fixed);
      }
      continue;
    }
    // XY-first so a conflict wait with an earlier x,y but a base-end graph
    // index keeps the earlier safety stop (C5 authority conflict — do not
    // raise to granted tip).
    std::size_t at = hold_index_by_xy(path, dep.wait_point, start);
    at = std::max(at, start);
    at = outside_given_base(path, at, granted_i);
    at = std::max(at, start);
    const std::size_t floor = start;
    at = clear_of_others(cache, path, at, floor, dep.amr_id, amr_half_diag);
    for (const auto & h : cache.holds(path.amr_id)) {
      if (h.amr_id != dep.amr_id && cache.hold_pinned(path.amr_id, h.amr_id)) {
        const std::size_t i = hold_index_on(path, h.wait_point, start);
        if (i >= start) {
          at = std::min(at, i);
        }
      }
    }
    WaitDependency pinned = dep;
    pinned.wait_point = safe_wait_from_wp(path.waypoints[at]);
    pinned.wait_point.wait_duration_sec = dep.wait_point.wait_duration_sec;
    apply_junction_hard_rule(
      path, start, safety_m, junctions, at, pinned.wait_point);
    cache.pin_hold(path.amr_id, pinned);
  }
}

/// Batch register: only waiters in req.updates. Paths must already be cached.
/// GO AMRs = blockers named in waiting_for but not listed → clear their waits.
/// A base given to another AMR is its alone: if \p path (robot at \p start,
/// look-ahead to \p look_hi) comes within \p footprint_m of another AMR's
/// reserved stretch, answer WAIT before it, whatever the times. No hold: the
/// wait ends by itself once the stretch is out of the way. While still
/// blocked on a *crossing* / non-follower conflict, a wait never moves on
/// from where it was (\p prev_wait). Back-to-back followers are different:
/// their wait tracks the front pose at safety_m and is not pinned.
void respond_reserved_waits(
  srv::EvaluatePath::Response & res,
  ScheduleCache & cache,
  const TimedPath & path,
  std::size_t start,
  std::size_t look_hi,
  std::optional<std::size_t> granted_i,
  double safety_m,
  double footprint_m,
  double amr_half_diag,
  const std::unordered_map<std::string, std::size_t> & prev_wait)
{
  const auto own = cache.reserved(path.amr_id);
  std::unordered_map<std::string, WaitDependency> rdeps;
  std::unordered_set<std::string> met;
  std::size_t first = path.waypoints.size() - 1;
  for (const auto & kv : cache.reserved_except(path.amr_id)) {
    std::vector<ReservedCell> cells = kv.second;
    const bool standing = cache.standing(kv.first);
    // ZM registered kv.first waiting for this AMR: this AMR is GO. Skip their
    // reserved stretch entirely; only a body on our path can still block.
    const bool reg_waiter = cache.registered_waiter(kv.first, path.amr_id);
    if (reg_waiter && !standing) {
      const auto owner_at = cache.pose(kv.first);
      if (!owner_at ||
        !near_path(owner_at->x, owner_at->y, path, footprint_m + safety_m))
      {
        continue;  // Yielding AMR still approaching its wait — not our blocker.
      }
    }
    const bool body_only = waits_for(cache, kv.first, path.amr_id) || standing || reg_waiter;
    const auto owner_at = cache.pose(kv.first);
    if (body_only) {
      // Standing / waiting / yielding: only the place it stands on blocks —
      // not its reserved tail (that made waits ~7 m when a leader arrived).
      const auto body = cache.standing_pose(kv.first);
      if (body) {
        cells = {{body->x, body->y, 0}};
      } else if (owner_at) {
        cells = {{owner_at->x, owner_at->y, 0}};
      } else {
        continue;
      }
    }
    TimedPath owner_path;
    if (auto p = cache.get(kv.first)) {
      owner_path = std::move(*p);
    }
    // Single-pose path for body-overlap when the standing robot has no path.
    TimedPath stand_path = owner_path;
    if (stand_path.waypoints.empty() && owner_at) {
      TimedPoint wp;
      wp.x = owner_at->x;
      wp.y = owner_at->y;
      stand_path.waypoints.push_back(wp);
    }
    // A waiting robot that is still driving to its wait point owns the base it
    // was given up to there (reserved cells ahead of its pose): body-only would
    // let another AMR be granted base through it. Parked at its wait point it
    // has none ahead — only its body blocks.
    bool has_ahead = false;
    if (body_only && !reg_waiter && owner_at && !owner_path.waypoints.empty()) {
      for (const auto & c : kv.second) {
        if (std::hypot(c.x - owner_at->x, c.y - owner_at->y) > footprint_m &&
          near_path(c.x, c.y, owner_path, 0.3))
        {
          cells.push_back(c);
          has_ahead = true;
        }
      }
    }
    // A robot standing anywhere on the rest of the path blocks it, not only
    // within the look-ahead.
    const std::size_t to_i = body_only ? path.waypoints.size() - 1 : look_hi;
    // Where this path first meets a moving owner's stretch is booked until
    // the owner is safety_m (2.5 m) past it. Following it the same way
    // (back to back), or once it is past that first place, what it still
    // drives through on ahead holds this robot only if it could get there
    // before the owner is that far past.
    bool is_follower_behind = false;
    std::size_t lead_idx = 0;
    if (!body_only && owner_at && !standing) {
      lead_idx = nearest_waypoint_index(path, owner_at->x, owner_at->y);
      const double d_lead = std::hypot(
        path.waypoints[lead_idx].x - owner_at->x,
        path.waypoints[lead_idx].y - owner_at->y);
      const bool on_path_ahead = (d_lead <= footprint_m + 1.0) && (lead_idx >= start);
      is_follower_behind = on_path_ahead &&
        same_heading_at(path, lead_idx > 0 ? lead_idx : 1, owner_path);

      // Back-to-back: wait tracks the front pose at safety_m. Do not hold the
      // whole reserved base or pin the wait (that froze amr2 far behind).
      // Only when this path does not also cross the owner's booked stretch
      // ahead (crossing / T traffic): those must still WAIT before the base.
      if (is_follower_behind) {
        const auto cross = wait_before_reserved(
          path, start, to_i, blocking_cells(cells, own, footprint_m),
          owner_path, footprint_m, safety_m, amr_half_diag);
        // Pure follower (no cell of the reserved stretch on this path ahead
        // beyond tracking the pose): gap already large → FREE for this pair.
        const bool cross_same_way = !cross ||
          (cross->hit_i <= lead_idx + 1 &&
          same_heading_at(path, cross->hit_i > 0 ? cross->hit_i : 1, owner_path));
        if (cross_same_way) {
          if (lead_idx >= start && lead_idx <= look_hi) {
            met.insert(kv.first);
            std::size_t at = clear_wait_index(
              path, lead_idx, start, safety_m, stand_path, footprint_m,
              amr_half_diag);
            // C5: keep safety stop; do not raise to granted_i.
            at = clear_of_others(cache, path, at, start, kv.first, amr_half_diag);
            WaitDependency dep;
            dep.amr_id = kv.first;
            dep.wait_point = safe_wait_from_wp(path.waypoints[at]);
            dep.clear_points = {{owner_at->x, owner_at->y}};
            dep.topology = kTopoSameWay;
            rdeps[kv.first] = std::move(dep);
            first = std::min(first, at);
          }
          continue;
        }
        // Crossing into the owner's booked base: fall through to strict base
        // booking below (base ownership outranks follower tracking).
      }

      const auto strict = wait_before_reserved(
        path, start, to_i, blocking_cells(cells, own, footprint_m),
        owner_path, footprint_m, safety_m, amr_half_diag);
      if (strict) {
        met.insert(kv.first);
        const auto & meet = path.waypoints[strict->hit_i];
        auto first_meet = cache.reserve_conflict(path.amr_id, kv.first);
        if (!first_meet) {
          first_meet = ClearPoint{meet.x, meet.y};
          cache.set_reserve_conflict(path.amr_id, kv.first, *first_meet);
        }
      }
      // HARD RULE (strict base booking): a base booked by a moving AMR — from
      // base_tail_m behind its pose to its base end — is its alone. Cells are
      // NOT dropped because "the owner passes first in time"; any waypoint of
      // this path inside the booked stretch gets WAIT, even a single point.
      // The rear frees only as the owner moves on (update_pose trims the tail).
      // First meeting point stays booked until the owner is safety_m (2.5 m)
      // past it — even if update_pose already trimmed that cell.
      if (auto fm = cache.reserve_conflict(path.amr_id, kv.first)) {
        const double gap = safety_m;
        double past = std::hypot(owner_at->x - fm->x, owner_at->y - fm->y);
        if (!owner_path.waypoints.empty()) {
          const std::size_t mi =
            nearest_waypoint_index(owner_path, fm->x, fm->y);
          const double d_meet = std::hypot(
            owner_path.waypoints[mi].x - fm->x,
            owner_path.waypoints[mi].y - fm->y);
          // Meet still on the remaining owner path: use along-path progress.
          // Else the path was trimmed past the meet — Euclidean past is right
          // (nearest remaining wp would wrongly report past = 0).
          if (d_meet <= footprint_m + 1e-9) {
            const std::size_t oi =
              nearest_waypoint_index(owner_path, owner_at->x, owner_at->y);
            past = (oi > mi) ? path_length_between(owner_path, mi, oi) : 0.0;
          }
        }
        if (past + 1e-9 < gap) {
          cells.push_back({fm->x, fm->y, 0});
          met.insert(kv.first);
        }
      }
    }

    // Standing / waiting blocker:
    // 1) wait at safety_m (2.5 m) before its pose (within look-ahead);
    // 2) only if L×W bodies would still dash, step further back.
    // Detect with body_touch so size conflict counts even outside clearance.
    if (standing && owner_at && !has_ahead) {
      const double stand_detect =
        std::max(footprint_m, amr_half_diag > 0.0 ? 2.0 * amr_half_diag : 0.0) +
        1.0;
      if (!near_path(owner_at->x, owner_at->y, path, stand_detect)) {
        continue;
      }
      lead_idx = nearest_waypoint_index(path, owner_at->x, owner_at->y);
      if (lead_idx < start || lead_idx > look_hi) {
        continue;
      }
      std::size_t at = clear_wait_index(
        path, lead_idx, start, safety_m, stand_path, footprint_m, amr_half_diag);
      // C5: keep safety stop; do not raise to granted_i.
      at = clear_of_others(cache, path, at, start, kv.first, amr_half_diag);
      WaitDependency dep;
      dep.amr_id = kv.first;
      dep.wait_point = safe_wait_from_wp(path.waypoints[at]);
      dep.clear_points = {{owner_at->x, owner_at->y}};
      set_dep_ownership(dep);
      rdeps[kv.first] = std::move(dep);
      first = std::min(first, at);
      continue;
    }

    const auto hit = wait_before_reserved(
      path, start, to_i, blocking_cells(cells, own, footprint_m),
      owner_path, footprint_m, safety_m, amr_half_diag);
    if (!hit) {
      continue;
    }
    // Record first meeting even when the owner pose was unknown on the first
    // hit (pose arrives later from /fleet_states).
    {
      met.insert(kv.first);
      const auto & meet = path.waypoints[hit->hit_i];
      if (!cache.reserve_conflict(path.amr_id, kv.first)) {
        cache.set_reserve_conflict(
          path.amr_id, kv.first, ClearPoint{meet.x, meet.y});
      }
    }
    std::size_t at = hit->wait_i;
    double exact_x = hit->wait_x;
    double exact_y = hit->wait_y;
    bool use_exact = hit->has_exact_wait;
    // C5: keep calculated safety stop. Pushing to granted_i past that stop is
    // an authority conflict, not a coordinate fix (body_only still pins at
    // grant end only when that is not past the hit).
    if (body_only && granted_i && *granted_i < hit->hit_i && at < *granted_i) {
      at = *granted_i;
      use_exact = false;
    }
    const std::size_t cleared =
      clear_of_others(cache, path, at, start, kv.first, amr_half_diag);
    if (cleared != at) {
      at = cleared;
      use_exact = false;
    }
    const auto prev = prev_wait.find(kv.first);
    // Crossing / non-follower: do not creep forward while still blocked.
    // (B2B followers already returned above and are allowed to track.)
    if (!standing && prev != prev_wait.end() && prev->second >= start &&
      prev->second < at)
    {
      at = prev->second;
      use_exact = false;
    }
    // C5: if safe stop is before granted tip — keep it (authority conflict).
    WaitDependency dep;
    dep.amr_id = kv.first;
    dep.wait_point = safe_wait_from_wp(path.waypoints[at]);
    if (use_exact) {
      dep.wait_point.x = exact_x;
      dep.wait_point.y = exact_y;
    }
    dep.clear_points = hit->cells;
    set_dep_ownership(dep);
    rdeps[kv.first] = std::move(dep);
    first = std::min(first, at);
  }

  cache.keep_reserve_conflicts(path.amr_id, met);
  std::unordered_set<std::string> blockers;
  for (const auto & kv : rdeps) {
    blockers.insert(kv.first);
  }
  cache.set_reserve_waits(path.amr_id, blockers);
  if (rdeps.empty()) {
    return;
  }

  std::string ids;
  for (const auto & kv : rdeps) {
    ids += (ids.empty() ? "" : ",") + kv.first;
  }
  const int64_t first_idx = waypoint_index_value(path.waypoints[first], first);
  const bool auth_conflict = granted_i && first < *granted_i;
  const std::string auth_suffix = auth_conflict ?
    "; authority conflict — safe wait before granted base tip" : "";
  if (res.status == "WAIT") {
    std::unordered_map<std::string, WaitDependency> deps;
    for (const auto & mdep : res.waiting_for) {
      merge_dep(deps, dep_from_msg(mdep));
    }
    for (auto & kv : rdeps) {
      if (deps.count(kv.first) == 0) {
        deps[kv.first] = std::move(kv.second);
      }
    }
    respond_wait(
      res, res.horizon_m,
      res.dep_from_index < 0 ? first_idx : std::min(res.dep_from_index, first_idx),
      std::max(res.dep_to_index, first_idx), deps,
      res.message + "; also outside reserved base of " + ids + auth_suffix);
    return;
  }
  respond_wait(
    res, 0.0, first_idx, first_idx, rdeps,
    "WAIT; base reserved by (or robot standing at) " + ids +
    " ahead — wait until it is clear (no register needed)" + auth_suffix);
}

/// First index at or after \p start where \p path enters another AMR's
/// reserved / granted stretch (within look-ahead \p to_i). Returns the wait
/// index (safety_m before the hit), or nullopt if the window is clear.
/// Hard rule: one owner per booked cell — times ignored.
std::optional<std::pair<std::string, std::size_t>> first_foreign_base_wait(
  const ScheduleCache & cache,
  const TimedPath & path,
  std::size_t start,
  std::size_t to_i,
  double safety_m,
  double footprint_m,
  double amr_half_diag)
{
  if (path.waypoints.empty()) {
    return std::nullopt;
  }
  const auto own = cache.reserved(path.amr_id);
  std::optional<std::pair<std::string, std::size_t>> best;
  for (const auto & kv : cache.reserved_except(path.amr_id)) {
    TimedPath owner_path;
    if (auto p = cache.get(kv.first)) {
      owner_path = std::move(*p);
    }
    const auto hit = wait_before_reserved(
      path, start, to_i,
      blocking_cells(kv.second, own, footprint_m),
      owner_path, footprint_m, safety_m, amr_half_diag);
    if (!hit) {
      continue;
    }
    if (!best || hit->wait_i < best->second) {
      best = std::make_pair(kv.first, hit->wait_i);
    }
  }
  return best;
}

/// Remember the stretch this evaluate gives \p path's robot: up to the base
/// end \p hi on FREE, up to its (first) wait point on WAIT.
void save_reserved(
  ScheduleCache & cache,
  const srv::EvaluatePath::Response & res,
  const TimedPath & path,
  std::size_t start,
  std::size_t hi)
{
  if (path.waypoints.empty()) {
    return;
  }
  std::size_t end = std::min(hi, path.waypoints.size() - 1);
  if (res.status == "WAIT") {
    end = path.waypoints.size() - 1;
    for (const auto & mdep : res.waiting_for) {
      end = std::min(
        end, hold_index_on(path, from_msg_wait_point(mdep.wait_point), start));
    }
    end = std::max(end, start);
  }
  // The last base_tail_m behind the robot stays its own: from the path
  // behind it, else the tail it already holds.
  const double tail_m = cache.base_tail_m();
  std::size_t from = start;
  double back_m = 0.0;
  while (from > 0) {
    const auto & a = path.waypoints[from - 1];
    const auto & b = path.waypoints[from];
    const double seg = std::hypot(b.x - a.x, b.y - a.y);
    if (back_m + seg > tail_m + 1e-9) {
      break;
    }
    back_m += seg;
    --from;
  }
  std::vector<ClearPoint> cells;
  if (from == 0 && back_m + 1e-9 < tail_m) {
    const auto held = cache.reserved(path.amr_id);
    const auto & robot = path.waypoints[0];
    std::size_t at = held.size();
    double best = std::numeric_limits<double>::max();
    for (std::size_t k = 0; k < held.size(); ++k) {
      const double d = std::hypot(held[k].x - robot.x, held[k].y - robot.y);
      if (d < best) {
        best = d;
        at = k;
      }
    }
    std::size_t k = at;
    while (k > 0 && k < held.size()) {
      const double seg = std::hypot(held[k].x - held[k - 1].x, held[k].y - held[k - 1].y);
      if (back_m + seg > tail_m + 1e-9) {
        break;
      }
      back_m += seg;
      --k;
    }
    for (std::size_t j = k; j < at && at < held.size(); ++j) {
      cells.push_back({held[j].x, held[j].y});
    }
  }
  for (std::size_t i = from; i <= end; ++i) {
    cells.push_back({path.waypoints[i].x, path.waypoints[i].y});
  }
  cache.set_reserved(path.amr_id, cells);
}

/// Remove waits for any of \p blockers from \p path. True if one was removed.
bool strip_waits_for(TimedPath & path, const std::unordered_set<std::string> & blockers)
{
  bool changed = false;
  for (auto & wp : path.waypoints) {
    auto & deps = wp.waiting_for;
    const auto before = deps.size();
    deps.erase(
      std::remove_if(
        deps.begin(), deps.end(),
        [&](const WaitDependency & d) {return blockers.count(d.amr_id) > 0;}),
      deps.end());
    if (deps.size() != before) {
      changed = true;
      if (deps.empty()) {
        wp.wait_duration_sec = 0.0;
      }
    }
  }
  return changed;
}

/// End \p path's robot's holds for \p blockers and remove its waits for them
/// (\p path and its cached path).
void drop_holds(
  ScheduleCache & cache,
  TimedPath & path,
  const std::unordered_set<std::string> & blockers,
  std::vector<TimedPath> * committed_out)
{
  if (blockers.empty()) {
    return;
  }
  for (const auto & blocker : blockers) {
    cache.remove_hold(path.amr_id, blocker);
  }
  strip_waits_for(path, blockers);
  auto cached = cache.get(path.amr_id);
  if (cached && strip_waits_for(*cached, blockers)) {
    cache.upsert(*cached);
    if (committed_out) {
      // Replace an earlier copy so /to_zone_manager never shows the old wait.
      auto it = std::find_if(
        committed_out->begin(), committed_out->end(),
        [&](const TimedPath & p) {return p.amr_id == cached->amr_id;});
      if (it != committed_out->end()) {
        *it = *cached;
      } else {
        committed_out->push_back(*cached);
      }
    }
  }
}

/// True if any clear cell is still on the blocker's path ahead of its pose
/// and also on \p path (the waiter). Empty clear list → not passed.
bool blocker_still_ahead(
  const ScheduleCache & cache,
  const TimedPath & path,
  const std::string & blocker,
  const std::vector<ClearPoint> & clear_points,
  double eps_m)
{
  const auto other = cache.get(blocker);
  if (!other || other->waypoints.empty()) {
    // No path left (task done): standing pose is reserved separately.
    return false;
  }
  if (clear_points.empty()) {
    return true;
  }
  TimedPath remaining = *other;
  if (const auto pose = cache.pose(blocker)) {
    const std::size_t at = nearest_waypoint_index(remaining, pose->x, pose->y);
    if (at > 0 && at < remaining.waypoints.size()) {
      remaining.waypoints.erase(
        remaining.waypoints.begin(), remaining.waypoints.begin() + at);
    }
  }
  return std::any_of(
    clear_points.begin(), clear_points.end(),
    [&](const ClearPoint & c) {
      // Keep the wait only while the blocker has yet to pass a shared cell.
      return near_path(c.x, c.y, remaining, eps_m) &&
             near_path(c.x, c.y, path, eps_m);
    });
}

/// Holds / registered waits of \p path's robot whose blocker has driven past
/// the shared stretch end. Register moves the pair out of holds_, so the
/// registered path must clear the same way or the waiter waits forever.
void release_passed_blockers(
  ScheduleCache & cache,
  TimedPath & path,
  double eps_m,
  std::vector<TimedPath> * committed_out)
{
  std::unordered_set<std::string> passed;
  for (const auto & dep : cache.holds(path.amr_id)) {
    // T / + are not real AMRs — managed by the junction approach gate.
    if (is_junction_wait_id(dep.amr_id)) {
      continue;
    }
    if (!blocker_still_ahead(cache, path, dep.amr_id, dep.clear_points, eps_m)) {
      passed.insert(dep.amr_id);
    }
  }
  // Drop only blockers whose own clear_points prove passed. Co-located holds
  // (joined onto the same wait XY) must not cascade-drop — survivors keep the
  // hold so later conflict placement can re-seat them.
  drop_holds(cache, path, passed, committed_out);

  std::unordered_set<std::string> passed_reg;
  for (const auto & blocker : cache.registered_blockers(path.amr_id)) {
    if (is_junction_wait_id(blocker)) {
      continue;
    }
    std::vector<ClearPoint> clears;
    for (const auto & wp : path.waypoints) {
      for (const auto & d : wp.waiting_for) {
        if (d.amr_id == blocker) {
          append_unique_clear_points(clears, d.clear_points);
        }
      }
    }
    // ZM sometimes omits clear_points: use the shared stretch still on both
    // paths. None left → blocker already clear.
    if (clears.empty()) {
      if (const auto other = cache.get(blocker)) {
        for (const auto & w : other->waypoints) {
          if (near_path(w.x, w.y, path, eps_m)) {
            clears.push_back({w.x, w.y});
          }
        }
      }
      if (clears.empty()) {
        passed_reg.insert(blocker);
        continue;
      }
    }
    if (!blocker_still_ahead(cache, path, blocker, clears, eps_m)) {
      passed_reg.insert(blocker);
    }
  }
  if (passed_reg.empty()) {
    return;
  }
  for (const auto & blocker : passed_reg) {
    cache.unmark_registered(path.amr_id, blocker);
  }
  strip_waits_for(path, passed_reg);
  auto cached = cache.get(path.amr_id);
  if (cached && strip_waits_for(*cached, passed_reg)) {
    cache.upsert(*cached);
    if (committed_out) {
      auto it = std::find_if(
        committed_out->begin(), committed_out->end(),
        [&](const TimedPath & p) {return p.amr_id == cached->amr_id;});
      if (it != committed_out->end()) {
        *it = *cached;
      } else {
        committed_out->push_back(*cached);
      }
    }
  }
}

/// Holds of \p path's robot (at \p start) whose wait point it has already
/// passed end; if the conflict is still ahead, evaluate places a new wait
/// from where the robot is, instead of dragging the old one along with it.
void drop_holds_behind_robot(
  ScheduleCache & cache,
  TimedPath & path,
  std::size_t start,
  std::vector<TimedPath> * committed_out)
{
  auto passed = [&](const PathWaitPoint & wp) {
      if (!wp.index.has_value()) {
        return nearest_waypoint_index(path, wp.x, wp.y) < start;
      }
      for (std::size_t i = start; i < path.waypoints.size(); ++i) {
        if (path.waypoints[i].index.has_value() && *path.waypoints[i].index == *wp.index) {
          return false;
        }
      }
      return true;
    };
  std::unordered_set<std::string> behind;
  for (const auto & dep : cache.holds(path.amr_id)) {
    if (passed(dep.wait_point)) {
      behind.insert(dep.amr_id);
    }
  }
  drop_holds(cache, path, behind, committed_out);
}

/// Base owner priority: \p path's robot meets these AMRs only inside its
/// given base, so it neither holds nor waits for them. Their holds and waits
/// are removed (cached path and \p path).
void drop_waits_inside_given_base(
  ScheduleCache & cache,
  TimedPath & path,
  std::size_t start,
  std::optional<std::size_t> granted_i,
  double eps_m,
  std::vector<TimedPath> * committed_out)
{
  if (!granted_i) {
    return;
  }
  std::unordered_set<std::string> blockers;
  for (const auto & dep : cache.holds(path.amr_id)) {
    blockers.insert(dep.amr_id);
  }
  for (const auto & wp : path.waypoints) {
    for (const auto & d : wp.waiting_for) {
      blockers.insert(d.amr_id);
    }
  }
  std::unordered_set<std::string> dropped;
  for (const auto & blocker : blockers) {
    const auto other = cache.get(blocker);
    if (other && (overlap_inside_given_base(path, *other, start, granted_i, eps_m) ||
      owns_overlap_start(path, *other, start, granted_i, eps_m)))
    {
      dropped.insert(blocker);
    }
  }
  drop_holds(cache, path, dropped, committed_out);
}

/// Base owner priority on \p amr_id's cached path, using the base already
/// given to it (FREE) for its current task.
void enforce_owner_priority(
  ScheduleCache & cache,
  const std::string & amr_id,
  double eps_m,
  std::vector<TimedPath> * committed_out)
{
  auto cached = cache.get(amr_id);
  const auto base = cache.granted_base(amr_id);
  if (!cached || cached->waypoints.empty() || !base) {
    return;
  }
  if (!base->task_id.empty() && !cached->task_id.empty() &&
    base->task_id != cached->task_id)
  {
    return;
  }
  const auto g = locate_granted_base(*cached, *base, 0);
  drop_waits_inside_given_base(cache, *cached, 0, g, eps_m, committed_out);
}

/// F7: a wait that cannot be trusted. Returns "" if acceptable.
/// Checks: blocker is a real conflict partner of the waiter; wait is before the
/// waiter's entry; wait is not behind the robot (or off its route); wait is
/// not on a T/+ junction centre. T/+ sentinels skip only the partner check.
std::string validate_register_wait(
  const ConflictStore & conflicts,
  const ScheduleCache & cache,
  const NavGraphJunctions * junctions,
  const std::string & waiter,
  const TimedPath & path,
  const msg::WaitDependency & mdep)
{
  constexpr double kOffRouteM = 1.0;   // wait xy farther than this from the route
  constexpr double kOnCentreM = 0.75;  // same as pin_holds
  if (path.waypoints.empty()) {
    return "register: " + waiter + " has an empty cached path";
  }
  const std::size_t wait_at = resolve_wait_at(path, mdep.wait_point);
  const auto & wp = path.waypoints[wait_at];
  std::size_t robot_i = 0;
  if (const auto pose = cache.pose(waiter)) {
    robot_i = nearest_waypoint_index(path, pose->x, pose->y, 0);
  }
  if (wait_at < robot_i ||
    std::hypot(mdep.wait_point.x - wp.x, mdep.wait_point.y - wp.y) > kOffRouteM)
  {
    return "register: wait for " + waiter + " is behind the robot / off its route";
  }
  if (junctions && !junctions->empty() &&
    junctions->near_junction(wp.x, wp.y, kOnCentreM))
  {
    return "register: wait for " + waiter + " is on a T/+ junction centre";
  }
  if (is_junction_wait_id(mdep.amr_id)) {
    return "";
  }
  const auto match = match_register_wait(
    conflicts, waiter, mdep.amr_id, wp.route_occurrence);
  return match.error.empty() ? "" :
         "register: " + waiter + " waiting for " + mdep.amr_id + ": " + match.error;
}

void handle_register_updates(
  const srv::EvaluatePath::Request & req,
  srv::EvaluatePath::Response & res,
  ScheduleCache & cache,
  std::vector<TimedPath> * committed_out,
  const ConflictStore * conflicts,
  const NavGraphJunctions * junctions)
{
  fill_identity(req, res);

  if (req.updates.empty()) {
    res.message = "register updates[] is empty";
    return;
  }

  // All-or-nothing: validate every wait before anything is built or written.
  if (conflicts) {
    for (const auto & upd : req.updates) {
      const auto cached = upd.amr_id.empty() ?
        std::nullopt : cache.get(upd.amr_id);
      if (!cached) {
        continue;  // malformed / unseeded: the checks below report it
      }
      for (const auto & mdep : upd.waiting_for) {
        if (mdep.amr_id.empty()) {
          continue;
        }
        const auto err = validate_register_wait(
          *conflicts, cache, junctions, upd.amr_id, *cached, mdep);
        if (!err.empty()) {
          res.message = err;
          return;
        }
      }
    }
  }

  std::unordered_map<std::string, TimedPath> working;
  std::unordered_set<std::string> waiter_ids;
  std::unordered_set<std::string> blocker_ids;

  for (const auto & upd : req.updates) {
    if (upd.amr_id.empty()) {
      res.message = "register update missing amr_id";
      return;
    }
    if (upd.waiting_for.empty()) {
      res.message = "register update for " + upd.amr_id +
        " has empty waiting_for (list only waiters)";
      return;
    }
    const auto cached = cache.get(upd.amr_id);
    if (!cached.has_value()) {
      res.message = "register update: " + upd.amr_id +
        " not in ScheduleCache (run Process #1 / seed path first)";
      return;
    }
    TimedPath path = *cached;
    if (!upd.task_id.empty()) {
      path.task_id = upd.task_id;
    }
    path.phase_index = static_cast<uint32_t>(std::max(0, upd.phase_index));
    path.path_revision =
      static_cast<uint32_t>(std::max(0, upd.path_revision));
    if (!req.fleet_name.empty()) {
      path.fleet_name = req.fleet_name;
    }

    strip_existing_waits(path);

    for (const auto & mdep : upd.waiting_for) {
      if (mdep.amr_id.empty()) {
        res.message = "waiting_for entry missing amr_id on " + upd.amr_id;
        return;
      }
      blocker_ids.insert(mdep.amr_id);
      const std::size_t wait_at = resolve_wait_at(path, mdep.wait_point);
      WaitDependency dep = dep_from_msg(mdep);
      // Prefer ZM wait_point pose; fill from path wp if unset.
      if (!wait_point_is_set(dep.wait_point)) {
        dep.wait_point = safe_wait_from_wp(path.waypoints[wait_at]);
      }
      const double delta = std::max(0.0, mdep.wait_point.wait_sec);
      if (delta > 1e-9) {
        delay_path_at(
          path, wait_at, delta, dep.amr_id, dep.clear_points);
        // Keep ZM-provided wait_point (index/xy) after delay.
        for (auto & d : path.waypoints[wait_at].waiting_for) {
          if (d.amr_id == dep.amr_id) {
            d.wait_point = dep.wait_point;
            d.wait_point.wait_duration_sec =
              path.waypoints[wait_at].wait_duration_sec;
            if (d.clear_points.empty()) {
              d.clear_points = dep.clear_points;
            }
            break;
          }
        }
      } else {
        attach_mutual_dep(
          path.waypoints[wait_at], dep.amr_id, dep.clear_points);
        for (auto & d : path.waypoints[wait_at].waiting_for) {
          if (d.amr_id == dep.amr_id) {
            d.wait_point = dep.wait_point;
            d.clear_points = dep.clear_points;
            break;
          }
        }
      }
    }

    waiter_ids.insert(upd.amr_id);
    working[upd.amr_id] = std::move(path);
  }

  // GO: blockers not listed as waiters — strip stale waits, keep geometry/times.
  for (const auto & blocker : blocker_ids) {
    if (waiter_ids.count(blocker) != 0) {
      continue;
    }
    if (working.count(blocker) != 0) {
      continue;
    }
    const auto cached = cache.get(blocker);
    if (!cached.has_value()) {
      continue;  // blocker may not be cached yet; waiters still recorded
    }
    TimedPath go = *cached;
    strip_existing_waits(go);
    working[blocker] = std::move(go);
  }

  if (committed_out) {
    committed_out->clear();
  }
  for (auto & kv : working) {
    cache.upsert(kv.second);
    if (committed_out) {
      committed_out->push_back(kv.second);
    }
  }

  // ZM decided these pairs: waiter keeps its registered wait, blocker is GO.
  for (const auto & upd : req.updates) {
    for (const auto & mdep : upd.waiting_for) {
      cache.mark_registered(upd.amr_id, mdep.amr_id);
    }
  }

  // Lean response: summarize first waiter (or all deps merged).
  std::unordered_map<std::string, WaitDependency> deps;
  int64_t dep_from = -1;
  int64_t dep_to = -1;
  std::string first_waiter;
  for (const auto & upd : req.updates) {
    const auto & path = working[upd.amr_id];
    if (first_waiter.empty()) {
      first_waiter = upd.amr_id;
      res.amr_id = upd.amr_id;
      res.task_id = path.task_id;
      res.path_revision = static_cast<int32_t>(path.path_revision);
    }
    const std::size_t hi =
      path.waypoints.empty() ? 0 : path.waypoints.size() - 1;
    collect_registered_waits(path, 0, hi, dep_from, dep_to, deps, false);
  }

  std::string msg = "register updates: ";
  for (const auto & id : waiter_ids) {
    msg += id + "(WAIT) ";
  }
  for (const auto & id : blocker_ids) {
    if (waiter_ids.count(id) == 0) {
      msg += id + "(GO) ";
    }
  }

  if (!deps.empty()) {
    respond_wait(res, 0.0, dep_from, dep_to, deps, msg);
  } else {
    respond_free(res, req.query_horizon_m, msg);
  }
  res.success = true;
}

/// ECBS process #2: apply ZM register decision (updates[] only).
/// Exact ZM form — waiters only; GO implied. Call only after evaluate WAIT.
void handle_register(
  const srv::EvaluatePath::Request & req,
  srv::EvaluatePath::Response & res,
  ScheduleCache & cache,
  const EcbsSolver & solver,
  std::vector<TimedPath> * committed_out,
  const ConflictStore * conflicts,
  const NavGraphJunctions * junctions)
{
  (void)solver;
  if (req.updates.empty()) {
    res.message =
      "register requires updates[] (ZM waiters-only form; no waypoints)";
    return;
  }
  handle_register_updates(req, res, cache, committed_out, conflicts, junctions);
}

/// AMRs waiting before \p amr_id's reserved stretch (reserve wait, no hold).
std::unordered_set<std::string> waiting_on_base_of(
  const ScheduleCache & cache, const std::string & amr_id)
{
  std::unordered_set<std::string> out;
  for (const auto & other : cache.all_except(amr_id)) {
    if (cache.reserve_waits(other.amr_id).count(amr_id) > 0) {
      out.insert(other.amr_id);
    }
  }
  return out;
}

/// Another AMR waits before this robot's base, and this robot's way on past
/// its base end (\p granted_i, up to \p ahead_m further) runs into that AMR's
/// path: this robot waits at its base end too (never inside its base) and the
/// traffic manager decides who goes. No hold. A follower (same way along all
/// of it), or an AMR that cannot get there before this robot is clear_past_m
/// past (typically safety_distance_m), is only in the way if this robot's
/// goal lies on its path; else this robot drives on ahead of it.
void wait_at_base_end_for_waiters(
  srv::EvaluatePath::Response & res,
  const ScheduleCache & cache,
  const TimedPath & path,
  std::size_t start,
  std::optional<std::size_t> granted_i,
  double ahead_m,
  double footprint_m,
  double clear_past_m,
  double safety_m,
  const NavGraphJunctions * junctions)
{
  const std::size_t n = path.waypoints.size();
  if (!res.success || !granted_i || *granted_i < start || *granted_i + 1 >= n) {
    return;
  }
  // Base-end wait: never on a T/cross centre — snap to safety_m before it.
  PathWaitPoint base_wait = safe_wait_from_wp(path.waypoints[*granted_i]);
  std::size_t base_wait_i = *granted_i;
  apply_junction_hard_rule(
    path, start, safety_m, junctions, base_wait_i, base_wait);
  std::unordered_map<std::string, WaitDependency> ydeps;
  for (const auto & waiter : waiting_on_base_of(cache, path.amr_id)) {
    // ZM registered waiter → this AMR is GO: do not hold at base end for them.
    if (cache.registered_waiter(waiter, path.amr_id)) {
      continue;
    }
    const auto other = cache.get(waiter);
    if (!other || other->waypoints.empty()) {
      continue;
    }
    const auto & ow = other->waypoints;
    std::size_t other_at = 0;
    if (const auto p = cache.pose(waiter)) {
      other_at = nearest_waypoint_index(*other, p->x, p->y);
    }
    // Along \p other from where it is to the first waypoint near (x, y).
    auto other_reach_m = [&](double x, double y) -> std::optional<double> {
        double m = 0.0;
        for (std::size_t j = other_at; j < ow.size(); ++j) {
          if (j > other_at) {
            m += std::hypot(ow[j].x - ow[j - 1].x, ow[j].y - ow[j - 1].y);
          }
          if (std::hypot(ow[j].x - x, ow[j].y - y) <= footprint_m) {
            return m;
          }
        }
        return std::nullopt;
      };
    std::vector<ClearPoint> cells;
    bool follower = true;
    bool passes_first = true;
    double travelled = 0.0;
    for (std::size_t i = *granted_i + 1; i < n && travelled <= ahead_m; ++i) {
      const auto & w = path.waypoints[i];
      travelled += std::hypot(w.x - path.waypoints[i - 1].x, w.y - path.waypoints[i - 1].y);
      if (near_path(w.x, w.y, *other, footprint_m)) {
        cells.push_back({w.x, w.y});
        follower = follower && same_heading_at(path, i, *other);
        const auto reach = other_reach_m(w.x, w.y);
        passes_first = passes_first && (!reach ||
          *reach >= path_length_between(path, start, i) + clear_past_m);
      }
    }
    if (cells.empty()) {
      continue;
    }
    if (follower || passes_first || leads(path, start, *other, footprint_m)) {
      continue;
    }
    WaitDependency dep;
    dep.amr_id = waiter;
    dep.wait_point = base_wait;
    dep.clear_points = std::move(cells);
    ydeps[waiter] = std::move(dep);
  }
  if (ydeps.empty()) {
    return;
  }
  std::string ids;
  for (const auto & kv : ydeps) {
    ids += (ids.empty() ? "" : ",") + kv.first;
  }
  const int64_t idx =
    waypoint_index_value(path.waypoints[base_wait_i], base_wait_i);
  const double to_wait = path_length_between(path, start, base_wait_i);
  if (res.status == "WAIT") {
    std::unordered_map<std::string, WaitDependency> deps;
    for (const auto & mdep : res.waiting_for) {
      merge_dep(deps, dep_from_msg(mdep));
    }
    for (auto & kv : ydeps) {
      if (deps.count(kv.first) == 0) {
        deps[kv.first] = std::move(kv.second);
      }
    }
    respond_wait(
      res, std::min(res.horizon_m, to_wait),
      res.dep_from_index < 0 ? idx : std::min(res.dep_from_index, idx),
      std::max(res.dep_to_index, idx), deps,
      res.message + "; also at base end for " + ids +
      " (traffic manager decides)");
    return;
  }
  respond_wait(
    res, to_wait, idx, idx, ydeps,
    "WAIT; at base end — " + ids +
    " waits for this base and its way runs on ahead (traffic manager decides)");
}

/// Permanent rule: no wait inside the already-granted base. Shared stop =
/// the earliest dep wait along the path that is still at/after the base end.
/// Do NOT pull an opposite-conflict wait forward to a later T/+ gate — that
/// released AMR3 through the junction wait while amr2 was still opposite.
void enforce_waits_outside_given_base(
  srv::EvaluatePath::Response & res,
  ScheduleCache & cache,
  const std::string & amr_id,
  const TimedPath & path,
  std::size_t start,
  std::optional<std::size_t> granted_i)
{
  if (res.status != "WAIT" || path.waypoints.empty() || res.waiting_for.empty()) {
    return;
  }
  // C5: floor is pose start only — never raise waits to granted_i past their
  // safety stop. Align deps onto the earliest calculated stop instead.
  const std::size_t floor_i = std::min(start, path.waypoints.size() - 1);

  std::size_t shared_i = path.waypoints.size() - 1;
  const msg::WaitPoint * shared_src = nullptr;
  bool auth_conflict = false;
  for (const auto & mdep : res.waiting_for) {
    const PathWaitPoint wp = from_msg_wait_point(mdep.wait_point);
    std::size_t at = hold_index_by_xy(path, wp, start);
    at = std::max(at, floor_i);
    if (granted_i && at < *granted_i) {
      auth_conflict = true;
    }
    if (at <= shared_i) {
      shared_i = at;
      shared_src = &mdep.wait_point;
    }
  }

  double sx = path.waypoints[shared_i].x;
  double sy = path.waypoints[shared_i].y;
  int64_t sidx = waypoint_index_value(path.waypoints[shared_i], shared_i);
  if (shared_src) {
    sx = shared_src->x;
    sy = shared_src->y;
    if (shared_src->index >= 0) {
      sidx = shared_src->index;
    }
  }

  for (auto & mdep : res.waiting_for) {
    const PathWaitPoint wp = from_msg_wait_point(mdep.wait_point);
    const std::size_t by_xy = hold_index_by_xy(path, wp, start);
    const bool needs_snap =
      by_xy > shared_i ||
      std::hypot(mdep.wait_point.x - sx, mdep.wait_point.y - sy) > 0.05;
    if (!needs_snap) {
      continue;
    }
    // Snap later deps back to the earliest safe stop — never forward past it.
    const double sec = mdep.wait_point.wait_sec;
    mdep.wait_point.x = sx;
    mdep.wait_point.y = sy;
    mdep.wait_point.index = sidx;
    mdep.wait_point.wait_sec = sec;
    WaitDependency dep = dep_from_msg(mdep);
    if (cache.hold_pinned(amr_id, dep.amr_id)) {
      cache.force_hold_wait_point(amr_id, dep);
    } else if (cache.has_hold(amr_id, dep.amr_id)) {
      cache.add_hold(amr_id, dep, true);
    } else {
      cache.add_hold(amr_id, dep, false);
    }
  }
  if (auth_conflict &&
    res.message.find("authority conflict") == std::string::npos)
  {
    res.message += "; authority conflict — safe wait before granted base tip";
  }
}

/// Hard rule: wait never on a T/cross centre — always safety_m before it.
void snap_waits_off_junctions(
  srv::EvaluatePath::Response & res,
  ScheduleCache & cache,
  const std::string & amr_id,
  const TimedPath & path,
  std::size_t start,
  double safety_m,
  const NavGraphJunctions * junctions)
{
  if (!junctions || junctions->empty() || path.waypoints.empty() ||
    res.waiting_for.empty())
  {
    return;
  }
  for (auto & mdep : res.waiting_for) {
    if (is_junction_wait_id(mdep.amr_id)) {
      continue;
    }
    const auto wp = from_msg_wait_point(mdep.wait_point);
    const std::size_t wait_i = hold_index_on(path, wp, start);
    if (auto snapped = junctions->snap_wait_point_off_junction(
        path, wait_i, wp.x, wp.y, start, safety_m))
    {
      mdep.wait_point.x = snapped->second.x;
      mdep.wait_point.y = snapped->second.y;
      mdep.wait_point.index = waypoint_index_value(
        path.waypoints[snapped->first], snapped->first);
    } else {
      const std::size_t at = junctions->snap_wait_off_junction(
        path, wait_i, start, safety_m);
      if (at != wait_i) {
        mdep.wait_point.x = path.waypoints[at].x;
        mdep.wait_point.y = path.waypoints[at].y;
        mdep.wait_point.index = waypoint_index_value(path.waypoints[at], at);
      }
    }
    // Keep pinned holds in sync with the hard-rule wait point.
    cache.force_hold_wait_point(amr_id, dep_from_msg(mdep));
  }
}

/// True if \p res still has a real (non T/+) AMR conflict dependency.
bool has_real_amr_conflict(const srv::EvaluatePath::Response & res)
{
  for (const auto & d : res.waiting_for) {
    if (!is_junction_wait_id(d.amr_id)) {
      return true;
    }
  }
  return false;
}

/// Drop T/+ deps from \p res. If nothing real remains, mark FREE.
void clear_junction_sentinels_from_response(srv::EvaluatePath::Response & res)
{
  std::vector<msg::WaitDependency> kept;
  kept.reserve(res.waiting_for.size());
  for (const auto & d : res.waiting_for) {
    if (!is_junction_wait_id(d.amr_id)) {
      kept.push_back(d);
    }
  }
  res.waiting_for = std::move(kept);
  if (res.waiting_for.empty() && res.status == "WAIT" && !has_real_amr_conflict(res)) {
    res.status = "FREE";
    res.message = "FREE; junction gate released — no conflict within release distance";
    res.dep_from_index = -1;
    res.dep_to_index = -1;
  }
}

/// Junction base clip. A junction deep inside the base (safety_m or more before
/// its end) is driven through: full base, no wait. Only a junction within
/// safety_m of the base end \p hi (either side) gets a stop: the base ends
/// safety_m before its centre (WAIT labeled "T" / "+"), so a robot waiting at
/// the base end never sits in the junction. Dropped (base extends through) once
/// the robot is within \p release_m of that stop with no real AMR conflict.
/// The stop XY depends only on the path and the junction (never on the robot
/// pose), and once the granted base is past it the clip never returns — so the
/// wait point neither moves nor blinks between evaluates (backend and viz).
void apply_junction_approach_gate(
  srv::EvaluatePath::Response & res,
  ScheduleCache & cache,
  TimedPath & path,
  std::size_t start,
  std::size_t hi,
  const std::optional<std::size_t> & granted_i,
  double safety_m,
  double release_m,
  const NavGraphJunctions * junctions)
{
  std::unordered_set<std::string> sentinels{kJunctionWaitT, kJunctionWaitCross};
  drop_holds(cache, path, sentinels, nullptr);
  clear_junction_sentinels_from_response(res);

  if (!junctions || junctions->empty() || path.waypoints.empty()) {
    return;
  }
  // A real AMR hold already stops the robot at its safe point; an extra gate
  // stop earlier would only park it far from the conflict.
  if (has_real_amr_conflict(res)) {
    return;
  }
  auto approach = junctions->approach_near_base_end(path, start, hi, safety_m);
  if (!approach) {
    return;
  }
  // Base already given past the stop (released earlier): never clip again.
  if (granted_i && *granted_i > approach->wait_i + 1) {
    return;
  }
  const double to_wait = path_length_between(path, start, approach->wait_i);
  // Within release_m of the approach stop: keep only real AMR conflicts.
  if (to_wait <= release_m + 1e-9) {
    return;
  }

  WaitDependency dep;
  dep.amr_id = junction_wait_label(approach->kind);
  dep.wait_point.x = approach->wait_point.x;
  dep.wait_point.y = approach->wait_point.y;
  dep.wait_point.index = path.waypoints[approach->wait_i].index;
  ClearPoint cell;
  cell.x = path.waypoints[approach->junction_i].x;
  cell.y = path.waypoints[approach->junction_i].y;
  dep.clear_points.push_back(cell);
  dep.topology = kTopoJunction;

  std::unordered_map<std::string, WaitDependency> deps;
  for (const auto & mdep : res.waiting_for) {
    merge_dep(deps, dep_from_msg(mdep));
  }
  deps[dep.amr_id] = dep;
  const int64_t idx = waypoint_index_value(
    path.waypoints[approach->wait_i], approach->wait_i);
  const std::string label = dep.amr_id;
  const double horizon = res.status == "WAIT" ?
    std::min(res.horizon_m, to_wait) : to_wait;
  respond_wait(
    res, horizon,
    res.status == "WAIT" && res.dep_from_index >= 0 ?
      std::min(res.dep_from_index, idx) : idx,
    res.status == "WAIT" && res.dep_to_index >= 0 ?
      std::max(res.dep_to_index, idx) : idx,
    deps,
    std::string("WAIT; base cut before junction ") + label +
      ": junction centre " +
      std::to_string(path_length_between(path, start, approach->junction_i)) +
      " m ahead, base end " + std::to_string(path_length_between(path, start, hi)) +
      " m ahead — stop " + std::to_string(safety_m) +
      " m before the centre (release within " +
      std::to_string(release_m) + " m if clear)");
  cache.add_hold(path.amr_id, dep, false);
}

void handle_evaluate(
  const srv::EvaluatePath::Request & req,
  srv::EvaluatePath::Response & res,
  ScheduleCache & cache,
  const EcbsSolver & solver,
  std::vector<TimedPath> * committed_out,
  const NavGraphJunctions * junctions,
  const bool record_granted_base)
{
  if (req.waypoints.empty()) {
    res.message = "evaluate requires waypoints";
    return;
  }

  // No ECBS solve on evaluate — erase previous path behind pose from cache.
  cache.erase_before_pose(req.amr_id, req.current_x, req.current_y);

  TimedPath path = timed_path_from_evaluate_request(req);

  // Overlay committed waits from cache (after trim) onto request path.
  if (const auto cached = cache.get(req.amr_id)) {
    for (auto & wp : path.waypoints) {
      if (waypoint_has_wait(wp) || !wp.index.has_value()) {
        continue;
      }
      for (const auto & cwp : cached->waypoints) {
        if (!cwp.index.has_value() || *cwp.index != *wp.index) {
          continue;
        }
        if (waypoint_has_wait(cwp)) {
          wp.wait_duration_sec = cwp.wait_duration_sec;
          wp.waiting_for = cwp.waiting_for;
        }
        break;
      }
    }
  }

  const std::size_t start = nearest_waypoint_index(
    path, req.current_x, req.current_y);
  // Waits on the path before the strips below (kept for the head-on check).
  const TimedPath seeded_waits = path;
  std::size_t lo = start;
  std::size_t hi = start;
  double travelled = 0.0;
  horizon_index_range(path, start, req.query_horizon_m, lo, hi, travelled);

  std::size_t look_lo = start;
  std::size_t look_hi = hi;
  double look_travelled = 0.0;
  horizon_index_range(
    path, start,
    effective_query_horizon_m(req.query_horizon_m) +
    std::max(0.0, solver.config().evaluate_lookahead_margin_m),
    look_lo, look_hi, look_travelled);

  // Base already given to this AMR (last evaluate FREE), if still ahead.
  // Locate by geometry regardless of task_id — a new task must not wipe the
  // outstanding corridor grant while the robot still occupies it (C2).
  std::optional<std::size_t> granted_i;
  if (const auto base = cache.granted_base(req.amr_id)) {
    granted_i = locate_granted_base(path, *base, start);
    if (!granted_i) {
      // Grant XY not on this path: drop only once pose has left the grant
      // cell (peers still see reserved_ until update_pose trims it).
      constexpr double kPastGrantM = 1.0;
      const auto pose = cache.pose(req.amr_id);
      const double px = pose ? pose->x : req.current_x;
      const double py = pose ? pose->y : req.current_y;
      if (std::hypot(px - base->x, py - base->y) > kPastGrantM) {
        cache.clear_granted_base(req.amr_id);
      }
    }
  }

  const double safety_m = solver.config().safety_distance_m;
  const double eps_m = solver.config().clearance_m;
  const double footprint_m = solver.config().clearance_m;
  const double amr_half_diag = std::hypot(
    solver.config().amr_length_m * 0.5, solver.config().amr_width_m * 0.5);
  // Waits for another AMR's reserved stretch are re-checked below; they
  // never become holds. Where they were is kept: while still blocked they
  // do not move on as the stretch ahead frees up.
  std::unordered_map<std::string, std::size_t> prev_reserve_wait;
  {
    std::unordered_set<std::string> reserve_only;
    for (const auto & b : cache.reserve_waits(req.amr_id)) {
      if (!cache.has_hold(req.amr_id, b) && !cache.registered_waiter(req.amr_id, b)) {
        reserve_only.insert(b);
      }
    }
    for (std::size_t i = start; i < path.waypoints.size(); ++i) {
      for (const auto & d : path.waypoints[i].waiting_for) {
        if (reserve_only.count(d.amr_id) > 0) {
          prev_reserve_wait.emplace(d.amr_id, i);
        }
      }
    }
    // Waits at the base end for AMRs waiting on this base: re-checked below.
    // A hold for such an AMR came from its stale times: dropped.
    std::unordered_set<std::string> stale_holds;
    for (const auto & w : waiting_on_base_of(cache, req.amr_id)) {
      if (cache.registered_waiter(req.amr_id, w)) {
        continue;
      }
      // Only an unpinned hold (seeded planner wait, built on stale times) is
      // stale. A pinned hold is this robot's own evaluate answer for the
      // conflict: dropping it here made evaluate recompute the wait "one past
      // the base just given" every call, so the wait point crept forward one
      // waypoint per evaluate on head-on (opposite) traffic.
      if (cache.has_hold(req.amr_id, w) && !cache.hold_pinned(req.amr_id, w)) {
        stale_holds.insert(w);
      }
      reserve_only.insert(w);
    }
    strip_waits_for(path, reserve_only);
    drop_holds(cache, path, stale_holds, committed_out);
  }
  // AMRs that were decided to wait for this one: it never waits for them.
  {
    std::unordered_set<std::string> yielding;
    for (const auto & wp : path.waypoints) {
      for (const auto & d : wp.waiting_for) {
        if (cache.has_right_of_way(req.amr_id, d.amr_id) ||
          cache.registered_waiter(d.amr_id, req.amr_id))
        {
          yielding.insert(d.amr_id);
        }
      }
    }
    for (const auto & other : cache.all_except(req.amr_id)) {
      if (cache.registered_waiter(other.amr_id, req.amr_id)) {
        yielding.insert(other.amr_id);
      }
    }
    if (!yielding.empty()) {
      strip_waits_for(path, yielding);
      drop_holds(cache, path, yielding, committed_out);
      auto cached = cache.get(req.amr_id);
      if (cached && strip_waits_for(*cached, yielding)) {
        cache.upsert(*cached);
        if (committed_out) {
          committed_out->push_back(*cached);
        }
      }
    }
  }
  release_passed_blockers(cache, path, eps_m, committed_out);
  drop_holds_behind_robot(cache, path, start, committed_out);
  drop_waits_inside_given_base(cache, path, start, granted_i, eps_m, committed_out);

  // Priority 1 — base ownership (reserved / granted cells). Independent of
  // times: one owner per booked stretch. Path/time must not clear this WAIT.
  respond_reserved_waits(
    res, cache, path, start, look_hi, granted_i, safety_m, footprint_m,
    amr_half_diag, prev_reserve_wait);

  // Priority 2 — path / time conflicts (ECBS). Merges onto a base WAIT; never
  // replaces a base WAIT with FREE when there is no time clash.
  respond_from_path_status(
    res, path, lo, hi, look_hi, granted_i, safety_m,
    req.query_horizon_m, cache, eps_m, "", amr_half_diag, junctions);

  // A wait the seed/planner placed never moves back toward the robot: if this
  // answer put it earlier on the path, keep the seeded point (no flutter).
  if (res.success && res.status == "WAIT") {
    const std::size_t last_j = std::min(look_hi, seeded_waits.waypoints.size() - 1);
    for (auto & mdep : res.waiting_for) {
      WaitDependency dep = dep_from_msg(mdep);
      const std::size_t now_i = hold_index_on(path, dep.wait_point, start);
      for (std::size_t j = now_i + 1; j <= last_j; ++j) {
        const auto & sw = seeded_waits.waypoints[j].waiting_for;
        if (std::any_of(
            sw.begin(), sw.end(),
            [&](const WaitDependency & d) {return d.amr_id == dep.amr_id;}))
        {
          const double sec = dep.wait_point.wait_duration_sec;
          dep.wait_point = safe_wait_from_wp(seeded_waits.waypoints[j]);
          dep.wait_point.wait_duration_sec = sec;
          mdep = to_msg_dep(dep);
          break;
        }
      }
    }
  }

  // FREE only once no wait is pending: a seeded wait for an AMR coming head-on
  // along our path (opposite heading — no follower, no owner inside its given
  // base) that the strips above dropped without a hold / register / base
  // re-check is adopted as a hold (cleared by release_passed_blockers once
  // the blocker has passed).
  if (res.success && res.status == "FREE") {
    for (std::size_t i = start; i <= look_hi && i < seeded_waits.waypoints.size(); ++i) {
      const auto & swp = seeded_waits.waypoints[i];
      for (const auto & d : swp.waiting_for) {
        const auto blocker_path = cache.get(d.amr_id);
        if (d.amr_id.empty() || is_junction_wait_id(d.amr_id) || !blocker_path ||
          (granted_i && i <= *granted_i) ||
          cache.has_hold(req.amr_id, d.amr_id) ||
          cache.has_right_of_way(req.amr_id, d.amr_id) ||
          cache.registered_waiter(req.amr_id, d.amr_id) ||
          cache.registered_waiter(d.amr_id, req.amr_id) ||
          !opposite_overlap(path, *blocker_path, eps_m) ||
          !blocker_still_ahead(cache, path, d.amr_id, d.clear_points, eps_m))
        {
          continue;
        }
        WaitDependency dep = d;
        dep.wait_point = safe_wait_from_wp(swp);
        cache.add_hold(req.amr_id, dep, false);
      }
    }
  }

  if (res.success) {
    // A conflict holds until the blocker has passed or ZM registers the pair;
    // both robots hold unless one got its path first.
    // Reserved-base waits are not holds (no ZM register).
    if (res.status == "WAIT") {
      const auto reserve_blockers = cache.reserve_waits(req.amr_id);
      for (const auto & mdep : res.waiting_for) {
        if (reserve_blockers.count(mdep.amr_id) > 0 &&
          !cache.has_hold(req.amr_id, mdep.amr_id))
        {
          continue;
        }
        WaitDependency dep = dep_from_msg(mdep);
        if (!wait_point_is_set(dep.wait_point)) {
          dep.wait_point = safe_wait_from_wp(
            path.waypoints[hold_index_on(path, dep.wait_point, start)]);
        }
        cache.add_hold(req.amr_id, dep, false);
      }
    }
    for (const auto & dep : cache.holds(req.amr_id)) {
      hold_cached_blocker(
        cache, dep.amr_id, path, safety_m, eps_m, committed_out, amr_half_diag);
    }
    hold_self_for_waiters(
      cache, path, start, granted_i, safety_m, eps_m, false, amr_half_diag);
    // Pin *after* junction hard-rule so the robot never locks onto a T/cross
    // centre (outside_given_base used to pin there, then viz snapped later).
    pin_holds(
      cache, path, start, granted_i, amr_half_diag, safety_m, junctions);

    // Pinned wait points are answered as they are: the wait point must not
    // move with the robot (holds it has passed were dropped above).
    const auto holds = cache.holds(req.amr_id);
    if (!holds.empty()) {
      const bool was_wait = res.status == "WAIT";
      std::unordered_map<std::string, WaitDependency> deps;
      std::size_t first_wait = path.waypoints.size() - 1;
      std::string ids;
      bool auth_conflict = false;
      for (auto dep : holds) {
        const std::size_t by_xy =
          std::max(hold_index_by_xy(path, dep.wait_point, start), start);
        // C5: keep calculated/pinned wait; do not raise to granted_i.
        std::size_t at = outside_given_base(path, by_xy, granted_i);
        at = std::max(at, start);
        if (granted_i && at < *granted_i) {
          auth_conflict = true;
        }
        if (at != by_xy) {
          const double sec = dep.wait_point.wait_duration_sec;
          dep.wait_point = safe_wait_from_wp(path.waypoints[at]);
          dep.wait_point.wait_duration_sec = sec;
          if (cache.hold_pinned(req.amr_id, dep.amr_id)) {
            cache.force_hold_wait_point(req.amr_id, dep);
          } else {
            cache.add_hold(req.amr_id, dep, true);
          }
        }
        first_wait = std::min(first_wait, at);
        ids += (ids.empty() ? "" : ",") + dep.amr_id;
        deps[dep.amr_id] = std::move(dep);
      }
      if (was_wait) {
        for (const auto & mdep : res.waiting_for) {
          if (deps.count(mdep.amr_id) == 0) {
            merge_dep(deps, dep_from_msg(mdep));
          }
        }
      }
      const int64_t first_idx =
        waypoint_index_value(path.waypoints[first_wait], first_wait);
      respond_wait(
        res,
        was_wait ? res.horizon_m : 0.0,
        was_wait ? res.dep_from_index : first_idx,
        was_wait ? res.dep_to_index : first_idx,
        deps,
        std::string("WAIT; holding for ") + ids +
        " until it has passed or ZM registers — hold at wait_point" +
        (auth_conflict ?
          "; authority conflict — safe wait before granted base tip" : ""));
    }

    // Re-apply base ownership after holds/path so a path FREE or hold merge
    // cannot leave the robot released into another AMR's booked stretch.
    respond_reserved_waits(
      res, cache, path, start, look_hi, granted_i, safety_m, footprint_m,
      amr_half_diag, prev_reserve_wait);
    wait_at_base_end_for_waiters(
      res, cache, path, start, granted_i,
      effective_query_horizon_m(req.query_horizon_m), footprint_m, safety_m,
      safety_m, junctions);
    // Hard rule: never on T/cross centre — always safety_m before it.
    snap_waits_off_junctions(
      res, cache, req.amr_id, path, start, safety_m, junctions);
    // Base ends safety_m before a T/+ near its end; released within release_m.
    apply_junction_approach_gate(
      res, cache, path, start, hi, granted_i, safety_m,
      solver.config().junction_release_m, junctions);
    // Align deps onto earliest safe stop; never push past safety to grant tip.
    enforce_waits_outside_given_base(
      res, cache, req.amr_id, path, start, granted_i);
    save_reserved(cache, res, path, start, hi);
  }

  // What this answer releases to the robot is its base: up to the horizon end
  // on FREE, up to the (first) wait point on WAIT. Once released it cannot be
  // taken back, so a base never shrinks (WAIT included — XY-based).
  // Hard rule: never grow into another AMR's reserved/granted cells — even
  // when path/time said FREE (base ownership outranks time).
  if (res.success && (res.status == "FREE" || res.status == "WAIT") &&
    hi < path.waypoints.size())
  {
    std::size_t end = hi;
    if (res.status == "WAIT") {
      for (const auto & mdep : res.waiting_for) {
        end = std::min(
          end,
          hold_index_by_xy(path, from_msg_wait_point(mdep.wait_point), start));
      }
      end = std::max(end, start);
      // C5: WAIT at a safe stop before the prior tip is an authority conflict —
      // do not keep releasing past that stop by forcing end >= granted_i.
    } else if (granted_i) {
      end = std::max(end, *granted_i);
    }
    if (const auto foreign = first_foreign_base_wait(
        cache, path, start, std::max(end, look_hi), safety_m, footprint_m,
        amr_half_diag))
    {
      // Do not shrink a base already given past the foreign stop.
      if (!granted_i || foreign->second >= *granted_i) {
        if (foreign->second < end) {
          end = std::max(foreign->second, start);
          if (res.status == "FREE") {
            WaitDependency dep;
            dep.amr_id = foreign->first;
            dep.wait_point = safe_wait_from_wp(path.waypoints[end]);
            set_dep_ownership(dep);
            std::unordered_map<std::string, WaitDependency> deps;
            deps[dep.amr_id] = dep;
            const int64_t idx = waypoint_index_value(path.waypoints[end], end);
            respond_wait(
              res, path_length_between(path, start, end), idx, idx, deps,
              "WAIT; base reserved by (or robot standing at) " + foreign->first +
              " ahead — wait until it is clear (no register needed)");
            cache.set_reserve_waits(req.amr_id, {foreign->first});
          }
        }
      }
    }
    // Reserved stretch still set above for geometric protection. Committed
    // grant authority is owned by TM CommitProposal when record_granted_base
    // is false (proposal/commit path).
    if (record_granted_base) {
      GrantedBase base;
      base.task_id = req.task_id;
      base.x = path.waypoints[end].x;
      base.y = path.waypoints[end].y;
      base.index = path.waypoints[end].index;
      cache.set_granted_base(req.amr_id, base);
    }
  }

  if (!res.success) {
    return;
  }
  // Evaluate replaces earlier (planner) waits on the robot's cached path so
  // /to_zone_manager shows the same waiting point as this response.
  if (sync_cache_with_evaluate(
      req, res, solver.config().evaluate_lookahead_margin_m, cache, committed_out))
  {
    return;
  }

  // No request waypoints: still publish the cached path with waits stamped so
  // /to_zone_manager (and to_zone_manager_viz) show holds directly — not only
  // when a PathRelease WAIT is asked later.
  if (committed_out && !res.waiting_for.empty()) {
    if (const auto cached = cache.get(req.amr_id)) {
      TimedPath viz = *cached;
      for (auto & wp : viz.waypoints) {
        wp.wait_duration_sec = 0.0;
        wp.waiting_for.clear();
      }
      for (const auto & mdep : res.waiting_for) {
        if (mdep.amr_id.empty() || viz.waypoints.empty()) {
          continue;
        }
        const std::size_t wait_at = resolve_wait_at(viz, mdep.wait_point);
        WaitDependency dep = dep_from_msg(mdep);
        if (!wait_point_is_set(dep.wait_point)) {
          dep.wait_point = safe_wait_from_wp(viz.waypoints[wait_at]);
        }
        if (dep.wait_point.wait_duration_sec > 1e-6) {
          viz.waypoints[wait_at].wait_duration_sec = std::max(
            viz.waypoints[wait_at].wait_duration_sec,
            dep.wait_point.wait_duration_sec);
        }
        viz.waypoints[wait_at].waiting_for.push_back(std::move(dep));
      }
      cache.upsert(viz);
      committed_out->push_back(std::move(viz));
    }
  }
}

}  // namespace

namespace
{

builtin_interfaces::msg::Time sec_to_stamp(double t_sec)
{
  builtin_interfaces::msg::Time out;
  if (t_sec < 0.0) {
    t_sec = 0.0;
  }
  const auto sec = static_cast<int32_t>(std::floor(t_sec));
  out.sec = sec;
  out.nanosec = static_cast<uint32_t>(std::llround((t_sec - sec) * 1e9));
  if (out.nanosec >= 1000000000u) {
    out.sec += 1;
    out.nanosec -= 1000000000u;
  }
  return out;
}

}  // namespace

double stamp_timed_path_with_eta(TimedPath & path, double start_speed_mps)
{
  if (path.waypoints.size() < 2) {
    return 0.0;
  }
  std::vector<std::pair<double, double>> xy;
  xy.reserve(path.waypoints.size());
  for (const auto & wp : path.waypoints) {
    xy.emplace_back(wp.x, wp.y);
  }
  const auto reach = monitor::path_point_reach_times_sec(
    xy, monitor::cached_eta_robot_params(), start_speed_mps);
  if (reach.size() != path.waypoints.size()) {
    return 0.0;
  }
  const double t0 = path.waypoints.front().t_sec;
  for (std::size_t i = 0; i < path.waypoints.size(); ++i) {
    path.waypoints[i].t_sec = t0 + reach[i];
  }
  return reach.back();
}

TimedPath timed_path_from_evaluate_request(
  const srv::EvaluatePath::Request & req)
{
  TimedPath path;
  path.amr_id = req.amr_id;
  path.fleet_name = req.fleet_name;
  path.task_id = req.task_id;
  path.dispatch_task_id = req.task_id;
  path.phase_index = static_cast<uint32_t>(std::max(0, req.phase_index));
  path.path_revision = static_cast<uint32_t>(std::max(0, req.path_revision));

  path.waypoints.reserve(req.waypoints.size());
  for (const auto & wp : req.waypoints) {
    TimedPoint p;
    p.t_sec = wp.t_sec;
    p.x = wp.x;
    p.y = wp.y;
    p.yaw = wp.yaw;
    p.level_name = wp.level_name;
    if (wp.index >= 0) {
      p.index = static_cast<uint64_t>(wp.index);
    }
    // Wait fields only stored when dependencies are present.
    if (wp.wait_sec > 1e-6 || !wp.waiting_for.empty()) {
      p.wait_duration_sec = wp.wait_sec;
      for (const auto & dep : wp.waiting_for) {
        WaitDependency d;
        d.amr_id = dep.amr_id;
        d.wait_point.x = dep.wait_point.x;
        d.wait_point.y = dep.wait_point.y;
        if (dep.wait_point.index >= 0) {
          d.wait_point.index = static_cast<uint64_t>(dep.wait_point.index);
        }
        d.wait_point.wait_duration_sec = dep.wait_point.wait_sec;
        // Msg WaitPoint defaults index=0; treat all-zero unset as "fill from hold wp".
        const bool unset =
          d.wait_point.wait_duration_sec <= 1e-6 &&
          std::abs(d.wait_point.x) < 1e-9 &&
          std::abs(d.wait_point.y) < 1e-9 &&
          (!d.wait_point.index.has_value() || *d.wait_point.index == 0);
        if (unset) {
          d.wait_point.x = p.x;
          d.wait_point.y = p.y;
          d.wait_point.index = p.index;
          d.wait_point.wait_duration_sec = wp.wait_sec;
        }
        for (const auto & c : dep.clear_points) {
          d.clear_points.push_back({c.x, c.y});
        }
        p.waiting_for.push_back(std::move(d));
      }
    }
    path.waypoints.push_back(std::move(p));
  }
  // ECBS / replan use monitor eta_path_calculator, not RMF planner times.
  stamp_timed_path_with_eta(path, 0.0);
  return path;
}

msg::ToZoneManager to_zone_manager_msg(const TimedPath & path)
{
  msg::ToZoneManager out;
  out.fleet_name = path.fleet_name;
  out.amr_id = path.amr_id;
  out.path.reserve(path.waypoints.size());
  for (const auto & wp : path.waypoints) {
    msg::PathPoint pt;
    pt.t = sec_to_stamp(wp.t_sec);
    pt.x = wp.x;
    pt.y = wp.y;
    pt.wait_sec = published_wait_sec(wp);
    for (const auto & dep : wp.waiting_for) {
      msg::WaitDependency d;
      d.amr_id = dep.amr_id;
      d.topology = dep.topology;
      d.self_blocks_other = dep.self_blocks_other;
      d.other_blocks_self = dep.other_blocks_self;
      d.wait_point.x = dep.wait_point.x;
      d.wait_point.y = dep.wait_point.y;
      d.wait_point.index = dep.wait_point.index.has_value() ?
        static_cast<int64_t>(*dep.wait_point.index) : static_cast<int64_t>(-1);
      d.wait_point.wait_sec = dep.wait_point.wait_duration_sec;
      for (const auto & c : dep.clear_points) {
        msg::ClearPoint cp;
        cp.x = c.x;
        cp.y = c.y;
        d.clear_points.push_back(cp);
      }
      pt.waiting_for.push_back(std::move(d));
    }
    out.path.push_back(std::move(pt));
  }
  return out;
}

msg::FromZoneManager from_zone_manager_msg(const TimedPath & path)
{
  msg::FromZoneManager out;
  out.amr_id = path.amr_id;
  out.fleet_name = path.fleet_name;
  out.task_id = path.task_id;
  out.phase_index = static_cast<int32_t>(path.phase_index);
  out.path_revision = static_cast<int32_t>(path.path_revision);
  out.waypoints.reserve(path.waypoints.size());
  for (const auto & wp : path.waypoints) {
    msg::EvaluatePathWaypoint p;
    p.t_sec = wp.t_sec;
    p.x = wp.x;
    p.y = wp.y;
    p.yaw = wp.yaw;
    p.level_name = wp.level_name;
    p.index = wp.index.has_value() ?
      static_cast<int64_t>(*wp.index) : static_cast<int64_t>(-1);
    if (wp.wait_duration_sec > 1e-6 || !wp.waiting_for.empty()) {
      p.wait_sec = wp.wait_duration_sec;
      for (const auto & dep : wp.waiting_for) {
        msg::WaitDependency d;
        d.amr_id = dep.amr_id;
        d.topology = dep.topology;
        d.self_blocks_other = dep.self_blocks_other;
        d.other_blocks_self = dep.other_blocks_self;
        d.wait_point.x = dep.wait_point.x;
        d.wait_point.y = dep.wait_point.y;
        d.wait_point.index = dep.wait_point.index.has_value() ?
          static_cast<int64_t>(*dep.wait_point.index) :
          static_cast<int64_t>(-1);
        d.wait_point.wait_sec = dep.wait_point.wait_duration_sec;
        for (const auto & c : dep.clear_points) {
          msg::ClearPoint cp;
          cp.x = c.x;
          cp.y = c.y;
          d.clear_points.push_back(cp);
        }
        p.waiting_for.push_back(std::move(d));
      }
    }
    out.waypoints.push_back(std::move(p));
  }
  return out;
}

void evaluate_path(
  const srv::EvaluatePath::Request & req,
  srv::EvaluatePath::Response & res,
  ScheduleCache & cache,
  const EcbsSolver & solver,
  std::vector<TimedPath> * committed_out,
  const NavGraphJunctions * junctions,
  const bool record_granted_base,
  const ConflictStore * conflicts)
{
  fill_identity(req, res);
  if (committed_out) {
    committed_out->clear();
  }

  // Handle before normalize_mode — empty-waypoint "clear" must not become evaluate.
  if (req.mode == "clear") {
    if (req.amr_id.empty()) {
      res.message = "amr_id is empty";
      return;
    }
    cache.erase(req.amr_id);
    res.success = true;
    res.status = "FREE";
    res.message = "cleared ScheduleCache entry";
    return;
  }

  const std::string mode = normalize_mode(
    req.mode, !req.waypoints.empty(), !req.updates.empty());

  // Batch register lists waiters in updates[]; top-level amr_id optional.
  if (req.amr_id.empty() && !(mode == "register" && !req.updates.empty())) {
    res.message = "amr_id is empty";
    return;
  }

  if (mode == "register") {
    handle_register(req, res, cache, solver, committed_out, conflicts, junctions);
    return;
  }
  if (mode == "evaluate") {
    handle_evaluate(
      req, res, cache, solver, committed_out, junctions, record_granted_base);
    return;
  }

  res.message = "unknown mode (use register, evaluate, or clear)";
}

std::size_t persist_register_dependencies(
  const srv::EvaluatePath::Request & req,
  const ScheduleCache & cache,
  ConflictStore & conflicts)
{
  std::size_t changed = 0;
  for (const auto & upd : req.updates) {
    const auto path = cache.get(upd.amr_id);
    if (!path || path->waypoints.empty()) {
      continue;
    }
    for (const auto & mdep : upd.waiting_for) {
      if (mdep.amr_id.empty() || is_junction_wait_id(mdep.amr_id)) {
        continue;
      }
      const auto occ = path->waypoints[resolve_wait_at(*path, mdep.wait_point)]
        .route_occurrence;
      const auto match = match_register_wait(conflicts, upd.amr_id, mdep.amr_id, occ);
      if (match.error.empty() &&
        conflicts.set_mandatory_dependency(
          match.conflict_id, upd.amr_id, mdep.amr_id, true))
      {
        ++changed;
      }
    }
  }
  return changed;
}

void register_from_zone_manager(
  const msg::FromZoneManager & msg,
  srv::EvaluatePath::Response & res,
  ScheduleCache & cache,
  const EcbsSolver & solver,
  std::vector<TimedPath> * committed_out)
{
  // Path seed only — not ZM wait-decision register (that is updates[]).
  srv::EvaluatePath::Request req;
  req.mode = "register";
  req.amr_id = msg.amr_id;
  req.fleet_name = msg.fleet_name;
  req.task_id = msg.task_id;
  req.phase_index = msg.phase_index;
  req.path_revision = msg.path_revision;
  req.waypoints = msg.waypoints;
  req.query_horizon_m = 0.0;
  req.current_x = 0.0;
  req.current_y = 0.0;
  fill_identity(req, res);
  if (req.amr_id.empty()) {
    res.message = "amr_id is empty";
    return;
  }
  if (req.waypoints.empty()) {
    res.message = "from_zone_manager requires waypoints to seed cache";
    return;
  }
  TimedPath incoming = timed_path_from_evaluate_request(req);
  TimedPath path = incoming;

  // Snapshot granted bases: solve() clears the candidate's; restore after so
  // owner-priority can drop waits that land inside a base already given.
  // Keep prior granted/reserved across task seed until pose vacates the
  // stretch (update_pose) or a later evaluate supersedes the base. Silent
  // clear on task_id change drops corridor authority while the body is still
  // there (C2).
  const auto granted_before = cache.granted_base(path.amr_id);

  // Planning on the shared ScheduleCache: resolve conflict waits here (gate
  // local solve is often empty with external_evaluate_server) so replan can
  // compare wait vs detour in the planning window.
  for (auto & wp : path.waypoints) {
    wp.waiting_for.clear();
    wp.wait_duration_sec = 0.0;
  }

  const double safety_m = solver.config().safety_distance_m;
  const double eps_m = solver.config().clearance_m;
  const auto solved = solver.solve(path, cache);
  path = solved.path;
  if (solved.status == SolveStatus::FailReplan) {
    attach_reserved_waits(path, cache, solver.config());
    cache.upsert(path);
  }
  // Keep adapter-seeded waits (e.g. head-on tags) when solve did not place them.
  {
    bool merged = false;
    for (std::size_t i = 0; i < incoming.waypoints.size() && i < path.waypoints.size(); ++i) {
      for (const auto & d : incoming.waypoints[i].waiting_for) {
        // One wait per blocker: the solver's (safety_m before the conflict)
        // wins; a second, later one made the wait point jump back at the
        // first evaluate.
        bool have = false;
        for (const auto & pw : path.waypoints) {
          for (const auto & e : pw.waiting_for) {
            have = have || e.amr_id == d.amr_id;
          }
        }
        if (!have && !d.amr_id.empty()) {
          path.waypoints[i].waiting_for.push_back(d);
          if (d.wait_point.wait_duration_sec > path.waypoints[i].wait_duration_sec) {
            path.waypoints[i].wait_duration_sec = d.wait_point.wait_duration_sec;
          }
          merged = true;
        }
      }
    }
    if (merged) {
      cache.upsert(path);
    }
  }
  if (granted_before) {
    cache.set_granted_base(path.amr_id, *granted_before);
  }
  // Reserved-only waits (no time clash) are not holds — they clear alone.
  {
    TimedPath for_holds = path;
    std::unordered_set<std::string> reserve_only;
    for (const auto & b : cache.reserve_waits(path.amr_id)) {
      const auto other = cache.get(b);
      if (!other || !EcbsSolver::earliest_conflict(path, *other, eps_m)) {
        reserve_only.insert(b);
      }
    }
    strip_waits_for(for_holds, reserve_only);
    cache.add_plan_holds(for_holds);
  }

  {
    std::unordered_set<std::string> yielding;
    for (const auto & wp : path.waypoints) {
      for (const auto & d : wp.waiting_for) {
        if (cache.has_right_of_way(path.amr_id, d.amr_id)) {
          yielding.insert(d.amr_id);
        }
      }
    }
    if (strip_waits_for(path, yielding)) {
      cache.upsert(path);
      cache.add_plan_holds(path);
    }
  }
  hold_self_for_waiters(cache, path, 0, std::nullopt, safety_m, eps_m, true);
  for (const auto & dep : cache.holds(path.amr_id)) {
    bool on_path = false;
    for (const auto & wp : path.waypoints) {
      for (const auto & d : wp.waiting_for) {
        on_path = on_path || d.amr_id == dep.amr_id;
      }
    }
    if (!on_path) {
      attach_hold(path, hold_index_on(path, dep.wait_point, 0), dep);
      cache.upsert(path);
    }
  }
  if (committed_out) {
    committed_out->clear();
    committed_out->push_back(path);
    for (const auto & other : solved.updated_others) {
      if (const auto latest = cache.get(other.amr_id)) {
        committed_out->push_back(*latest);
      }
    }
  }
  for (const auto & dep : cache.holds(path.amr_id)) {
    hold_cached_blocker(cache, dep.amr_id, path, safety_m, eps_m, committed_out);
  }
  enforce_owner_priority(cache, path.amr_id, eps_m, committed_out);
  for (const auto & other : cache.all_except(path.amr_id)) {
    enforce_owner_priority(cache, other.amr_id, eps_m, committed_out);
  }
  if (const auto latest = cache.get(path.amr_id)) {
    path = *latest;
  }

  std::size_t n_wait = 0;
  for (const auto & wp : path.waypoints) {
    n_wait += wp.waiting_for.size();
  }
  respond_free(
    res, 0.0,
    "seeded path into ScheduleCache (planning solve; waits=" +
    std::to_string(n_wait) + "; not ZM register)");
}

}  // namespace ecbs_planner

