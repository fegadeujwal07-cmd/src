#include "ecbs_planner/replan_candidate.hpp"

#include "ecbs_planner/path_utils.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <unordered_set>
#include <utility>
#include <vector>

#include "monitor/eta_path_calculator.hpp"

namespace ecbs_planner
{

namespace
{

bool near(const TimedPoint & a, const TimedPoint & b, double eps_m)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy <= eps_m * eps_m;
}

/// Travel time along TimedPath XY via monitor ETA (falls back to length/speed).
double eta_travel_sec(const TimedPath & path, double fallback_speed_mps)
{
  if (path.waypoints.size() < 2) {
    return 0.0;
  }
  std::vector<std::pair<double, double>> xy;
  xy.reserve(path.waypoints.size());
  for (const auto & wp : path.waypoints) {
    xy.emplace_back(wp.x, wp.y);
  }
  const double t = monitor::estimate_path_travel_time_sec(
    xy, monitor::cached_eta_robot_params(), 0.0);
  if (t > 0.5) {
    return t;
  }
  double len = 0.0;
  for (std::size_t i = 1; i < path.waypoints.size(); ++i) {
    len += std::hypot(
      path.waypoints[i].x - path.waypoints[i - 1].x,
      path.waypoints[i].y - path.waypoints[i - 1].y);
  }
  return len / std::max(1e-3, fallback_speed_mps);
}

/// Time from path start to waypoint index i using monitor reach times.
double eta_to_index_sec(
  const TimedPath & path, std::size_t index, double fallback_speed_mps)
{
  if (path.waypoints.empty() || index == 0) {
    return 0.0;
  }
  index = std::min(index, path.waypoints.size() - 1);
  std::vector<std::pair<double, double>> xy;
  xy.reserve(index + 1);
  for (std::size_t i = 0; i <= index; ++i) {
    xy.emplace_back(path.waypoints[i].x, path.waypoints[i].y);
  }
  const auto reach = monitor::path_point_reach_times_sec(
    xy, monitor::cached_eta_robot_params(), 0.0);
  if (reach.size() == xy.size() && reach.back() > 0.5) {
    return reach.back();
  }
  return eta_travel_sec(path, fallback_speed_mps) *
         (static_cast<double>(index) /
         static_cast<double>(std::max<std::size_t>(1, path.waypoints.size() - 1)));
}

bool near_any(const TimedPoint & p, const TimedPath & path, double eps_m)
{
  for (const auto & w : path.waypoints) {
    if (near(p, w, eps_m)) {
      return true;
    }
  }
  return false;
}

/// Shortest distance from (px,py) to the segment (ax,ay)–(bx,by).
double point_to_segment_m(
  double px, double py, double ax, double ay, double bx, double by)
{
  const double abx = bx - ax;
  const double aby = by - ay;
  const double apx = px - ax;
  const double apy = py - ay;
  const double ab2 = abx * abx + aby * aby;
  if (ab2 < 1e-18) {
    return std::hypot(apx, apy);
  }
  double t = (apx * abx + apy * aby) / ab2;
  t = std::clamp(t, 0.0, 1.0);
  return std::hypot(px - (ax + t * abx), py - (ay + t * aby));
}

void push_blocked_unique(
  std::vector<int64_t> & blocked, std::vector<double> & until,
  const TimedPoint & w, double until_sec)
{
  if (!w.index) {
    return;
  }
  const int64_t idx = static_cast<int64_t>(*w.index);
  if (std::find(blocked.begin(), blocked.end(), idx) != blocked.end()) {
    return;
  }
  blocked.push_back(idx);
  until.push_back(until_sec);
}

/// Index on \p path of the end of the base already given to its AMR. The
/// traffic manager has released up to there, so no wait may be put before it.
std::optional<std::size_t> given_base_end(const ScheduleCache & cache, const TimedPath & path)
{
  const auto base = cache.granted_base(path.amr_id);
  if (!base || path.waypoints.empty() ||
    (!base->task_id.empty() && base->task_id != path.task_id))
  {
    return std::nullopt;
  }
  return locate_granted_base(path, *base, 0);
}

/// Drop \p blocker from every wait on \p path; true if anything changed.
bool strip_waits_for(TimedPath & path, const std::string & blocker)
{
  bool changed = false;
  for (auto & wp : path.waypoints) {
    const auto before = wp.waiting_for.size();
    wp.waiting_for.erase(
      std::remove_if(
        wp.waiting_for.begin(), wp.waiting_for.end(),
        [&](const WaitDependency & d) {return d.amr_id == blocker;}),
      wp.waiting_for.end());
    if (wp.waiting_for.size() != before) {
      changed = true;
      if (wp.waiting_for.empty()) {
        wp.wait_duration_sec = 0.0;
      }
    }
  }
  return changed;
}

}  // namespace

bool opposite_overlap(const TimedPath & a, const TimedPath & b, double eps_m)
{
  // (index on a, nearest index on b) for every place a shares with b.
  std::vector<std::pair<std::size_t, std::size_t>> shared;
  for (std::size_t i = 0; i < a.waypoints.size(); ++i) {
    std::optional<std::size_t> best;
    double best_d2 = eps_m * eps_m;
    for (std::size_t j = 0; j < b.waypoints.size(); ++j) {
      const double dx = a.waypoints[i].x - b.waypoints[j].x;
      const double dy = a.waypoints[i].y - b.waypoints[j].y;
      const double d2 = dx * dx + dy * dy;
      if (d2 <= best_d2) {
        best_d2 = d2;
        best = j;
      }
    }
    if (best) {
      shared.emplace_back(i, *best);
    }
  }
  if (shared.size() < 2) {
    return false;
  }
  return shared.back().second < shared.front().second;
}

std::string choose_replan_candidate(
  const std::string & a, const std::string & b,
  const ScheduleCache & cache,
  std::string * why)
{
  std::string later;
  std::string first;
  if (cache.accepted_before(a, b)) {
    first = a;
    later = b;
  } else if (cache.accepted_before(b, a)) {
    first = b;
    later = a;
  } else {
    later = a > b ? a : b;
    first = later == a ? b : a;
  }
  if (why) {
    *why = cache.accepted_before(first, later) || cache.accepted_before(later, first) ?
      "first come, first served: " + first + " got its task first" :
      "first come, first served: same time, later robot id replans";
  }
  return later;
}

ReplanCoordinator::ReplanCoordinator(ReplanConfig config)
: config_(config)
{
}

void ReplanCoordinator::set_facts(const std::string & amr, const RobotFacts & facts)
{
  facts_[amr] = facts;
}

bool ReplanCoordinator::involved(const std::string & amr) const
{
  for (const auto & kv : pairs_) {
    if (kv.second.candidate == amr || kv.second.keeper == amr) {
      return true;
    }
  }
  return false;
}

bool ReplanCoordinator::path_still_conflicted(
  const ScheduleCache & cache, const std::string & amr, double now_sec) const
{
  const auto path = cache.get(amr);
  if (!path || path->waypoints.size() < 2) {
    return false;
  }
  return path_has_obstacles(*path, cache, now_sec, nullptr);
}

bool ReplanCoordinator::force_obstacle_fallback(
  ScheduleCache & cache, const std::string & amr,
  std::vector<ReplanDecision> & out)
{
  auto sit = sessions_.find(amr);
  if (sit == sessions_.end()) {
    return false;
  }
  IterSession & s = sit->second;
  if (s.done && s.fallback) {
    cache.upsert(s.first_path);
    changed_.insert(amr);
    return true;
  }
  emit_fallback(
    s, cache, amr,
    "no path avoiding parked AMRs and goals after replan timeout -> using first RMF path",
    out);
  return true;
}

bool ReplanCoordinator::path_has_obstacles(
  const TimedPath & path, const ScheduleCache & cache, double now_sec,
  std::vector<BlockTag> * hits) const
{
  std::vector<BlockTag> found;
  if (path.waypoints.size() < 2) {
    if (hits) {
      hits->clear();
    }
    return false;
  }
  const auto & wps = path.waypoints;
  const std::size_t n = wps.size();
  const double until = now_sec + 3600.0;
  auto interior = [n](std::size_t j) {
      return j > 0 && j + 1 < n;
    };
  auto append_near = [&](
    double x, double y, double radius, bool parked_rule,
    const std::string & owner, const std::string & reason)
    {
      TimedPoint at;
      at.x = x;
      at.y = y;
      if (near(wps.front(), at, config_.clearance_m)) {
        return;
      }
      if (!parked_rule && near(wps.back(), at, radius)) {
        return;
      }
      bool near_segment = false;
      std::vector<int64_t> blocked;
      std::vector<double> untils;
      for (std::size_t i = 0; i + 1 < n; ++i) {
        const double d = point_to_segment_m(
          at.x, at.y, wps[i].x, wps[i].y, wps[i + 1].x, wps[i + 1].y);
        if (d > radius) {
          continue;
        }
        near_segment = true;
        const double d0 = std::hypot(at.x - wps[i].x, at.y - wps[i].y);
        const double d1 = std::hypot(at.x - wps[i + 1].x, at.y - wps[i + 1].y);
        const bool close0 = d0 <= radius;
        const bool close1 = d1 <= radius;
        auto push_interior = [&](std::size_t j) {
            if (interior(j)) {
              push_blocked_unique(blocked, untils, wps[j], until);
            }
          };
        if (close0 || close1) {
          if (close0) {
            push_interior(i);
          }
          if (close1) {
            push_interior(i + 1);
          }
        } else {
          push_interior(i);
          push_interior(i + 1);
        }
      }
      if (!near_segment) {
        return;
      }
      if (blocked.empty() && parked_rule && near(wps.back(), at, radius) &&
        n >= 2)
      {
        push_blocked_unique(blocked, untils, wps[n - 2], until);
      }
      for (const auto wp : blocked) {
        found.push_back(BlockTag{wp, owner, reason});
      }
    };

  for (const auto & kv : facts_) {
    const std::string & parked = kv.first;
    if (!kv.second.idle || parked == path.amr_id) {
      continue;
    }
    const auto pose = cache.standing_pose(parked);
    if (!pose) {
      continue;
    }
    append_near(
      pose->x, pose->y, config_.parked_block_radius_m, true, parked,
      "parked " + parked);
  }

  for (const auto & other : cache.all_except(path.amr_id)) {
    if (other.waypoints.size() < 2 || idle(other.amr_id) ||
      candidate_idle(other.amr_id))
    {
      continue;
    }
    const auto & goal = other.waypoints.back();
    append_near(
      goal.x, goal.y, config_.goal_block_radius_m, false, other.amr_id,
      "goal of " + other.amr_id);
  }
  if (hits) {
    *hits = found;
  }
  return !found.empty();
}

std::vector<int64_t> ReplanCoordinator::union_waypoints(const IterSession & s) const
{
  std::vector<int64_t> out;
  for (const auto & b : s.union_blocks) {
    if (std::find(out.begin(), out.end(), b.wp) == out.end()) {
      out.push_back(b.wp);
    }
  }
  return out;
}

ReplanDecision ReplanCoordinator::make_obstacle(
  ReplanDecisionKind kind, const std::string & candidate,
  const IterSession & s, const std::string & reason,
  const ScheduleCache & cache)
{
  Pair p;
  p.candidate = candidate;
  if (!s.keepers.empty()) {
    p.keeper = s.keepers.begin()->first;
  }
  p.blocked = union_waypoints(s);
  const double until = s.keepers.empty() || s.keepers.begin()->second.until.empty() ?
    1e9 : s.keepers.begin()->second.until.front();
  p.until.assign(p.blocked.size(), until);
  auto d = make(kind, p, reason, cache);
  d.conflicting.clear();
  d.conflicting.push_back(candidate);
  for (const auto & kv : s.keepers) {
    d.conflicting.push_back(kv.first);
  }
  return d;
}

void ReplanCoordinator::emit_fallback(
  IterSession & s, ScheduleCache & cache, const std::string & amr,
  const std::string & why, std::vector<ReplanDecision> & out)
{
  note(amr, why);
  cache.upsert(s.first_path);
  changed_.insert(amr);
  out.push_back(make_obstacle(ReplanDecisionKind::Clear, amr, s, why, cache));
  for (auto pit = parked_.begin(); pit != parked_.end(); ) {
    if (pit->second.candidate == amr) {
      pit = parked_.erase(pit);
    } else {
      ++pit;
    }
  }
  s.keepers.clear();
  s.done = true;
  s.fallback = true;
}

bool ReplanCoordinator::idle(const std::string & amr) const
{
  const auto it = facts_.find(amr);
  return it != facts_.end() && it->second.idle;
}

void ReplanCoordinator::set_awaiting(const std::string & amr, bool awaiting)
{
  if (awaiting) {
    awaiting_.insert(amr);
  } else {
    awaiting_.erase(amr);
  }
}

bool ReplanCoordinator::awaiting(const std::string & amr) const
{
  return awaiting_.count(amr) != 0;
}

bool ReplanCoordinator::candidate_idle(const std::string & amr) const
{
  return idle(amr) && !awaiting(amr);
}

std::string ReplanCoordinator::key(const std::string & a, const std::string & b)
{
  return a < b ? a + "|" + b : b + "|" + a;
}

bool ReplanCoordinator::path_changed(const TimedPath & now, const TimedPath & then) const
{
  // The cache only trims passed waypoints; a place the old path never had
  // means a new plan.
  for (const auto & w : now.waypoints) {
    if (!near_any(w, then, 0.3)) {
      return true;
    }
  }
  return false;
}

void ReplanCoordinator::blocked_corridor(
  const TimedPath & keeper, const TimedPath & candidate_now,
  const TimedPath & candidate_orig,
  std::vector<int64_t> & wps, std::vector<double> & until,
  double now_sec, int attempt) const
{
  wps.clear();
  until.clear();
  if (keeper.waypoints.empty() || candidate_orig.waypoints.empty()) {
    return;
  }
  const double eps = config_.clearance_m;
  // Shared keeper indices (order along the keeper's path).
  std::vector<std::size_t> shared;
  for (std::size_t k = 0; k < keeper.waypoints.size(); ++k) {
    if (near_any(keeper.waypoints[k], candidate_orig, eps)) {
      shared.push_back(k);
    }
  }
  if (shared.empty()) {
    return;
  }
  double shared_len = 0.0;
  for (std::size_t i = 1; i < shared.size(); ++i) {
    const auto & a = keeper.waypoints[shared[i - 1]];
    const auto & b = keeper.waypoints[shared[i]];
    shared_len += std::hypot(b.x - a.x, b.y - a.y);
  }
  // Attempts open more near the keeper: 0=2.5m, 1=3.5m, 2=4.5m, ...
  const double trim_m = config_.blocked_tail_ignore_m +
    static_cast<double>(attempt) * 1.0;
  // A2: a multi-point stretch shorter than the base trim → block nothing.
  // A single shared cell (crossing) still goes through the time filter.
  if (shared.size() >= 2 && shared_len < config_.blocked_tail_ignore_m) {
    return;
  }
  if (!warned_tail_ &&
    config_.blocked_tail_ignore_m > std::min(config_.safety_distance_m, config_.base_tail_m) + 1e-9)
  {
    warned_tail_ = true;
  }

  std::vector<bool> drop(shared.size(), false);
  if (shared.size() >= 2) {
    const TimedPoint keeper_at = keeper.waypoints.front();
    (void)candidate_now;
    const auto & shared_lo = keeper.waypoints[shared.front()];
    const auto & shared_hi = keeper.waypoints[shared.back()];
    const double d_lo = std::hypot(keeper_at.x - shared_lo.x, keeper_at.y - shared_lo.y);
    const double d_hi = std::hypot(keeper_at.x - shared_hi.x, keeper_at.y - shared_hi.y);
    const bool trim_from_hi = d_hi <= d_lo;
    double acc = 0.0;
    if (trim_from_hi) {
      drop.back() = true;
      for (std::size_t i = shared.size(); i-- > 1; ) {
        const auto & a = keeper.waypoints[shared[i - 1]];
        const auto & b = keeper.waypoints[shared[i]];
        acc += std::hypot(b.x - a.x, b.y - a.y);
        drop[i - 1] = true;
        if (acc + 1e-9 >= trim_m) {
          break;
        }
      }
    } else {
      drop.front() = true;
      for (std::size_t i = 1; i < shared.size(); ++i) {
        const auto & a = keeper.waypoints[shared[i - 1]];
        const auto & b = keeper.waypoints[shared[i]];
        acc += std::hypot(b.x - a.x, b.y - a.y);
        drop[i] = true;
        if (acc + 1e-9 >= trim_m) {
          break;
        }
      }
    }
  }

  std::optional<uint64_t> cand_here;
  if (!candidate_now.waypoints.empty()) {
    cand_here = candidate_now.waypoints.front().index;
  }
  std::optional<uint64_t> cand_goal;
  std::optional<uint64_t> cand_start;
  if (!candidate_orig.waypoints.empty()) {
    cand_goal = candidate_orig.waypoints.back().index;
    cand_start = candidate_orig.waypoints.front().index;
  }

  const double t0 = keeper.waypoints.front().t_sec;
  // Later attempts use a smaller growth → fewer time overlaps → fewer closures.
  const double growth = config_.blocked_margin_growth /
    (1.0 + 0.5 * static_cast<double>(attempt));
  const double base_m = config_.blocked_margin_sec;

  for (std::size_t si = 0; si < shared.size(); ++si) {
    if (drop[si]) {
      continue;
    }
    const std::size_t k = shared[si];
    const auto & w = keeper.waypoints[k];
    if (!w.index || w.index == cand_here || w.index == cand_goal ||
      w.index == cand_start)
    {
      continue;
    }
    const double keeper_arr = now_sec + std::max(0.0, w.t_sec - t0);
    // Parking at the keeper's goal never clears — keep the window open so a
    // later AMR whose path hits that goal still sees a time overlap (A1 +
    // goal-on-path special case).
    const bool parking = near(w, keeper.waypoints.back(), eps);
    const double keeper_leave = parking ? (now_sec + 3600.0) :
      ((k + 1 < keeper.waypoints.size()) ?
        now_sec + std::max(0.0, keeper.waypoints[k + 1].t_sec - t0) : keeper_arr);
    const double margin = base_m + growth * std::max(0.0, keeper_arr - now_sec);
    const double cand_arr = candidate_arrival_at(candidate_orig, w.x, w.y, now_sec);
    // A1: publish only when the candidate arrives inside the keeper window.
    if (cand_arr < keeper_arr - margin || cand_arr > keeper_leave + margin) {
      continue;
    }
    // Attempts 3+: keep only every other overlapping waypoint (sparser).
    if (attempt >= 3 && (si % 2) == 1) {
      continue;
    }
    const auto idx = static_cast<int64_t>(*w.index);
    const auto it = std::find(wps.begin(), wps.end(), idx);
    const double until_t = keeper_leave + margin;
    if (it == wps.end()) {
      wps.push_back(idx);
      until.push_back(until_t);
    } else {
      auto & u = until[static_cast<std::size_t>(it - wps.begin())];
      u = std::max(u, until_t);
    }
  }
}

/// True when the keeper's pose is past every waypoint that was closed for the
/// candidate (the first REPLAN's blocked set). Trimming the keeper's path must
/// not clear early — that reopened lanes and caused another replan.
bool ReplanCoordinator::keeper_passed_blocked(
  const TimedPath & keeper, const Pair & p, const ScheduleCache & cache) const
{
  if (p.blocked.empty()) {
    return true;
  }
  const auto pose = cache.pose(p.keeper);
  if (!pose || keeper.waypoints.empty()) {
    return false;
  }
  std::size_t last = 0;
  bool any = false;
  for (std::size_t i = 0; i < keeper.waypoints.size(); ++i) {
    const auto & idx = keeper.waypoints[i].index;
    if (!idx) {
      continue;
    }
    if (std::find(p.blocked.begin(), p.blocked.end(), static_cast<int64_t>(*idx)) !=
      p.blocked.end())
    {
      last = i;
      any = true;
    }
  }
  if (!any) {
    // Original blocked indices are gone from the remaining path (trimmed
    // behind the robot): the keeper has passed them.
    return true;
  }
  const std::size_t at = nearest_waypoint_index(keeper, pose->x, pose->y);
  return at > last;
}

ReplanDecision ReplanCoordinator::make(
  ReplanDecisionKind kind, const Pair & p, const std::string & reason,
  const ScheduleCache & cache)
{
  ReplanDecision d;
  d.kind = kind;
  d.candidate = p.candidate;
  d.keeper = p.keeper;
  if (const auto cp = cache.get(p.candidate)) {
    d.task_id = cp->task_id;
  }
  d.reason = reason;
  d.conflicting = {p.candidate, p.keeper};
  if (kind == ReplanDecisionKind::Replan) {
    d.blocked_waypoints = p.blocked;
    d.blocked_until_sec = p.until;
  }
  d.revision = ++revision_;
  return d;
}

void ReplanCoordinator::set_graph(NavGraph graph)
{
  graph_ = std::move(graph);
}

ReplanCoordinator::RoutePlan ReplanCoordinator::plan_route(
  const TimedPath & candidate, const TimedPath & keeper, double now_sec,
  const ScheduleCache * cache) const
{
  RoutePlan r;
  if (candidate.waypoints.empty()) {
    return r;
  }
  if (config_.blocked_tail_ignore_m >
    std::min(config_.safety_distance_m, config_.base_tail_m) + 1e-9 && !warned_tail_)
  {
    warned_tail_ = true;
    note(
      "cfg",
      "blocked_tail_ignore_m (" + std::to_string(config_.blocked_tail_ignore_m) +
      " m) > safety/base_tail (" +
      std::to_string(std::min(config_.safety_distance_m, config_.base_tail_m)) +
      " m): trim may let the candidate into the keeper's reserved tail");
  }
  const int max_try = std::max(1, config_.replan_max_retries);
  std::unordered_set<int64_t> extra_closed;
  for (int attempt = 0; attempt < max_try; ++attempt) {
    std::vector<int64_t> closed;
    std::vector<double> until;
    blocked_corridor(
      keeper, candidate, candidate, closed, until, now_sec, attempt);
    for (const auto e : extra_closed) {
      if (std::find(closed.begin(), closed.end(), e) == closed.end()) {
        closed.push_back(e);
        until.push_back(now_sec + 3600.0);
      }
    }
    if (closed.empty()) {
      note(
        key(candidate.amr_id, keeper.amr_id),
        "attempt " + std::to_string(attempt) +
        ": no time-overlapping corridor — no replan");
      r.ok = false;
      r.closed.clear();
      r.until.clear();
      r.attempt = attempt;
      if (attempt == 0) {
        return r;
      }
      continue;
    }
    const auto from = candidate.waypoints.front().index;
    const auto to = candidate.waypoints.back().index;
    if (graph_.empty() || !from || !to) {
      r.ok = true;
      r.closed = std::move(closed);
      r.until = std::move(until);
      r.attempt = attempt;
      return r;
    }
    std::unordered_set<int64_t> shut(closed.begin(), closed.end());
    auto reach = [&]() {
        return graph_.reachable(
          static_cast<std::size_t>(*from), static_cast<std::size_t>(*to), shut);
      };
    std::vector<int64_t> exit;
    bool ok = reach();
    for (std::size_t i = 1; !ok && i < candidate.waypoints.size(); ++i) {
      const auto & idx = candidate.waypoints[i].index;
      if (idx && *idx == *from) {
        continue;
      }
      if (!idx || shut.erase(static_cast<int64_t>(*idx)) == 0 || shut.empty()) {
        break;
      }
      exit.push_back(static_cast<int64_t>(*idx));
      ok = reach();
    }
    note(
      key(candidate.amr_id, keeper.amr_id),
      "attempt " + std::to_string(attempt) + ": closed " +
      std::to_string(shut.size()) + " wps, reachable=" + (ok ? "yes" : "no"));
    if (ok) {
      std::vector<int64_t> kept;
      std::vector<double> kept_until;
      for (std::size_t i = 0; i < closed.size(); ++i) {
        if (shut.count(closed[i])) {
          kept.push_back(closed[i]);
          kept_until.push_back(until[i]);
        }
      }
      r.ok = true;
      r.closed = std::move(kept);
      r.until = std::move(kept_until);
      r.exit = std::move(exit);
      r.attempt = attempt;
      // Hard rule: a route through an idle AMR or another AMR's goal is never
      // taken; the attempt fails and the next one runs (up to max_try).
      if (!cache || avoid_idle_and_goals(r, candidate, *cache, now_sec)) {
        return r;
      }
      r.ok = false;
      r.closed.clear();
      r.until.clear();
      r.exit.clear();
    }
    const auto open_route = graph_.shortest_route(
      static_cast<std::size_t>(*from), static_cast<std::size_t>(*to), {});
    if (open_route.size() >= 3) {
      extra_closed.insert(static_cast<int64_t>(open_route[open_route.size() / 2]));
    }
  }
  r.ok = false;
  r.attempt = max_try;
  note(
    key(candidate.amr_id, keeper.amr_id),
    "all " + std::to_string(max_try) +
    " alternatives failed — NO_ALTERNATIVE then CLEAR");
  return r;
}

// Reject detours through an idle AMR or another AMR's goal: close that spot
// (a goal: the whole path of its owner) and look again, until a route is clean.
// No clean route: r stays as the first (corridor-only) plan.
bool ReplanCoordinator::avoid_idle_and_goals(
  RoutePlan & r, const TimedPath & candidate, const ScheduleCache & cache,
  double now_sec) const
{
  const auto from = candidate.waypoints.front().index;
  const auto to = candidate.waypoints.back().index;
  if (graph_.empty() || !from || !to) {
    return true;
  }
  const auto others = cache.all_except(candidate.amr_id);
  std::unordered_set<int64_t> closed(r.closed.begin(), r.closed.end());
  std::vector<int64_t> added;
  // Never close the candidate's own start/goal, nor its exit (the stretch it
  // must drive first to get out — e.g. out of a dead end): closing them
  // leaves no route at all.
  const std::unordered_set<int64_t> exit(r.exit.begin(), r.exit.end());
  auto close = [&](int64_t i) {
      if (i != static_cast<int64_t>(*from) && i != static_cast<int64_t>(*to) &&
        exit.count(i) == 0 && closed.insert(i).second)
      {
        added.push_back(i);
      }
    };
  // Each pass closes at least one new waypoint, so this ends.
  for (std::size_t pass = 0; pass <= graph_.num_waypoints(); ++pass) {
    const auto route = graph_.shortest_route(
      static_cast<std::size_t>(*from), static_cast<std::size_t>(*to), closed);
    if (route.empty()) {
      return false;  // Every way is dirty: this attempt fails, try the next.
    }
    // The route AHEAD of the candidate: its start is where it stands now and
    // is being left, so another AMR's goal there (a head-on keeper heading to
    // the candidate's spot) does not make the detour dirty.
    const std::unordered_set<int64_t> on_route(route.begin() + 1, route.end());
    const auto added_before = added.size();
    for (const auto & o : others) {
      if (idle(o.amr_id)) {
        const auto pose = cache.standing_pose(o.amr_id);
        const auto at = pose ? graph_.nearest(pose->x, pose->y) : std::nullopt;
        if (at && on_route.count(static_cast<int64_t>(*at))) {
          close(static_cast<int64_t>(*at));
        }
      } else if (o.waypoints.size() > 1 && o.waypoints.back().index &&
        on_route.count(static_cast<int64_t>(*o.waypoints.back().index)))
      {
        for (const auto & w : o.waypoints) {
          if (w.index) {
            close(static_cast<int64_t>(*w.index));
          }
        }
      }
    }
    if (added.size() == added_before) {
      for (const auto i : added) {
        r.closed.push_back(i);
        r.until.push_back(now_sec + 3600.0);  // CLEAR reopens it
      }
      return true;
    }
  }
  return true;
}

bool ReplanCoordinator::has_route_around(
  const std::string & candidate, const std::string & keeper,
  const ScheduleCache & cache) const
{
  const auto cp = cache.get(candidate);
  const auto kp = cache.get(keeper);
  return cp && kp && plan_route(*cp, *kp, 0.0).ok;
}

bool ReplanCoordinator::left_exit(const TimedPath & candidate, const Pair & p) const
{
  for (const auto & w : candidate.waypoints) {
    for (const auto & c : p.exit_cells) {
      if (near(w, c, config_.clearance_m)) {
        return false;
      }
    }
  }
  return true;
}

void ReplanCoordinator::hold_keeper_before_exit(ScheduleCache & cache, Pair & p) const
{
  p.exit_cells.clear();
  auto kpath = cache.get(p.keeper);
  const auto cpath = cache.get(p.candidate);
  if (!kpath || !cpath || kpath->waypoints.empty() || cpath->waypoints.empty()) {
    return;
  }
  // The candidate's spot and its exit: the keeper must not enter them
  // before the candidate has left.
  std::vector<TimedPoint> cells{cpath->waypoints.front()};
  for (const auto & w : cpath->waypoints) {
    if (w.index && std::find(
        p.exit.begin(), p.exit.end(), static_cast<int64_t>(*w.index)) != p.exit.end())
    {
      cells.push_back(w);
    }
  }
  std::optional<std::size_t> first;
  for (std::size_t k = 0; k < kpath->waypoints.size() && !first; ++k) {
    const auto & w = kpath->waypoints[k];
    for (const auto & c : cells) {
      if ((w.index && c.index && *w.index == *c.index) || near(w, c, 0.3)) {
        first = k;
        break;
      }
    }
  }
  // No hold if the exit is not on the keeper's way, or lies in the base it
  // was already given (released: it owns it, the candidate waits instead).
  const auto base = given_base_end(cache, *kpath);
  if (!first || (base && *base >= *first)) {
    drop_hold(cache, p.keeper, p.candidate);
    return;
  }
  p.exit_cells = cells;
  // Wait far enough back that the candidate passes the exit beside it, but
  // never inside the given base.
  std::size_t wait_i = *first > 0 ? *first - 1 : 0;
  auto beside_exit = [&](const TimedPoint & w) {
      for (const auto & c : cells) {
        if (near(w, c, config_.clearance_m)) {
          return true;
        }
      }
      return false;
    };
  const std::size_t min_i = base.value_or(0);
  while (wait_i > min_i && beside_exit(kpath->waypoints[wait_i])) {
    --wait_i;
  }
  WaitDependency dep;
  dep.amr_id = p.candidate;
  const auto & at = kpath->waypoints[wait_i];
  dep.wait_point.x = at.x;
  dep.wait_point.y = at.y;
  dep.wait_point.index = at.index;
  for (const auto & c : cells) {
    dep.clear_points.push_back(ClearPoint{c.x, c.y});
  }
  cache.remove_hold(p.keeper, p.candidate);
  cache.add_hold(p.keeper, dep);
  cache.pin_hold(p.keeper, dep);

  strip_waits_for(*kpath, p.candidate);
  kpath->waypoints[wait_i].waiting_for.push_back(dep);
  cache.upsert(*kpath);
  changed_.insert(p.keeper);
}

void ReplanCoordinator::drop_hold(
  ScheduleCache & cache, const std::string & waiter, const std::string & blocker) const
{
  cache.remove_hold(waiter, blocker);
  if (auto path = cache.get(waiter)) {
    if (strip_waits_for(*path, blocker)) {
      cache.upsert(*path);
      changed_.insert(waiter);
    }
  }
}

std::vector<std::string> ReplanCoordinator::take_changed()
{
  std::vector<std::string> out(changed_.begin(), changed_.end());
  changed_.clear();
  return out;
}

std::vector<std::string> ReplanCoordinator::take_notes()
{
  return std::exchange(notes_, {});
}

void ReplanCoordinator::note(const std::string & pair_key, const std::string & text) const
{
  auto & last = last_note_[pair_key];
  if (last != text) {
    last = text;
    notes_.push_back(pair_key + ": " + text);
  }
}

void ReplanCoordinator::hold_in_place(ScheduleCache & cache, const Pair & p) const
{
  auto path = cache.get(p.candidate);
  if (!path || path->waypoints.empty()) {
    return;
  }
  WaitDependency dep;
  dep.amr_id = p.keeper;
  for (const auto & h : cache.holds(p.candidate)) {
    if (h.amr_id == p.keeper) {
      dep = h;
      break;
    }
  }
  // Where it stands, or at the end of the base it was already given (it
  // drives there anyway).
  auto & here = path->waypoints[given_base_end(cache, *path).value_or(0)];
  dep.wait_point.x = here.x;
  dep.wait_point.y = here.y;
  dep.wait_point.index = here.index;
  cache.remove_hold(p.candidate, p.keeper);
  cache.add_hold(p.candidate, dep);
  cache.pin_hold(p.candidate, dep);

  strip_waits_for(*path, p.keeper);
  here.waiting_for.push_back(dep);
  cache.upsert(*path);
  changed_.insert(p.candidate);
}

void ReplanCoordinator::start(
  Pair & p, ScheduleCache & cache, double now_sec,
  const std::string & why, std::vector<ReplanDecision> & out)
{
  const auto cp = cache.get(p.candidate);
  const auto kp = cache.get(p.keeper);
  if (!cp || !kp) {
    return;
  }
  p.candidate_path = *cp;
  p.decided_at = now_sec;
  p.resolved = false;
  if (p.blocked.empty()) {
    auto plan = plan_route(*cp, *kp, now_sec, &cache);
    p.exit = std::move(plan.exit);
    if (!plan.ok) {
      return;
    }
    p.blocked = std::move(plan.closed);
    p.until = std::move(plan.until);
  }
  if (p.blocked.empty()) {
    return;
  }
  hold_in_place(cache, p);
  hold_keeper_before_exit(cache, p);
  out.push_back(make(ReplanDecisionKind::Replan, p, why, cache));
}

double ReplanCoordinator::candidate_arrival_at(
  const TimedPath & candidate, double x, double y, double now_sec) const
{
  if (candidate.waypoints.empty()) {
    return now_sec;
  }
  std::size_t best = 0;
  double best_d2 = 1e300;
  for (std::size_t i = 0; i < candidate.waypoints.size(); ++i) {
    const double dx = candidate.waypoints[i].x - x;
    const double dy = candidate.waypoints[i].y - y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best = i;
    }
  }
  const double v = speed_mps(candidate);
  double travel = 0.0;
  double waits = 0.0;
  for (std::size_t i = 0; i < best; ++i) {
    waits += std::max(0.0, candidate.waypoints[i].wait_duration_sec);
    const auto & a = candidate.waypoints[i];
    const auto & b = candidate.waypoints[i + 1];
    travel += std::hypot(b.x - a.x, b.y - a.y);
  }
  const double t_path =
    candidate.waypoints[best].t_sec - candidate.waypoints.front().t_sec;
  if (t_path > 0.5) {
    return now_sec + t_path + waits;
  }
  return now_sec + (v > 1e-6 ? travel / v : 0.0) + waits;
}

double ReplanCoordinator::speed_mps(const TimedPath & path) const
{
  double len = 0.0;
  for (std::size_t i = 1; i < path.waypoints.size(); ++i) {
    len += std::hypot(
      path.waypoints[i].x - path.waypoints[i - 1].x,
      path.waypoints[i].y - path.waypoints[i - 1].y);
  }
  const double t = path.waypoints.empty() ? 0.0 :
    path.waypoints.back().t_sec - path.waypoints.front().t_sec;
  return (len > 0.5 && t > 0.5) ? len / t : config_.default_speed_mps;
}

std::optional<double> ReplanCoordinator::eta_waiting(
  const TimedPath & candidate, const TimedPath & keeper, double now_sec) const
{
  if (candidate.waypoints.empty() || keeper.waypoints.empty()) {
    return std::nullopt;
  }
  const double eps = config_.clearance_m;
  // Fully opposite: the candidate stands on the keeper's path, so the keeper
  // cannot pass until the candidate moves; waiting never solves this.
  if (near_any(candidate.waypoints.front(), keeper, eps)) {
    return std::nullopt;
  }

  double travel = candidate.waypoints.back().t_sec - candidate.waypoints.front().t_sec;
  if (travel <= 0.5) {  // Path has no usable times: length at the default speed.
    double len = 0.0;
    for (std::size_t i = 1; i < candidate.waypoints.size(); ++i) {
      len += std::hypot(
        candidate.waypoints[i].x - candidate.waypoints[i - 1].x,
        candidate.waypoints[i].y - candidate.waypoints[i - 1].y);
    }
    travel = len / config_.default_speed_mps;
  }

  // Prefer waiting points / wait_sec already on the planned path (planning).
  double wait_from_path = 0.0;
  bool saw_path_wait = false;
  for (const auto & wp : candidate.waypoints) {
    for (const auto & d : wp.waiting_for) {
      if (d.amr_id != keeper.amr_id) {
        continue;
      }
      saw_path_wait = true;
      wait_from_path = std::max(
        wait_from_path,
        std::max(d.wait_point.wait_duration_sec, wp.wait_duration_sec));
    }
  }
  if (saw_path_wait) {
    return std::max(0.0, travel) + wait_from_path + config_.blocked_margin_sec;
  }

  std::optional<std::size_t> last;
  for (std::size_t k = 0; k < keeper.waypoints.size(); ++k) {
    if (near_any(keeper.waypoints[k], candidate, eps)) {
      last = k;
    }
  }
  if (!last) {
    return std::nullopt;
  }
  // keeper_clear = now + remaining time along keeper path to last shared cell.
  const double t0 = keeper.waypoints.front().t_sec;
  const double keeper_clear =
    now_sec + std::max(0.0, keeper.waypoints[*last].t_sec - t0);
  const double wait =
    std::max(0.0, keeper_clear - now_sec) + config_.blocked_margin_sec;
  return std::max(0.0, travel) + wait;
}

std::optional<double> ReplanCoordinator::eta_replanned(
  const TimedPath & candidate, const std::vector<int64_t> & closed) const
{
  const auto from = candidate.waypoints.empty() ? std::nullopt : candidate.waypoints.front().index;
  const auto to = candidate.waypoints.empty() ? std::nullopt : candidate.waypoints.back().index;
  if (graph_.empty() || !from || !to) {
    return std::nullopt;
  }
  const std::unordered_set<int64_t> shut(closed.begin(), closed.end());
  const auto len = graph_.shortest_length(
    static_cast<std::size_t>(*from), static_cast<std::size_t>(*to), shut);
  if (!len) {
    return std::nullopt;
  }
  return *len / speed_mps(candidate);
}

bool ReplanCoordinator::detour_still_meets_keeper(
  const TimedPath & candidate, const TimedPath & keeper,
  const std::vector<int64_t> & closed) const
{
  if (graph_.empty() || candidate.waypoints.empty() || keeper.waypoints.empty()) {
    return false;
  }
  const auto from = candidate.waypoints.front().index;
  const auto to = candidate.waypoints.back().index;
  if (!from || !to) {
    return false;
  }
  const std::unordered_set<int64_t> shut(closed.begin(), closed.end());
  const auto route = graph_.shortest_route(
    static_cast<std::size_t>(*from), static_cast<std::size_t>(*to), shut);
  if (route.size() < 3) {
    return true;
  }
  const TimedPoint & goal = candidate.waypoints.back();
  const double eps = config_.clearance_m;
  for (std::size_t i = 1; i + 1 < route.size(); ++i) {
    const auto xy = graph_.position(route[i]);
    if (!xy) {
      continue;
    }
    TimedPoint p;
    p.x = xy->first;
    p.y = xy->second;
    p.index = static_cast<int64_t>(route[i]);
    if (near(p, goal, eps)) {
      continue;
    }
    if (near_any(p, keeper, eps)) {
      return true;
    }
  }
  return false;
}

void ReplanCoordinator::clear_pair_holds(ScheduleCache & cache, const Pair & p) const
{
  cache.clear_right_of_way(p.candidate, p.keeper);
  drop_hold(cache, p.candidate, p.keeper);
  drop_hold(cache, p.keeper, p.candidate);
}

bool ReplanCoordinator::planning(const TimedPath & path, double now_sec) const
{
  const auto it = first_seen_.find(path.amr_id);
  return it != first_seen_.end() && now_sec - it->second.second <= config_.planning_window_sec;
}

void ReplanCoordinator::replan_around_parked(
  ScheduleCache & cache, double now_sec, std::vector<ReplanDecision> & out)
{
  // The parked AMR drove off (or the mover is done): reopen its lanes.
  for (auto it = parked_.begin(); it != parked_.end(); ) {
    const auto cp = cache.get(it->second.candidate);
    if (idle(it->second.keeper) && cp && !cp->waypoints.empty()) {
      ++it;
      continue;
    }
    out.push_back(make(ReplanDecisionKind::Clear, it->second, "parked AMR left", cache));
    it = parked_.erase(it);
  }

  for (const auto & path : cache.all_except("")) {
    // Awaiting-approval movers still count as candidates (empty task_id).
    if (candidate_idle(path.amr_id) || path.waypoints.size() < 2 ||
      !planning(path, now_sec))
    {
      continue;
    }
    // One REPLAN per AMR at a time: a second one re-closes different lanes and
    // cancels the first detour (it came back on the original path and waited).
    if (involved(path.amr_id) || std::any_of(
        parked_.begin(), parked_.end(),
        [&](const auto & kv) {return kv.second.candidate == path.amr_id;}))
    {
      continue;
    }
    for (const auto & kv : facts_) {
      const std::string & parked = kv.first;
      const std::string k = key(path.amr_id, parked);
      const auto pose = kv.second.idle && parked != path.amr_id ?
        cache.standing_pose(parked) : std::nullopt;
      if (!pose || parked_.count(k)) {
        continue;
      }
      TimedPoint at;
      at.x = pose->x;
      at.y = pose->y;
      // Start skip uses solver cell clearance (often ~0.05), not
      // parked_block_radius_m: a robot beside the start must not abort the
      // rest of the path check.
      if (near(path.waypoints.front(), at, config_.clearance_m)) {
        continue;
      }
      Pair p;
      p.candidate = path.amr_id;
      p.keeper = parked;
      const double radius = config_.parked_block_radius_m;
      const double until = now_sec + 3600.0;  // CLEAR reopens when the AMR leaves
      const auto & wps = path.waypoints;
      const std::size_t n = wps.size();
      bool near_segment = false;
      double min_seg_dist = 1e9;
      const bool near_goal = near(wps.back(), at, radius);
      auto interior = [n](std::size_t j) {
          return j > 0 && j + 1 < n;
        };
      for (std::size_t i = 0; i + 1 < n; ++i) {
        const double d = point_to_segment_m(
          at.x, at.y, wps[i].x, wps[i].y, wps[i + 1].x, wps[i + 1].y);
        if (d > radius) {
          continue;
        }
        near_segment = true;
        min_seg_dist = std::min(min_seg_dist, d);
        // Goal occupancy: last segment alone must not close the approach cell.
        if (near_goal && i + 1 == n - 1) {
          continue;
        }
        const double d0 = std::hypot(at.x - wps[i].x, at.y - wps[i].y);
        const double d1 = std::hypot(at.x - wps[i + 1].x, at.y - wps[i + 1].y);
        const bool close0 = d0 <= radius;
        const bool close1 = d1 <= radius;
        // Near an endpoint → block only that end. Mid-segment hit → both ends.
        // (Avoids closing the approach cell of the previous segment.)
        if (close0 || close1) {
          if (close0 && interior(i)) {
            push_blocked_unique(p.blocked, p.until, wps[i], until);
          }
          if (close1 && interior(i + 1)) {
            push_blocked_unique(p.blocked, p.until, wps[i + 1], until);
          }
        } else {
          if (interior(i)) {
            push_blocked_unique(p.blocked, p.until, wps[i], until);
          }
          if (interior(i + 1)) {
            push_blocked_unique(p.blocked, p.until, wps[i + 1], until);
          }
        }
      }
      const auto from = wps.front().index;
      const auto to = wps.back().index;
      if (p.blocked.empty()) {
        if (near_goal) {
          note(
            k, "parked " + parked + " is at " + path.amr_id +
            "'s goal — keep waiting");
        } else if (near_segment) {
          note(
            k, "parked " + parked + " near " + path.amr_id +
            "'s path (min_seg_dist_m=" + std::to_string(min_seg_dist) +
            ") but no interior waypoint to close — keep waiting");
        }
        continue;
      }
      if (!graph_.empty() && from && to && !graph_.reachable(
          static_cast<std::size_t>(*from), static_cast<std::size_t>(*to),
          std::unordered_set<int64_t>(p.blocked.begin(), p.blocked.end())))
      {
        note(k, "no detour path is available for " + path.amr_id + " around parked " +
          parked + " — keep waiting (min_seg_dist_m=" +
          std::to_string(min_seg_dist) + ")");
        continue;
      }
      out.push_back(make(
        ReplanDecisionKind::Replan, p, "parked " + parked + " on the path", cache));
      parked_[k] = std::move(p);
    }
  }
}

void ReplanCoordinator::replan_goal_on_path(
  ScheduleCache & cache, double now_sec, std::vector<ReplanDecision> & out)
{
  const auto paths = cache.all_except("");
  for (const auto & first : paths) {
    for (const auto & later : paths) {
      // Keeper stays plain idle(); later is the candidate/mover.
      if (first.amr_id == later.amr_id || idle(first.amr_id) ||
        candidate_idle(later.amr_id) ||
        first.waypoints.size() < 2 || later.waypoints.size() < 2 ||
        !cache.accepted_before(first.amr_id, later.amr_id) ||
        involved(later.amr_id) ||
        std::any_of(
          parked_.begin(), parked_.end(),
          [&](const auto & kv) {return kv.second.candidate == later.amr_id;}))
      {
        continue;
      }
      const std::string k = key(first.amr_id, later.amr_id);
      const auto cleared = cleared_at_.find(k);
      if (pairs_.count(k) ||
        (cleared != cleared_at_.end() && now_sec - cleared->second < config_.reopen_guard_sec))
      {
        continue;
      }
      // The goal is on a stretch the first AMR still has to drive through.
      // waypoints[0] is where the first AMR stands now (the cache trims each
      // path behind the live pose), not a stretch still ahead of it — so start
      // at 1, like blocks_path below. Otherwise a later AMR heading to the
      // spot the first one is leaving (head-on pair) was misread as
      // goal-on-path and decided twice.
      bool goal_on_path = false;
      for (std::size_t j = 1; j + 1 < first.waypoints.size() && !goal_on_path; ++j) {
        goal_on_path = near(later.waypoints.back(), first.waypoints[j], config_.clearance_m);
      }
      // Hard rule: first AMR's goal sits on the later path — it stops there for
      // good. Alternate → REPLAN (no ETA). No alternate → keep waiting.
      bool blocks_path = false;
      for (std::size_t j = 1; j + 1 < later.waypoints.size() && !blocks_path; ++j) {
        blocks_path = near(first.waypoints.back(), later.waypoints[j], config_.clearance_m);
      }
      if (!goal_on_path && !blocks_path) {
        continue;
      }
      // Soft goal_on_path (ETA) still only during the planning window.
      if (!blocks_path && !planning(later, now_sec)) {
        continue;
      }
      const auto route = plan_route(later, first, now_sec, &cache);
      if (!route.ok) {
        Pair p;
        p.candidate = later.amr_id;
        p.keeper = first.amr_id;
        p.candidate_path = later;
        p.keeper_path = first;
        // Soft ETA case: no shared timed corridor → waiting is fine.
        if (!blocks_path && route.closed.empty() && route.attempt <= 0) {
          note(k, "no time overlap with " + first.amr_id + " — keep waiting");
          continue;
        }
        // Hard rule: first's goal sits on later's path. Never approve/drive that
        // path — even when there is no time-overlap corridor yet (that skip is
        // what released the conflicted shortest path before REPLAN).
        if (blocks_path) {
          if (first.waypoints.back().index) {
            p.blocked.push_back(static_cast<int64_t>(*first.waypoints.back().index));
            p.until.push_back(now_sec + 3600.0);
          }
          for (std::size_t j = 1; j + 1 < later.waypoints.size(); ++j) {
            if (!near(first.waypoints.back(), later.waypoints[j], config_.clearance_m) ||
              !later.waypoints[j].index)
            {
              continue;
            }
            const auto idx = static_cast<int64_t>(*later.waypoints[j].index);
            if (std::find(p.blocked.begin(), p.blocked.end(), idx) == p.blocked.end()) {
              p.blocked.push_back(idx);
              p.until.push_back(now_sec + 3600.0);
            }
          }
          if (!p.blocked.empty()) {
            const auto from = later.waypoints.front().index;
            const auto to = later.waypoints.back().index;
            const bool detour = graph_.empty() || !from || !to || graph_.reachable(
              static_cast<std::size_t>(*from), static_cast<std::size_t>(*to),
              std::unordered_set<int64_t>(p.blocked.begin(), p.blocked.end()));
            if (detour) {
              start(
                p, cache, now_sec,
                first.amr_id + "'s goal is on the path; alternate taken", out);
              if (!p.blocked.empty()) {
                pairs_[k] = std::move(p);
              }
              continue;
            }
          }
          out.push_back(make(
            ReplanDecisionKind::NoAlternative, p,
            "no detour path is available after " +
            std::to_string(config_.replan_max_retries) + " attempts", cache));
          out.push_back(make(
            ReplanDecisionKind::Clear, p,
            "no detour path is available (" + first.amr_id +
            "'s goal is on the path) — keep waiting", cache));
          p.gave_up = true;
          cleared_at_[k] = now_sec;
          pairs_[k] = std::move(p);
        }
        continue;
      }
      if (!blocks_path) {
        const auto eta_wait = eta_waiting(later, first, now_sec);
        const auto eta_alt = eta_replanned(later, route.closed);
        if (!eta_wait || !eta_alt || !(*eta_alt < *eta_wait)) {
          continue;  // Waiting for the first AMR to pass is as fast or faster.
        }
        if (detour_still_meets_keeper(later, first, route.closed)) {
          note(
            k,
            later.amr_id + " detour still meets " + first.amr_id +
            " (wait ETA " + std::to_string(static_cast<int>(*eta_wait)) +
            " s vs replanned " + std::to_string(static_cast<int>(*eta_alt)) +
            " s) — keep waiting");
          continue;
        }
      }
      Pair p;
      p.candidate = later.amr_id;
      p.keeper = first.amr_id;
      start(
        p, cache, now_sec,
        blocks_path ? first.amr_id + "'s goal is on the path; alternate taken" :
        "goal on " + first.amr_id + "'s path; detour is faster", out);
      if (!p.blocked.empty()) {
        pairs_[k] = std::move(p);
      }
    }
  }
}

std::vector<ReplanDecision> ReplanCoordinator::update(ScheduleCache & cache, double now_sec)
{
  std::vector<ReplanDecision> out;
  const auto paths = cache.all_except("");
  for (const auto & p : paths) {
    if (p.waypoints.empty()) {
      continue;
    }
    const auto & g = p.waypoints.back();
    const std::string goal = std::to_string(std::lround(g.x * 10)) + "," +
      std::to_string(std::lround(g.y * 10));
    auto & seen = first_seen_[p.amr_id];
    if (seen.first != goal) {
      seen = {goal, now_sec};
    }
  }

  // Follow pairs already being resolved.
  for (auto it = pairs_.begin(); it != pairs_.end(); ) {
    Pair & p = it->second;
    const auto kp = cache.get(p.keeper);
    const auto cp = cache.get(p.candidate);
    const bool gone = !kp || !cp || kp->waypoints.empty() || cp->waypoints.empty();

    if (gone) {
      if (!p.gave_up) {
        if (!p.resolved) {
          clear_pair_holds(cache, p);
        }
        cache.clear_right_of_way(p.candidate, p.keeper);
        out.push_back(make(ReplanDecisionKind::Clear, p, "path finished or dropped", cache));
        cleared_at_[it->first] = now_sec;
      }
      it = pairs_.erase(it);
      continue;
    }

    // Awaiting candidate is still the mover — do not CLEAR as "idle".
    if (idle(p.keeper) || candidate_idle(p.candidate)) {
      if (!p.gave_up) {
        if (!p.resolved) {
          clear_pair_holds(cache, p);
        }
        cache.clear_right_of_way(p.candidate, p.keeper);
        out.push_back(make(ReplanDecisionKind::Clear, p, "idle AMR: wait instead", cache));
        cleared_at_[it->first] = now_sec;
      }
      it = pairs_.erase(it);
      continue;
    }

    if (p.gave_up) {
      if (path_changed(*cp, p.candidate_path) || path_changed(*kp, p.keeper_path)) {
        it = pairs_.erase(it);
      } else {
        ++it;
      }
      continue;
    }

    if (!p.resolved) {
      // Resolved once the candidate has a new path that keeps out of the
      // closed corridor (its exit, start and goal may touch the keeper's
      // path; who goes first there is the traffic manager's call). The
      // keeper keeps its hold before the exit until the candidate left it.
      bool clear_of_corridor = path_changed(*cp, p.candidate_path);
      for (std::size_t i = 1; clear_of_corridor && i + 1 < cp->waypoints.size(); ++i) {
        const auto & idx = cp->waypoints[i].index;
        clear_of_corridor = !idx || std::find(
          p.blocked.begin(), p.blocked.end(), static_cast<int64_t>(*idx)) == p.blocked.end();
      }
      std::string failure;
      if (clear_of_corridor) {
        // Decided: the keeper waits before the exit, the candidate goes.
        p.resolved = true;
        drop_hold(cache, p.candidate, p.keeper);
        cache.set_right_of_way(p.candidate, p.keeper);
      } else if (now_sec - p.decided_at > config_.replan_timeout_sec) {
        failure = "no detour path is available around " + p.keeper + " (timeout)";
      }

      if (!failure.empty()) {
        // The first AMR never replans: both just wait until a path changes.
        clear_pair_holds(cache, p);
        // Fully opposite paths and no detour: neither can pass, so both hold
        // for each other (mutual wait) instead of only the candidate.
        if (opposite_overlap(*cp, *kp, config_.clearance_m)) {
          Pair rev;
          rev.candidate = p.keeper;
          rev.keeper = p.candidate;
          hold_in_place(cache, p);
          hold_in_place(cache, rev);
        }
        out.push_back(make(ReplanDecisionKind::NoAlternative, p, failure, cache));
        out.push_back(make(ReplanDecisionKind::Clear, p, failure, cache));
        p.gave_up = true;
        p.candidate_path = *cp;
        p.keeper_path = *kp;
        ++it;
        continue;
      }
    }

    // The keeper waited only for the candidate to get out of its exit.
    if (p.resolved && !p.keeper_released && left_exit(*cp, p)) {
      p.keeper_released = true;
      drop_hold(cache, p.keeper, p.candidate);
    }

    // One REPLAN only for this pair. Do NOT CLEAR when the keeper leaves the
    // corridor while the candidate still has a path — that reopened lanes and
    // RMF planned again. CLEAR is only emitted when the candidate's path is
    // gone / idle (handled above).
    (void)now_sec;
    ++it;
  }

  replan_around_parked(cache, now_sec, out);
  replan_goal_on_path(cache, now_sec, out);

  // Every wait is compared with a detour: a mutual hold (head-on / crossing)
  // or a one-way wait of `a` for `b`. No planning-window limit.
  for (const auto & path : paths) {
    const std::string & a = path.amr_id;
    // Planner holds plus pairs ZM already registered (register erases the
    // hold, but the wait is still the one to compare with a detour).
    std::set<std::string> blockers;
    for (const auto & dep : cache.holds(a)) {
      blockers.insert(dep.amr_id);
    }
    for (const auto & r : cache.registered_blockers(a)) {
      blockers.insert(r);
    }
    for (const std::string & b : blockers) {
      const bool mutual = cache.has_hold(b, a);
      // a = candidate (holds for b); b = keeper / blocker.
      if ((mutual && !(a < b)) || pairs_.count(key(a, b)) ||
        candidate_idle(a) || idle(b))
      {
        continue;
      }
      // One REPLAN per AMR at a time (as for parked / goal-on-path).
      if (!mutual && (involved(a) || std::any_of(
          parked_.begin(), parked_.end(),
          [&](const auto & kv) {return kv.second.candidate == a;})))
      {
        continue;
      }
      const auto pb = cache.get(b);
      // Mutual hold on opposite paths (head-on) or on any shared place (a
      // crossing): either way neither AMR can ever go first.
      const bool head_on = pb && opposite_overlap(path, *pb, config_.clearance_m);
      bool shares = head_on;
      for (std::size_t i = 0; pb && !shares && i < path.waypoints.size(); ++i) {
        shares = near_any(path.waypoints[i], *pb, config_.clearance_m);
      }
      if (!shares) {
        continue;
      }
      std::string fcfs;
      const std::string later = mutual ? choose_replan_candidate(a, b, cache, &fcfs) : a;
      if (!mutual) {
        fcfs = "waits for " + b;
      }
      const std::string first = later == a ? b : a;
      // Just after a CLEAR the paths may still be settling (the candidate's
      // detour is being re-planned or released): do not start over at once.
      const auto cleared = cleared_at_.find(key(a, b));
      if (cleared != cleared_at_.end() &&
        now_sec - cleared->second < config_.reopen_guard_sec)
      {
        continue;
      }
      // First come, first served: only the later AMR replans, and only if it
      // still reaches its goal around the first one; otherwise both keep
      // waiting (no closures, no RMF planning failure).
      const auto lp = cache.get(later);
      const auto fp = cache.get(first);
      if (!lp || !fp) {
        continue;
      }
      // Hard rule: first's goal on later's path — alternate → REPLAN, else wait.
      bool goal_blocks = false;
      for (std::size_t j = 1; j + 1 < lp->waypoints.size() && !goal_blocks; ++j) {
        goal_blocks = near(fp->waypoints.back(), lp->waypoints[j], config_.clearance_m);
      }
      const auto route = plan_route(*lp, *fp, now_sec, &cache);
      if (goal_blocks) {
        Pair p;
        p.candidate = later;
        p.keeper = first;
        p.candidate_path = *lp;
        p.keeper_path = *fp;
        if (!route.ok) {
          if (route.closed.empty() && route.attempt <= 0) {
            note(key(a, b), "no time overlap with " + first + " — keep waiting");
            continue;
          }
          out.push_back(make(
            ReplanDecisionKind::NoAlternative, p,
            "no detour path is available after " +
            std::to_string(config_.replan_max_retries) + " attempts", cache));
          out.push_back(make(
            ReplanDecisionKind::Clear, p,
            "no detour path is available (" + first +
            "'s goal is on the path) — keep waiting", cache));
          p.gave_up = true;
          cleared_at_[key(a, b)] = now_sec;
          pairs_[key(a, b)] = std::move(p);
          continue;
        }
        p.blocked = route.closed;
        p.until = route.until;
        p.exit = route.exit;
        start(
          p, cache, now_sec,
          first + "'s goal is on the path; alternate taken", out);
        if (!p.blocked.empty()) {
          pairs_[key(a, b)] = std::move(p);
        }
        continue;
      }
      const std::string pk = key(a, b);
      if (!route.ok) {
        if (route.closed.empty() && route.attempt <= 0) {
          note(pk, "no time overlap with " + first + " — keep waiting");
          continue;
        }
        // Mutual head-on/crossing: A3 fallback after all retries.
        if (mutual) {
          Pair p;
          p.candidate = later;
          p.keeper = first;
          p.candidate_path = *lp;
          p.keeper_path = *fp;
          out.push_back(make(
            ReplanDecisionKind::NoAlternative, p,
            "no detour path is available after " +
            std::to_string(config_.replan_max_retries) + " attempts", cache));
          out.push_back(make(
            ReplanDecisionKind::Clear, p,
            "no detour path is available — keep waiting", cache));
          p.gave_up = true;
          cleared_at_[key(a, b)] = now_sec;
          pairs_[key(a, b)] = std::move(p);
          continue;
        }
        const auto ew = eta_waiting(*lp, *fp, now_sec);
        note(pk, "no detour path is available for " + later + " around " + first + " (closed " +
          std::to_string(route.closed.size()) + " waypoints); wait ETA " +
          (ew ? std::to_string(static_cast<int>(*ew)) + " s" : std::string("n/a")) +
          " — keep waiting, retrying");
        continue;
      }
      // Replan only in the planning moment; after that the AMRs just wait.
      if (!planning(*lp, now_sec)) {
        note(pk, "planning window over for " + later + " — keep waiting");
        continue;
      }
      // Compare travel+wait vs detour when waiting can solve the hold.
      const auto eta_wait = eta_waiting(*lp, *fp, now_sec);
      const auto eta_alt = eta_replanned(*lp, route.closed);
      // No ETA but the candidate is not standing on the keeper's path: the
      // hold is an approach/base wait that clears by itself — keep waiting.
      const std::string etas =
        std::string("wait ETA ") + (eta_wait ? std::to_string(static_cast<int>(*eta_wait)) : "n/a") +
        " s vs replanned " + (eta_alt ? std::to_string(static_cast<int>(*eta_alt)) : "n/a") + " s";
      if (!eta_wait && !near_any(lp->waypoints.front(), *fp, config_.clearance_m)) {
        note(pk, later + " approach wait clears by itself (" + etas + ") — keep waiting");
        continue;
      }
      // One-way wait: replan only if the detour is provably sooner.
      if (!mutual && !(eta_wait && eta_alt && *eta_alt < *eta_wait)) {
        note(pk, later + " waits for " + first + ": " + etas + " — keep waiting");
        continue;
      }
      note(pk, later + " vs " + first + ": " + etas);
      // Waiting solvable and not faster than detour → keep waiting.
      if (eta_wait && eta_alt && !(*eta_alt < *eta_wait)) {
        Pair p;
        p.candidate = later;
        p.keeper = first;
        p.gave_up = true;
        p.candidate_path = *lp;
        p.keeper_path = *fp;
        out.push_back(make(
          ReplanDecisionKind::Clear, p,
          "waiting is sooner: ETA " + std::to_string(static_cast<int>(*eta_wait)) +
          " s vs replanned " + std::to_string(static_cast<int>(*eta_alt)) + " s", cache));
        cleared_at_[key(a, b)] = now_sec;
        pairs_[key(a, b)] = std::move(p);
        continue;
      }
      if (eta_wait && eta_alt && *eta_alt < *eta_wait &&
        detour_still_meets_keeper(*lp, *fp, route.closed))
      {
        note(
          pk,
          later + " detour still meets " + first + " (" + etas + ") — keep waiting");
        continue;
      }
      // Detour sooner (clear of keeper), or waiting cannot solve → replan.
      std::string why =
        std::string(head_on ? "head-on" : "crossing") + " with " + first + "; " + fcfs;
      if (eta_wait && eta_alt) {
        why += "; detour sooner: wait ETA " +
          std::to_string(static_cast<int>(*eta_wait)) + " s vs replanned " +
          std::to_string(static_cast<int>(*eta_alt)) + " s";
      } else if (!eta_wait) {
        why += "; waiting cannot solve — replan";
      }
      Pair p;
      p.candidate = later;
      p.keeper = first;
      p.blocked = route.closed;
      p.until = route.until;
      p.exit = route.exit;
      start(p, cache, now_sec, why + " (attempt " + std::to_string(route.attempt) + ")", out);
      if (!p.blocked.empty()) {
        pairs_[key(a, b)] = std::move(p);
      }
    }
  }
  return out;
}

}  // namespace ecbs_planner
