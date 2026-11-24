#include "ecbs_planner/schedule_cache.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

#include "ecbs_planner/path_utils.hpp"

namespace ecbs_planner
{

/// Insert/replace TimedPath keyed by amr_id.
void ScheduleCache::upsert(const TimedPath & path)
{
  // Refuse empty keys (would collide / be unusable).
  if (path.amr_id.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  TimedPath stored = path;
  ensure_route_occurrences(stored);  // immutable task-route indices
  by_amr_[path.amr_id] = std::move(stored);
  // New/replaced path: reset pose hint (local indices may no longer match).
  last_pose_wp_[path.amr_id] = 0;
  // FCFS: only a new task_id advances the accept order (not path_revision).
  const auto it = accepted_.find(path.amr_id);
  if (it == accepted_.end() ||
    (!path.task_id.empty() && it->second.first != path.task_id))
  {
    accepted_[path.amr_id] = {path.task_id, ++accept_seq_};
  }
}

/// Drop one AMR entry.
void ScheduleCache::erase(const std::string & amr_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  by_amr_.erase(amr_id);
  last_pose_wp_.erase(amr_id);
  granted_.erase(amr_id);
  forget_holds_locked(amr_id);
  hold_task_.erase(amr_id);
  reserved_.erase(amr_id);
  reserve_waits_.erase(amr_id);
  reserve_conflicts_.erase(amr_id);
  accepted_.erase(amr_id);
  idle_.erase(amr_id);
  busy_task_.erase(amr_id);
}

/// Drop every entry.
void ScheduleCache::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  by_amr_.clear();
  last_pose_wp_.clear();
  granted_.clear();
  holds_.clear();
  registered_.clear();
  pinned_.clear();
  hold_task_.clear();
  reserved_.clear();
  reserve_waits_.clear();
  reserve_conflicts_.clear();
  poses_.clear();
  accepted_.clear();
  idle_.clear();
  busy_task_.clear();
}

/// Lookup one AMR; returns a copy so callers can mutate safely.
std::optional<TimedPath> ScheduleCache::get(const std::string & amr_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = by_amr_.find(amr_id);
  if (it == by_amr_.end()) {
    return std::nullopt;  // unknown AMR
  }
  return it->second;
}

/// Snapshot of all paths except optionally one AMR (the candidate).
std::vector<TimedPath> ScheduleCache::all_except(const std::string & exclude_amr) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<TimedPath> out;
  out.reserve(by_amr_.size());
  for (const auto & kv : by_amr_) {
    // Skip the excluded AMR when exclude_amr is non-empty.
    if (!exclude_amr.empty() && kv.first == exclude_amr) {
      continue;
    }
    out.push_back(kv.second);
  }
  return out;
}

/// How many AMRs are stored.
std::size_t ScheduleCache::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return by_amr_.size();
}

bool ScheduleCache::erase_before_pose(
  const std::string & amr_id, double x, double y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return erase_before_pose_locked(amr_id, x, y);
}

bool ScheduleCache::erase_before_pose_locked(
  const std::string & amr_id, double x, double y)
{
  auto it = by_amr_.find(amr_id);
  if (it == by_amr_.end() || it->second.waypoints.empty()) {
    return false;
  }
  auto & wps = it->second.waypoints;
  const std::size_t hint = last_pose_wp_.count(amr_id) ? last_pose_wp_[amr_id] : 0;
  const std::size_t best = nearest_waypoint_index(it->second, x, y, hint);
  // Rolling view only: drop passed geometry. route_occurrence on remaining
  // waypoints is preserved — never renumber / reinterpret TM grant tips.
  last_pose_wp_[amr_id] = 0;
  if (best == 0) {
    return false;
  }
  wps.erase(wps.begin(), wps.begin() + static_cast<std::ptrdiff_t>(best));
  return true;
}

void ScheduleCache::set_granted_base(
  const std::string & amr_id, const GrantedBase & base)
{
  if (amr_id.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  granted_[amr_id] = base;
}

std::optional<GrantedBase> ScheduleCache::granted_base(
  const std::string & amr_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = granted_.find(amr_id);
  if (it == granted_.end()) {
    return std::nullopt;
  }
  return it->second;
}

void ScheduleCache::clear_granted_base(const std::string & amr_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  granted_.erase(amr_id);
}

bool ScheduleCache::pair_registered_locked(
  const std::string & a, const std::string & b) const
{
  const auto ia = registered_.find(a);
  if (ia != registered_.end() && ia->second.count(b) > 0) {
    return true;
  }
  const auto ib = registered_.find(b);
  return ib != registered_.end() && ib->second.count(a) > 0;
}

void ScheduleCache::add_hold_locked(
  const std::string & waiter, const WaitDependency & dep, bool replace_wait_point)
{
  if (waiter.empty() || dep.amr_id.empty() || waiter == dep.amr_id) {
    return;
  }
  if (pair_registered_locked(waiter, dep.amr_id)) {
    return;
  }
  const auto row = right_of_way_.find(waiter);
  if (row != right_of_way_.end() && row->second.count(dep.amr_id) > 0) {
    return;
  }
  auto & by_blocker = holds_[waiter];
  const auto it = by_blocker.find(dep.amr_id);
  if (it == by_blocker.end()) {
    by_blocker.emplace(dep.amr_id, dep);
    return;
  }
  if (replace_wait_point && !pinned_locked(waiter, dep.amr_id)) {
    it->second.wait_point = dep.wait_point;
  }
  if (!dep.clear_points.empty()) {
    it->second.clear_points = dep.clear_points;
  }
}

bool ScheduleCache::pinned_locked(
  const std::string & waiter, const std::string & blocker) const
{
  const auto it = pinned_.find(waiter);
  return it != pinned_.end() && it->second.count(blocker) > 0;
}

void ScheduleCache::unpin_locked(const std::string & waiter, const std::string & blocker)
{
  const auto it = pinned_.find(waiter);
  if (it != pinned_.end()) {
    it->second.erase(blocker);
  }
}

void ScheduleCache::forget_outgoing_holds_locked(const std::string & amr_id)
{
  holds_.erase(amr_id);
  registered_.erase(amr_id);
  pinned_.erase(amr_id);
  // Old ZM GO/register naming this AMR as blocker is stale on task change;
  // inbound occupancy holds (holds_[waiter][amr_id]) stay until clear/pass.
  for (auto & kv : registered_) {
    kv.second.erase(amr_id);
  }
  for (auto & kv : pinned_) {
    kv.second.erase(amr_id);
  }
}

void ScheduleCache::forget_holds_locked(const std::string & amr_id)
{
  forget_outgoing_holds_locked(amr_id);
  for (auto & kv : holds_) {
    kv.second.erase(amr_id);
  }
}


void ScheduleCache::set_right_of_way(const std::string & goer, const std::string & yielder)
{
  std::lock_guard<std::mutex> lock(mutex_);
  right_of_way_[goer].insert(yielder);
}

void ScheduleCache::clear_right_of_way(const std::string & goer, const std::string & yielder)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = right_of_way_.find(goer);
  if (it != right_of_way_.end()) {
    it->second.erase(yielder);
  }
}

bool ScheduleCache::has_right_of_way(const std::string & goer, const std::string & other) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = right_of_way_.find(goer);
  return it != right_of_way_.end() && it->second.count(other) > 0;
}

void ScheduleCache::pin_hold(const std::string & waiter, const WaitDependency & dep)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto iw = holds_.find(waiter);
  if (iw == holds_.end()) {
    return;
  }
  const auto it = iw->second.find(dep.amr_id);
  if (it == iw->second.end() || pinned_locked(waiter, dep.amr_id)) {
    return;
  }
  it->second.wait_point = dep.wait_point;
  pinned_[waiter].insert(dep.amr_id);
}

void ScheduleCache::force_hold_wait_point(
  const std::string & waiter, const WaitDependency & dep)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (waiter.empty() || dep.amr_id.empty()) {
    return;
  }
  const auto iw = holds_.find(waiter);
  if (iw == holds_.end()) {
    return;
  }
  const auto it = iw->second.find(dep.amr_id);
  if (it == iw->second.end()) {
    return;
  }
  it->second.wait_point = dep.wait_point;
  if (!dep.clear_points.empty()) {
    it->second.clear_points = dep.clear_points;
  }
}

bool ScheduleCache::hold_pinned(
  const std::string & waiter, const std::string & blocker) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return pinned_locked(waiter, blocker);
}

void ScheduleCache::add_hold(
  const std::string & waiter, const WaitDependency & dep, bool replace_wait_point)
{
  std::lock_guard<std::mutex> lock(mutex_);
  add_hold_locked(waiter, dep, replace_wait_point);
}

void ScheduleCache::add_plan_holds(const TimedPath & path)
{
  if (path.amr_id.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!path.task_id.empty()) {
    auto & task = hold_task_[path.amr_id];
    if (!task.empty() && task != path.task_id) {
      forget_outgoing_holds_locked(path.amr_id);
    }
    task = path.task_id;
  }
  for (const auto & wp : path.waypoints) {
    for (const auto & dep : wp.waiting_for) {
      WaitDependency copy = dep;
      const bool unset = copy.wait_point.wait_duration_sec <= 1e-6 &&
        !copy.wait_point.index.has_value() &&
        std::abs(copy.wait_point.x) < 1e-9 && std::abs(copy.wait_point.y) < 1e-9;
      if (unset) {
        copy.wait_point.x = wp.x;
        copy.wait_point.y = wp.y;
        copy.wait_point.index = wp.index;
        copy.wait_point.wait_duration_sec = wp.wait_duration_sec;
      }
      add_hold_locked(path.amr_id, copy, true);
    }
  }
}

std::vector<WaitDependency> ScheduleCache::holds(const std::string & waiter) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<WaitDependency> out;
  const auto it = holds_.find(waiter);
  if (it == holds_.end()) {
    return out;
  }
  out.reserve(it->second.size());
  for (const auto & kv : it->second) {
    out.push_back(kv.second);
  }
  return out;
}

bool ScheduleCache::has_hold(
  const std::string & waiter, const std::string & blocker) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = holds_.find(waiter);
  return it != holds_.end() && it->second.count(blocker) > 0;
}

std::vector<std::string> ScheduleCache::waiters_for(const std::string & blocker) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> out;
  for (const auto & kv : holds_) {
    if (kv.second.count(blocker) > 0) {
      out.push_back(kv.first);
    }
  }
  return out;
}

void ScheduleCache::remove_hold(
  const std::string & waiter, const std::string & blocker)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto iw = holds_.find(waiter);
  if (iw != holds_.end()) {
    iw->second.erase(blocker);
  }
  unpin_locked(waiter, blocker);
}

void ScheduleCache::mark_registered(
  const std::string & waiter, const std::string & blocker)
{
  if (waiter.empty() || blocker.empty() || waiter == blocker) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  registered_[waiter].insert(blocker);
  const auto iw = holds_.find(waiter);
  if (iw != holds_.end()) {
    iw->second.erase(blocker);
  }
  const auto ib = holds_.find(blocker);
  if (ib != holds_.end()) {
    ib->second.erase(waiter);
  }
  unpin_locked(waiter, blocker);
  unpin_locked(blocker, waiter);
}

void ScheduleCache::unmark_registered(
  const std::string & waiter, const std::string & blocker)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = registered_.find(waiter);
  if (it != registered_.end()) {
    it->second.erase(blocker);
  }
}

std::vector<std::string> ScheduleCache::registered_blockers(const std::string & waiter) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = registered_.find(waiter);
  if (it == registered_.end()) {
    return {};
  }
  return {it->second.begin(), it->second.end()};
}

bool ScheduleCache::registered_waiter(
  const std::string & waiter, const std::string & blocker) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = registered_.find(waiter);
  return it != registered_.end() && it->second.count(blocker) > 0;
}

void ScheduleCache::set_reserved(
  const std::string & amr_id, const std::vector<ClearPoint> & cells)
{
  if (amr_id.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (cells.empty()) {
    reserved_.erase(amr_id);
    return;
  }
  const auto prev = reserved_.find(amr_id);
  std::vector<ReservedCell> out;
  out.reserve(cells.size());
  const uint64_t fresh = ++reserve_seq_;
  for (const auto & c : cells) {
    ReservedCell cell{c.x, c.y, fresh};
    if (prev != reserved_.end()) {
      for (const auto & p : prev->second) {
        if (std::hypot(p.x - c.x, p.y - c.y) < 1e-3) {
          cell.since = p.since;
          break;
        }
      }
    }
    out.push_back(cell);
  }
  reserved_[amr_id] = std::move(out);
}

std::vector<ReservedCell> ScheduleCache::reserved(const std::string & amr_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = reserved_.find(amr_id);
  return it == reserved_.end() ? std::vector<ReservedCell>{} : it->second;
}

std::unordered_map<std::string, std::vector<ReservedCell>> ScheduleCache::reserved_except(
  const std::string & exclude_amr) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto out = reserved_;
  for (const auto & kv : poses_) {
    if (kv.first != exclude_amr && standing_locked(kv.first, kv.second)) {
      out[kv.first].push_back({kv.second.x, kv.second.y, 0});
    }
  }
  out.erase(exclude_amr);
  return out;
}

void ScheduleCache::set_pose(const std::string & amr_id, double x, double y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  poses_[amr_id] = {x, y};
}

void ScheduleCache::set_base_tail_m(double tail_m)
{
  std::lock_guard<std::mutex> lock(mutex_);
  base_tail_m_ = std::max(0.0, tail_m);
}

double ScheduleCache::base_tail_m() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return base_tail_m_;
}

void ScheduleCache::update_pose(const std::string & amr_id, double x, double y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  poses_[amr_id] = {x, y};
  erase_before_pose_locked(amr_id, x, y);

  auto it = reserved_.find(amr_id);
  if (it == reserved_.end() || it->second.empty()) {
    return;
  }
  // Cells run from where the stretch was given towards its end: the robot is
  // at the first closest cell going forward (a U-turn later in the stretch
  // may pass closer, but the robot is not there yet).
  auto & cells = it->second;
  auto dist = [&](std::size_t i) {return std::hypot(cells[i].x - x, cells[i].y - y);};
  std::size_t at = 0;
  while (at + 1 < cells.size() && dist(at + 1) <= dist(at)) {
    ++at;
  }
  // A curve / bend can put a false local minimum early in the stretch (cells
  // first move away from the robot, then come back): the walk above stops
  // there although the robot is far from that cell, and the whole base stays
  // reserved however far the robot has driven on. A robot that is not on the
  // cell it "reached" has driven on: take the closest cell further on.
  constexpr double kOnCellM = 1.0;
  if (dist(at) > kOnCellM) {
    std::size_t best = at;
    for (std::size_t j = at + 1; j < cells.size(); ++j) {
      if (dist(j) < dist(best)) {
        best = j;
      }
    }
    at = best;
  }
  double back_m = 0.0;
  while (at > 0) {
    const double seg =
      std::hypot(cells[at].x - cells[at - 1].x, cells[at].y - cells[at - 1].y);
    if (back_m + seg > base_tail_m_ + 1e-9) {
      break;
    }
    back_m += seg;
    --at;
  }
  cells.erase(cells.begin(), cells.begin() + static_cast<std::ptrdiff_t>(at));
}

void ScheduleCache::set_idle(const std::string & amr_id, bool idle)
{
  if (amr_id.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const auto path = by_amr_.find(amr_id);
  if (!idle) {
    idle_.erase(amr_id);
    busy_task_[amr_id] = path == by_amr_.end() ? std::string{} : path->second.task_id;
    return;
  }
  if (!idle_.insert(amr_id).second) {
    return;
  }
  const auto busy = busy_task_.find(amr_id);
  if (busy == busy_task_.end()) {
    return;
  }
  const bool new_task_seeded = path != by_amr_.end() && path->second.task_id != busy->second;
  busy_task_.erase(busy);
  if (new_task_seeded) {
    return;
  }
  by_amr_.erase(amr_id);
  last_pose_wp_.erase(amr_id);
  granted_.erase(amr_id);
  reserved_.erase(amr_id);
  reserve_waits_.erase(amr_id);
  reserve_conflicts_.erase(amr_id);
  holds_.erase(amr_id);
  pinned_.erase(amr_id);
  registered_.erase(amr_id);
  hold_task_.erase(amr_id);
}

bool ScheduleCache::standing(const std::string & amr_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = poses_.find(amr_id);
  return it != poses_.end() && standing_locked(amr_id, it->second);
}

std::optional<ClearPoint> ScheduleCache::standing_pose(const std::string & amr_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = poses_.find(amr_id);
  if (it == poses_.end() || !standing_locked(amr_id, it->second)) {
    return std::nullopt;
  }
  return it->second;
}

std::optional<ClearPoint> ScheduleCache::pose(const std::string & amr_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = poses_.find(amr_id);
  if (it == poses_.end()) {
    return std::nullopt;
  }
  return it->second;
}

void ScheduleCache::note_path(const std::string & amr_id, const std::string & task_key)
{
  if (amr_id.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = accepted_.find(amr_id);
  if (it != accepted_.end() && (task_key.empty() || it->second.first == task_key)) {
    return;
  }
  accepted_[amr_id] = {task_key, ++accept_seq_};
}

bool ScheduleCache::accepted_before(const std::string & a, const std::string & b) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto ia = accepted_.find(a);
  const auto ib = accepted_.find(b);
  return ia != accepted_.end() && ib != accepted_.end() && ia->second.second < ib->second.second;
}

bool ScheduleCache::standing_locked(const std::string & amr_id, const ClearPoint & pose) const
{
  if (idle_.count(amr_id) > 0) {
    return true;
  }
  const auto path = by_amr_.find(amr_id);
  if (path == by_amr_.end() || path->second.waypoints.size() <= 1) {
    return true;
  }
  const auto & last = path->second.waypoints.back();
  if (std::hypot(last.x - pose.x, last.y - pose.y) <= kArrivedM) {
    return true;
  }
  auto any = [&amr_id](const auto & by_waiter) {
      const auto it = by_waiter.find(amr_id);
      return it != by_waiter.end() && !it->second.empty();
    };
  return any(holds_) || any(registered_) || any(reserve_waits_);
}

void ScheduleCache::set_reserve_waits(
  const std::string & waiter, std::unordered_set<std::string> blockers)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (blockers.empty()) {
    reserve_waits_.erase(waiter);
  } else {
    reserve_waits_[waiter] = std::move(blockers);
  }
}

std::unordered_set<std::string> ScheduleCache::reserve_waits(const std::string & waiter) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = reserve_waits_.find(waiter);
  return it == reserve_waits_.end() ? std::unordered_set<std::string>{} : it->second;
}

void ScheduleCache::set_reserve_conflict(
  const std::string & waiter, const std::string & blocker, const ClearPoint & at)
{
  std::lock_guard<std::mutex> lock(mutex_);
  reserve_conflicts_[waiter][blocker] = at;
}

void ScheduleCache::keep_reserve_conflicts(
  const std::string & waiter, const std::unordered_set<std::string> & blockers)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto conflicts = reserve_conflicts_.find(waiter);
  if (conflicts == reserve_conflicts_.end()) {
    return;
  }
  for (auto it = conflicts->second.begin(); it != conflicts->second.end(); ) {
    it = blockers.count(it->first) > 0 ? std::next(it) : conflicts->second.erase(it);
  }
  if (conflicts->second.empty()) {
    reserve_conflicts_.erase(conflicts);
  }
}

std::optional<ClearPoint> ScheduleCache::reserve_conflict(
  const std::string & waiter, const std::string & blocker) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = reserve_conflicts_.find(waiter);
  if (it == reserve_conflicts_.end()) {
    return std::nullopt;
  }
  const auto c = it->second.find(blocker);
  if (c == it->second.end()) {
    return std::nullopt;
  }
  return c->second;
}

}  // namespace ecbs_planner

