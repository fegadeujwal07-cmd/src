#include "ecbs_planner/proposal_engine.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>
#include <unordered_set>

#include "ecbs_planner/action_compatibility.hpp"
#include "ecbs_planner/passage_action.hpp"
#include "ecbs_planner/path_utils.hpp"
#include "ecbs_planner/spatial_index.hpp"
#include "ecbs_planner/wait_reason.hpp"

namespace ecbs_planner
{
namespace
{

std::size_t path_content_hash(const TimedPath & path)
{
  std::size_t h = 0;
  auto mix = [&](std::size_t v) {
      h ^= v + 0x9e3779b9 + (h << 6) + (h >> 2);
    };
  mix(std::hash<std::string>{}(path.amr_id));
  mix(path.path_revision);
  mix(path.waypoints.size());
  for (const auto & w : path.waypoints) {
    mix(std::hash<double>{}(w.x));
    mix(std::hash<double>{}(w.y));
    mix(std::hash<std::string>{}(w.level_name));
  }
  return h;
}

double path_length_to_index(const TimedPath & path, std::size_t through)
{
  if (path.waypoints.empty()) {
    return 0.0;
  }
  through = std::min(through, path.waypoints.size() - 1);
  double L = 0.0;
  for (std::size_t i = 1; i <= through; ++i) {
    L += std::hypot(
      path.waypoints[i].x - path.waypoints[i - 1].x,
      path.waypoints[i].y - path.waypoints[i - 1].y);
  }
  return L;
}

int32_t clamp_index(const TimedPath & path, int32_t idx)
{
  if (path.waypoints.empty()) {
    return -1;
  }
  if (idx < 0) {
    return 0;
  }
  if (static_cast<std::size_t>(idx) >= path.waypoints.size()) {
    return static_cast<int32_t>(path.waypoints.size() - 1);
  }
  return idx;
}

int32_t index_for_extension(
  const TimedPath & path,
  int32_t from_index,
  double desired_m)
{
  from_index = clamp_index(path, from_index);
  if (from_index < 0) {
    return -1;
  }
  double L = 0.0;
  std::size_t i = static_cast<std::size_t>(from_index);
  while (i + 1 < path.waypoints.size() && L < desired_m) {
    L += std::hypot(
      path.waypoints[i + 1].x - path.waypoints[i].x,
      path.waypoints[i + 1].y - path.waypoints[i].y);
    ++i;
  }
  return static_cast<int32_t>(i);
}

WaitDependency wait_dep_from_record(
  const ConflictRecord & r, const std::string & waiter_id)
{
  WaitDependency d;
  d.amr_id = r.blocker_amr;
  if (d.amr_id.empty()) {
    for (const auto & p : r.participant_amrs()) {
      if (p != waiter_id) {
        d.amr_id = p;
        break;
      }
    }
  }
  d.topology = r.topology;
  if (const auto * self = r.participant(waiter_id)) {
    d.self_blocks_other = self->blocks_other;
    d.other_blocks_self = self->blocked_by_other;
    d.wait_point.x = self->wait.x;
    d.wait_point.y = self->wait.y;
    d.wait_point.index = self->wait.route_occurrence;
  }
  for (const auto & p : r.participants) {
    if (p.amr_id != waiter_id) {
      d.clear_points.push_back({p.exit.x, p.exit.y});
    }
  }
  return d;
}

/// Owners whose reserved base intersects [from, through] (empty = clear).
std::vector<std::string> foreign_base_owners_blocking(
  const ScheduleCache & cache,
  const std::string & amr_id,
  const TimedPath & path,
  int32_t from_index,
  int32_t through_index)
{
  std::vector<std::string> owners;
  from_index = clamp_index(path, from_index);
  through_index = clamp_index(path, through_index);
  if (from_index < 0 || through_index < 0) {
    return owners;
  }
  if (through_index < from_index) {
    std::swap(through_index, from_index);
  }
  const auto reserved = cache.reserved_except(amr_id);
  constexpr double hit_m = 0.5;
  const double hit2 = hit_m * hit_m;
  for (int32_t i = from_index; i <= through_index; ++i) {
    const auto & w = path.waypoints[static_cast<std::size_t>(i)];
    for (const auto & kv : reserved) {
      if (std::find(owners.begin(), owners.end(), kv.first) != owners.end()) {
        continue;
      }
      for (const auto & c : kv.second) {
        const double dx = w.x - c.x;
        const double dy = w.y - c.y;
        if (dx * dx + dy * dy <= hit2) {
          owners.push_back(kv.first);
          break;
        }
      }
    }
  }
  std::sort(owners.begin(), owners.end());
  return owners;
}

/// Foreign-base check only over the intended movement window [from, through].
bool foreign_base_blocks_range(
  const ScheduleCache & cache,
  const std::string & amr_id,
  const TimedPath & path,
  int32_t from_index,
  int32_t through_index)
{
  return !foreign_base_owners_blocking(
    cache, amr_id, path, from_index, through_index).empty();
}

bool grant_intersects_conflict(
  const ConflictRecord & rec,
  const std::string & amr_id,
  const CommittedGrantView & grant,
  const uint32_t path_revision)
{
  const auto * part = rec.participant(amr_id);
  if (!part || rec.physically_cleared) {
    return false;
  }
  uint32_t entry_occ = part->entry.route_occurrence;
  if (part->entry.path_revision != path_revision && path_revision != 0) {
    // Prefer matching revision when present among participants.
    for (const auto & p : rec.participants) {
      if (p.amr_id == amr_id && p.entry.path_revision == path_revision) {
        entry_occ = p.entry.route_occurrence;
        break;
      }
    }
  }
  return static_cast<int32_t>(entry_occ) <= grant.release_through_index;
}

bool waypoint_has_wait(const TimedPoint & wp)
{
  return wp.wait_duration_sec > 1e-6 || !wp.waiting_for.empty();
}

/// First local index at/after \p from with an evaluate/path wait, if any.
std::optional<int32_t> first_path_wait_at_or_after(
  const TimedPath & path, int32_t from)
{
  if (path.waypoints.empty()) {
    return std::nullopt;
  }
  from = std::max(0, from);
  for (std::size_t i = static_cast<std::size_t>(from);
    i < path.waypoints.size(); ++i)
  {
    if (waypoint_has_wait(path.waypoints[i])) {
      return static_cast<int32_t>(i);
    }
  }
  return std::nullopt;
}

/// Nearest local index for a hold wait_point (xy / optional graph index).
int32_t hold_local_index(const TimedPath & path, const PathWaitPoint & wp)
{
  if (path.waypoints.empty()) {
    return 0;
  }
  double best = 1e100;
  int32_t best_i = 0;
  for (std::size_t i = 0; i < path.waypoints.size(); ++i) {
    const double d = std::hypot(
      path.waypoints[i].x - wp.x, path.waypoints[i].y - wp.y);
    if (d < best) {
      best = d;
      best_i = static_cast<int32_t>(i);
    }
  }
  return best_i;
}

void append_unique_wait_dep(
  std::vector<WaitDependency> & out, const WaitDependency & dep)
{
  if (dep.amr_id.empty()) {
    return;
  }
  for (const auto & e : out) {
    if (e.amr_id == dep.amr_id) {
      return;
    }
  }
  out.push_back(dep);
}

void fill_waiting_for(
  ProposalQueryResult & out,
  const TimedPath & path,
  int32_t from_idx,
  const std::vector<ConflictRecord> & cluster_records,
  const ScheduleCache & cache,
  const std::string & amr_id)
{
  out.waiting_for.clear();
  for (const auto & rec : cluster_records) {
    if (rec.physically_cleared || !rec.participant(amr_id)) {
      continue;
    }
    append_unique_wait_dep(out.waiting_for, wait_dep_from_record(rec, amr_id));
  }
  for (const auto & dep : cache.holds(amr_id)) {
    append_unique_wait_dep(out.waiting_for, dep);
  }
  const int32_t lo = std::max(0, from_idx);
  for (std::size_t i = static_cast<std::size_t>(lo);
    i < path.waypoints.size(); ++i)
  {
    for (const auto & dep : path.waypoints[i].waiting_for) {
      append_unique_wait_dep(out.waiting_for, dep);
    }
    if (!out.waiting_for.empty()) {
      break;
    }
  }
}

ProposalQueryResult make_wait_at_hold(
  ProposalEngineState & state,
  const TimedPath & path,
  int32_t from_idx,
  int32_t hold_local,
  const std::vector<ConflictRecord> & cluster_records,
  const ScheduleCache & cache,
  const std::string & amr_id,
  const std::string & reason)
{
  ProposalQueryResult out;
  out.success = true;
  out.status = kStatusWait;
  out.wait_reason_code = kWaitClearance;
  out.wait_reason = reason;
  out.decision_reason = reason;
  out.message = reason;
  fill_waiting_for(out, path, hold_local, cluster_records, cache, amr_id);
  // Ensure TM gets a concrete wait_point even if deps lacked geometry.
  if (!out.waiting_for.empty() &&
    out.waiting_for.front().wait_point.wait_duration_sec <= 1e-6 &&
    std::abs(out.waiting_for.front().wait_point.x) < 1e-9 &&
    std::abs(out.waiting_for.front().wait_point.y) < 1e-9)
  {
    const int32_t i = clamp_index(path, hold_local);
    if (i >= 0) {
      out.waiting_for.front().wait_point.x = path.waypoints[
        static_cast<std::size_t>(i)].x;
      out.waiting_for.front().wait_point.y = path.waypoints[
        static_cast<std::size_t>(i)].y;
      out.waiting_for.front().wait_point.index =
        path.waypoints[static_cast<std::size_t>(i)].route_occurrence;
    }
  }
  const int32_t tip = clamp_index(path, std::max(from_idx, 0));
  if (tip >= 0) {
    out.release_through_index = static_cast<int32_t>(
      occurrence_at_local(path, static_cast<std::size_t>(tip)));
    out.from_occurrence = make_occurrence(
      path.path_revision,
      occurrence_at_local(path, static_cast<std::size_t>(tip)),
      path.waypoints[static_cast<std::size_t>(tip)].x,
      path.waypoints[static_cast<std::size_t>(tip)].y);
    out.to_occurrence = out.from_occurrence;
  }
  out.relevant_conflicts = cluster_records;
  state.groups.set_pending_extension(amr_id, true);
  return out;
}

}  // namespace

RouteStamp make_route_stamp(const TimedPath & path)
{
  RouteStamp s;
  s.path_revision = path.path_revision;
  s.points.reserve(path.waypoints.size());
  for (const auto & w : path.waypoints) {
    std::size_t h = std::hash<double>{}(w.x);
    h ^= std::hash<double>{}(w.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= std::hash<std::string>{}(w.level_name) + 0x9e3779b9 + (h << 6) + (h >> 2);
    s.points.emplace_back(w.route_occurrence, h);
  }
  return s;
}

bool route_stamp_matches(const RouteStamp & stamp, const TimedPath & path)
{
  const auto cur = make_route_stamp(path);
  if (cur.path_revision != stamp.path_revision ||
    cur.points.empty() != stamp.points.empty())
  {
    return false;
  }
  if (cur.points.empty()) {
    return true;
  }
  // Compare the shared suffix: a prefix erase moves the front, never the rest.
  const uint32_t from = std::max(stamp.points.front().first, cur.points.front().first);
  auto at = [from](const RouteStamp & r) {
      return std::lower_bound(
        r.points.begin(), r.points.end(), from,
        [](const std::pair<uint32_t, std::size_t> & pt, uint32_t occ) {
          return pt.first < occ;
        });
    };
  const auto a = at(stamp);
  const auto b = at(cur);
  return stamp.points.end() - a == cur.points.end() - b &&
         std::equal(a, stamp.points.end(), b);
}

namespace
{

/// Changes whenever a grant is added / replaced / retired for any group member.
uint64_t group_authority_stamp(const ProposalEngineState & state, const std::string & amr_id)
{
  std::vector<std::string> members{amr_id};
  if (const auto * g = state.groups.group_for_amr(amr_id)) {
    members.assign(g->member_amrs.begin(), g->member_amrs.end());
  }
  std::sort(members.begin(), members.end());
  uint64_t h = 0;  // 0 = no member holds a grant
  for (const auto & m : members) {
    const auto it = state.commitments.find(m);
    if (it == state.commitments.end()) {
      continue;
    }
    h ^= std::hash<std::string>{}(m) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    h ^= it->second.commitment_version + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
  }
  return h;
}

/// Stale only if something this proposal depends on changed (F4); unrelated
/// robots and the global route/occupancy counters never invalidate it.
/// \p cache null → route stamps cannot be checked (conflict/authority still are).
bool proposal_is_stale(
  const ProposalEngineState & state, const ScheduleCache * cache,
  const PendingProposal & p)
{
  if (state.unresolved_authority.count(p.amr_id)) {
    return true;
  }
  for (const auto & kv : p.conflict_versions) {
    const auto r = state.conflicts.get(kv.first);
    if (!r || r->record_version != kv.second) {
      return true;
    }
  }
  if (cache) {
    for (const auto & kv : p.route_stamps) {
      const auto path = cache->get(kv.first);
      if (!path || !route_stamp_matches(kv.second, *path)) {
        return true;
      }
    }
  }
  return group_authority_stamp(state, p.amr_id) != p.group_authority_stamp;
}

}  // namespace

std::vector<std::string> refresh_conflicts_from_cache(
  ProposalEngineState & state,
  const ScheduleCache & cache,
  const double detect_distance_m,
  const double position_epsilon_m,
  const double safety_distance_m)
{
  const auto all = cache.all_except("");
  bool routes_changed = false;
  for (const auto & p : all) {
    const auto h = path_content_hash(p);
    const auto it = state.last_path_content_hash.find(p.amr_id);
    if (it == state.last_path_content_hash.end() || it->second != h) {
      routes_changed = true;
      state.last_path_content_hash[p.amr_id] = h;
    }
    state.current_task_by_robot[p.amr_id] = p.task_id;
  }
  if (routes_changed) {
    ++state.route_version;
  }

  RouteSpatialIndex index(std::max(2.0, detect_distance_m * 2.0));
  std::unordered_map<std::string, TimedPath> by_id;
  for (const auto & p : all) {
    by_id[p.amr_id] = p;
    index.upsert_path(p);
  }
  std::unordered_set<std::string> touched;
  for (const auto & self : all) {
    std::vector<TimedPath> others;
    std::unordered_set<std::string> seen;
    for (const auto & cid : index.candidates_for(self)) {
      const auto it = by_id.find(cid);
      if (it != by_id.end() && seen.insert(cid).second) {
        others.push_back(it->second);
      }
    }
    // Idle / single-point occupancy always considered.
    for (const auto & o : all) {
      if (o.amr_id == self.amr_id || !seen.insert(o.amr_id).second) {
        continue;
      }
      if (o.waypoints.size() <= 1) {
        others.push_back(o);
      }
    }
    auto found = discover_route_overlaps(
      self, others, detect_distance_m, position_epsilon_m, safety_distance_m,
      state.route_version, state.occupancy_version);
    for (auto & rec : found) {
      // Both directions find the same overlap with different exits; keeping
      // only the lower-id robot's view stops record_version ping-ponging.
      if (rec.participants.size() == 2 && rec.participants[1].amr_id < self.amr_id) {
        continue;
      }
      // Symmetric across both discovery directions, else record_version
      // ping-pongs once only one participant holds a grant.
      rec.tentative = true;
      for (const auto & pa : rec.participants) {
        if (state.commitments.count(pa.amr_id)) {
          rec.tentative = false;
        }
      }
      if (state.conflicts.upsert(rec)) {
        touched.insert(self.amr_id);
        for (const auto & p : rec.participant_amrs()) {
          touched.insert(p);
        }
      }
    }
  }
  return std::vector<std::string>(touched.begin(), touched.end());
}

ProposalQueryResult query_base_extension(
  ProposalEngineState & state,
  const ScheduleCache & cache,
  const ProposalQueryInput & in,
  const double detect_distance_m,
  const double position_epsilon_m,
  const double safety_distance_m)
{
  ProposalQueryResult out;
  out.success = true;

  const auto path_opt = cache.get(in.amr_id);
  if (!path_opt || path_opt->waypoints.empty()) {
    out.status = kStatusStaleRoute;
    out.message = "route not in ScheduleCache — reconcile /from_zone_manager";
    return out;
  }
  const auto & path = *path_opt;
  if (in.path_revision != 0 && path.path_revision != in.path_revision) {
    out.status = kStatusStaleRoute;
    out.message = "path_revision mismatch";
    return out;
  }

  // Canonical route identity: ScheduleCache path (planner/dispatch), not the
  // robot execution order id. Prefer dispatch_task_id when present.
  const std::string canonical_task = !path.dispatch_task_id.empty() ?
    path.dispatch_task_id : path.task_id;
  out.task_id = canonical_task;
  out.execution_order_id = in.execution_order_id;
  state.current_task_by_robot[in.amr_id] = canonical_task;

  // Priority facts are immutable by task identity — never relabel an old
  // priority onto a new task_id without replacing the priority itself.
  auto & fact = state.priorities[in.amr_id];
  fact.robot_id = in.amr_id;
  if (!canonical_task.empty() && fact.task_id != canonical_task) {
    fact.task_id = canonical_task;
    fact.task_priority.clear();
    fact.priority_version = 0;
  }
  fact.progress_occurrence = in.observed_progress.route_occurrence;
  fact.progress_valid = true;
  fact.progress_x = in.current_x;
  fact.progress_y = in.current_y;
  fact.progress_xy_valid = true;
  if (in.distance_origin == "POSE" || in.current_grant_through_index < 0) {
    double best = 1e100;
    std::size_t best_i = 0;
    for (std::size_t i = 0; i < path.waypoints.size(); ++i) {
      const double d = std::hypot(
        path.waypoints[i].x - in.current_x,
        path.waypoints[i].y - in.current_y);
      if (d < best) {
        best = d;
        best_i = i;
      }
    }
    // Progress is the immutable route occurrence, not the trimmed vector index.
    fact.progress_occurrence = occurrence_at_local(path, best_i);
  }

  refresh_conflicts_from_cache(
    state, cache, detect_distance_m, position_epsilon_m, safety_distance_m);

  const auto grant_it = state.commitments.find(in.amr_id);

  // AUTHORITY_CONFLICT: retain until explicitly reconciled (clearance / retire).
  if (state.unresolved_authority.count(in.amr_id)) {
    out.status = kStatusAuthorityConflict;
    out.authority_conflict = true;
    out.authority_conflict_id = state.unresolved_authority[in.amr_id];
    if (grant_it != state.commitments.end()) {
      out.grant_id_hit = grant_it->second.grant_id;
    }
    out.message =
      "unresolved AUTHORITY_CONFLICT — stop/reconcile required before extension";
    return out;
  }

  // Always check grant vs active overlaps (not only when store version bumped).
  if (grant_it != state.commitments.end() &&
    grant_it->second.release_through_index >= 0)
  {
    for (const auto & rec : state.conflicts.active_for_robot(in.amr_id)) {
      if (!grant_intersects_conflict(
          rec, in.amr_id, grant_it->second, path.path_revision))
      {
        continue;
      }
      const auto & known = grant_it->second.required_conflict_ids;
      const bool known_at_commit = std::find(
          known.begin(), known.end(), rec.conflict_id) != known.end();
      if (known_at_commit) {
        continue;
      }
      if (rec.source_route_version <=
          grant_it->second.committed_at_route_version &&
        rec.source_occupancy_version <=
          grant_it->second.committed_at_occupancy_version)
      {
        continue;
      }
      state.unresolved_authority[in.amr_id] = rec.conflict_id;
      out.status = kStatusAuthorityConflict;
      out.authority_conflict = true;
      out.authority_conflict_id = rec.conflict_id;
      out.grant_id_hit = grant_it->second.grant_id;
      out.message =
        "new conflict intersects committed grant — retain reservation";
      out.relevant_conflicts.push_back(rec);
      return out;
    }
  }

  state.pending_demand.insert(in.amr_id);
  state.groups.rebuild_from_conflicts(
    state.conflicts, state.route_version, state.commitment_version,
    state.occupancy_version, state.priority_version);

  PassageGroup * my_group = state.groups.group_for_amr(in.amr_id);
  std::vector<ConflictRecord> cluster_records;
  if (my_group) {
    cluster_records = state.conflicts.for_cluster_members(my_group->member_amrs);
  } else {
    cluster_records = state.conflicts.active_for_robot(in.amr_id);
  }
  std::sort(cluster_records.begin(), cluster_records.end(),
    [](const ConflictRecord & a, const ConflictRecord & b) {
      return a.conflict_id < b.conflict_id;
    });
  out.relevant_conflicts = cluster_records;

  // TM grant tips / observed progress are immutable route occurrences.
  // Map them onto the current rolling vector — never clamp a foreign global
  // index onto a different local waypoint.
  int32_t from_idx = 0;
  if (in.distance_origin == "POSE") {
    const auto local = local_index_for_occurrence(path, fact.progress_occurrence);
    from_idx = local ? static_cast<int32_t>(*local) : 0;
  } else if (in.distance_origin == "ROUTE_OCCURRENCE") {
    const auto local = local_index_for_grant_tip(
      path, static_cast<int32_t>(in.observed_progress.route_occurrence));
    if (!local) {
      out.status = kStatusStaleRoute;
      out.message =
        "observed progress occurrence not on cached route (past cache end)";
      return out;
    }
    from_idx = *local;
  } else {
    // GRANT_TIP (default): current_grant_through_index is the committed tip
    // on the original PathIntent numbering.
    if (in.current_grant_through_index < 0) {
      from_idx = 0;
    } else {
      const auto local =
        local_index_for_grant_tip(path, in.current_grant_through_index);
      if (!local) {
        out.status = kStatusStaleRoute;
        out.message =
          "grant tip occurrence past cached route end | cache_wps=" +
          std::to_string(path.waypoints.size()) +
          " grant_through=" + std::to_string(in.current_grant_through_index) +
          " front_occ=" +
          std::to_string(path.waypoints.front().route_occurrence) +
          " back_occ=" +
          std::to_string(path.waypoints.back().route_occurrence);
        return out;
      }
      from_idx = *local;
    }
  }
  from_idx = clamp_index(path, from_idx);

  // Evaluate/path waits and ScheduleCache holds are authoritative: approach
  // only up to the wait point; once there, WAIT with waiting_for until clear.
  std::optional<int32_t> hold_local = first_path_wait_at_or_after(path, 0);
  for (const auto & dep : cache.holds(in.amr_id)) {
    const int32_t at = hold_local_index(path, dep.wait_point);
    if (!hold_local || at < *hold_local) {
      hold_local = at;
    }
  }
  // Also treat conflict wait occurrence as a hold cap when closer than path wait.
  for (const auto & rec : cluster_records) {
    if (rec.physically_cleared) {
      continue;
    }
    const auto * p = rec.participant(in.amr_id);
    if (!p) {
      continue;
    }
    const auto mapped = local_index_for_grant_tip(
      path, static_cast<int32_t>(p->wait.route_occurrence));
    if (!mapped) {
      continue;
    }
    if (!hold_local || *mapped < *hold_local) {
      hold_local = *mapped;
    }
  }
  if (hold_local && from_idx >= *hold_local) {
    // Already at/past wait tip. Without a conflict group there is nothing to
    // schedule against — hold until clearance. With a group, fall through so
    // the scheduler can pick exclusive passage via easiness / priority
    // (mutual WAIT at hold tips previously skipped that path).
    bool still_blocked = !cache.holds(in.amr_id).empty();
    if (!still_blocked && hold_local &&
      static_cast<std::size_t>(*hold_local) < path.waypoints.size() &&
      waypoint_has_wait(path.waypoints[static_cast<std::size_t>(*hold_local)]))
    {
      still_blocked = true;
    }
    if (!still_blocked) {
      for (const auto & rec : cluster_records) {
        if (!rec.physically_cleared && rec.participant(in.amr_id)) {
          still_blocked = true;
          break;
        }
      }
    }
    if (still_blocked && !my_group) {
      return make_wait_at_hold(
        state, path, from_idx, *hold_local, cluster_records, cache, in.amr_id,
        "at_wait_point — hold until conflict clears");
    }
  }

  CompatibilitySnapshot snap;
  std::unordered_set<std::string> members;
  if (my_group) {
    members = my_group->member_amrs;
  } else {
    members.insert(in.amr_id);
  }
  std::vector<PassageAction> all_actions;
  for (const auto & mid : members) {
    const auto mp = cache.get(mid);
    if (!mp) {
      continue;
    }
    snap.paths[mid] = *mp;
    int32_t m_from = 0;
    if (mid == in.amr_id) {
      m_from = from_idx;
    } else {
      const auto cit = state.commitments.find(mid);
      if (cit != state.commitments.end() &&
        cit->second.release_through_index >= 0)
      {
        const auto mapped =
          local_index_for_grant_tip(*mp, cit->second.release_through_index);
        m_from = mapped ? *mapped : static_cast<int32_t>(mp->waypoints.size()) - 1;
      }
    }
    const auto git = state.commitments.find(mid);
    if (git != state.commitments.end() &&
      git->second.release_through_index >= 0)
    {
      const auto mapped =
        local_index_for_grant_tip(*mp, git->second.release_through_index);
      const int32_t tip_local = mapped ?
        *mapped : static_cast<int32_t>(mp->waypoints.size()) - 1;
      snap.reserved_intervals[mid] = {0, tip_local};
    }
    auto & f = state.priorities[mid];
    f.robot_id = mid;
    // Peer look-ahead uses the same horizon as the requester (TM yaml
    // query_horizon_m). Hardcoded 20 m previously built oversize actions.
    const double ext_m = in.desired_extension_m > 0.0 ?
      in.desired_extension_m : 10.0;
    const int32_t m_to = index_for_extension(*mp, m_from, ext_m);
    f.physically_feasible =
      !foreign_base_blocks_range(cache, mid, *mp, m_from, m_to);

    PassageActionBuildInput bin;
    bin.amr_id = mid;
    bin.path = &(*mp);
    bin.from_index = m_from;
    bin.desired_extension_m = ext_m;
    bin.task_id = !mp->dispatch_task_id.empty() ?
      mp->dispatch_task_id : mp->task_id;
    bin.execution_order_id = mid == in.amr_id ? in.execution_order_id : "";
    bin.relevant = state.conflicts.for_cluster_members(
      my_group ? my_group->member_amrs : std::unordered_set<std::string>{mid});
    bin.exit_blocked = foreign_base_blocks_range(
      cache, mid, *mp, m_from, m_to);
    auto built = build_passage_actions(bin);
    all_actions.insert(all_actions.end(), built.begin(), built.end());
  }

  PassageDecision decision;
  ScheduleResult sched;
  if (my_group) {
    my_group->actions = all_actions;
    ScheduleRequest sreq;
    sreq.group = my_group;
    sreq.requester_amr = in.amr_id;
    sreq.snapshot = snap;
    sreq.facts = state.priorities;
    sreq.budget.max_nodes = state.scheduler_max_nodes;
    sreq.max_bypass_count = state.max_bypass_count;
    for (const auto & mid : my_group->member_amrs) {
      sreq.easiness_by_amr[mid] = cluster_easiness_for(
        cluster_records, my_group->member_amrs, mid);
    }
    sched = state.scheduler.schedule(sreq);
    out.group_id = my_group->group_id;
    out.group_structure_version = my_group->group_structure_version;
    out.route_geometry_version = my_group->route_geometry_version;
    out.authority_version = my_group->authority_version;
    out.occupancy_clearance_version = my_group->occupancy_clearance_version;
    out.policy_priority_version = my_group->policy_priority_version;

    for (const auto & a : sched.eligible_actions) {
      decision.go_robots.push_back(a.amr_id);
    }
    for (const auto & m : my_group->member_amrs) {
      if (std::find(decision.go_robots.begin(), decision.go_robots.end(), m) ==
        decision.go_robots.end())
      {
        decision.wait_robots.push_back(m);
      }
    }
    decision.reason = sched.debug_summary;
    decision.debug_summary = sched.debug_summary;
    decision.unresolved_opposite = sched.hard_hold;
    decision.recovery_required = sched.hard_hold &&
      sched.eligible_actions.empty();
  } else {
    sched.eligible_actions = all_actions;
    decision.go_robots = {in.amr_id};
    decision.reason = "no_cluster";
  }
  out.decision = decision;

  const PassageAction * chosen = nullptr;
  for (const auto & a : sched.eligible_actions) {
    if (a.amr_id == in.amr_id && a.safe_hold_verified) {
      chosen = &a;
      break;
    }
  }

  if (!chosen) {
    // Tip already at / past schedule end → not a clearance wait (no blockers).
    // Usually PathIntent vs ScheduleCache length/index mismatch; TM must reseed.
    const int32_t target = index_for_extension(
      path, from_idx, in.desired_extension_m);
    if (target <= from_idx) {
      const int32_t cache_last =
        path.waypoints.empty() ?
        -1 : static_cast<int32_t>(path.waypoints.size()) - 1;
      const double tip_x = path.waypoints.empty() ?
        0.0 : path.waypoints[static_cast<std::size_t>(from_idx)].x;
      const double tip_y = path.waypoints.empty() ?
        0.0 : path.waypoints[static_cast<std::size_t>(from_idx)].y;
      out.status = kStatusStaleRoute;
      out.message =
        "no schedule beyond grant tip — reseed PathIntent into ScheduleCache"
        " | cache_wps=" + std::to_string(path.waypoints.size()) +
        " cache_last=" + std::to_string(cache_last) +
        " from_idx=" + std::to_string(from_idx) +
        " target=" + std::to_string(target) +
        " grant_through=" + std::to_string(in.current_grant_through_index) +
        " desired_m=" + std::to_string(in.desired_extension_m) +
        " origin=" + in.distance_origin +
        " path_rev=" + std::to_string(path.path_revision) +
        " tip_xy=(" + std::to_string(tip_x) + "," + std::to_string(tip_y) + ")" +
        " actions=" + std::to_string(all_actions.size()) +
        " eligible=" + std::to_string(sched.eligible_actions.size());
      out.decision_reason = out.message;
      const uint32_t tip_occ = occurrence_at_local(
        path, static_cast<std::size_t>(std::max(0, from_idx)));
      out.release_through_index = static_cast<int32_t>(tip_occ);
      out.from_occurrence = make_occurrence(
        path.path_revision, tip_occ, tip_x, tip_y);
      out.to_occurrence = out.from_occurrence;
      state.groups.set_pending_extension(in.amr_id, true);
      return out;
    }
    out.status = kStatusWait;
    out.wait_reason_code = sched.wait_reason_code.empty() ?
      kWaitClearance : sched.wait_reason_code;
    out.wait_reason = out.wait_reason_code;
    out.decision_reason = sched.debug_summary;
    out.message = sched.debug_summary;
    state.groups.set_pending_extension(in.amr_id, true);
    // Always publish waiters + wait_point so TM can PathRelease WAIT (empty
    // waiting_for previously dropped the hold and left robots tip-frozen).
    fill_waiting_for(
      out, path, from_idx, cluster_records, cache, in.amr_id);
    if (out.waiting_for.empty()) {
      for (const auto & rec : cluster_records) {
        if (rec.physically_cleared || !rec.participant(in.amr_id)) {
          continue;
        }
        out.waiting_for.push_back(wait_dep_from_record(rec, in.amr_id));
      }
    }
    if (decision.recovery_required) {
      out.status = kStatusRecoveryRequired;
    }
    return out;
  }

  // Cap GO tip at evaluate/conflict wait — never FREE past the hold.
  PassageAction capped = *chosen;
  if (hold_local && capped.to_index > *hold_local && from_idx < *hold_local) {
    capped.to_index = *hold_local;
    capped.to = make_occurrence(
      path.path_revision,
      occurrence_at_local(path, static_cast<std::size_t>(*hold_local)),
      path.waypoints[static_cast<std::size_t>(*hold_local)].x,
      path.waypoints[static_cast<std::size_t>(*hold_local)].y);
    capped.is_approach = true;
    capped.action_id =
      "approach:" + in.amr_id + ":" + std::to_string(capped.from_index) +
      ":" + std::to_string(capped.to_index);
    capped.safe_hold_verified = true;
    chosen = &capped;
  }

  {
    const auto owners = foreign_base_owners_blocking(
      cache, in.amr_id, path, chosen->from_index, chosen->to_index);
    if (!owners.empty()) {
      out = make_wait_at_hold(
        state, path, from_idx, hold_local ? *hold_local : from_idx,
        cluster_records, cache, in.amr_id,
        "proposed range hits foreign reserved base");
      out.wait_reason = "ownership";
      // Ownership must name the base owner — empty waiting_for made TM skip
      // PathRelease (tip-freeze) and cleared wait markers.
      if (out.waiting_for.empty()) {
        const int32_t hold_i = clamp_index(
          path, hold_local ? *hold_local : from_idx);
        for (const auto & owner : owners) {
          WaitDependency d;
          d.amr_id = owner;
          d.topology = kTopoOwnership;
          if (hold_i >= 0) {
            d.wait_point.x = path.waypoints[static_cast<std::size_t>(hold_i)].x;
            d.wait_point.y = path.waypoints[static_cast<std::size_t>(hold_i)].y;
            d.wait_point.index = path.waypoints[
              static_cast<std::size_t>(hold_i)].route_occurrence;
          }
          out.waiting_for.push_back(std::move(d));
        }
      }
      return out;
    }
  }

  // Same tip as committed grant while a wait/conflict remains → WAIT, not
  // another FREE_PROPOSAL (TM "same tip" loop left robots tip-frozen).
  if (in.current_grant_through_index >= 0 &&
    static_cast<int32_t>(chosen->to.route_occurrence) <=
      in.current_grant_through_index)
  {
    bool blocked = hold_local.has_value();
    if (!blocked) {
      for (const auto & rec : cluster_records) {
        if (!rec.physically_cleared && rec.participant(in.amr_id)) {
          blocked = true;
          break;
        }
      }
    }
    if (blocked) {
      return make_wait_at_hold(
        state, path, from_idx,
        hold_local ? *hold_local : from_idx,
        cluster_records, cache, in.amr_id,
        "at_granted_wait_tip — waiting_for conflict clearance");
    }
  }

  const auto latest = state.latest_proposal_by_amr.find(in.amr_id);
  if (latest != state.latest_proposal_by_amr.end()) {
    const auto pit = state.pending_proposals.find(latest->second);
    if (pit != state.pending_proposals.end()) {
      const bool reusable = pit->second.to_index == chosen->to_index &&
        pit->second.path_revision == path.path_revision;
      if (reusable && proposal_is_stale(state, &cache, pit->second)) {
        // Never re-offer a proposal commit would reject as stale.
        state.pending_proposals.erase(pit);
      } else if (reusable) {
        out.status = kStatusFreeProposal;
        out.proposal_id = pit->second.proposal_id;
        out.decision_id = pit->second.decision_id;
        // Grant tip is immutable route occurrence (matches to_occurrence).
        out.release_through_index =
          static_cast<int32_t>(pit->second.to_occurrence.route_occurrence);
        out.from_occurrence = pit->second.from_occurrence;
        out.to_occurrence = pit->second.to_occurrence;
        out.required_conflict_ids = pit->second.required_conflict_ids;
        out.action_id = pit->second.action_id;
        out.decision_reason = "reuse_pending_txn";
        out.message = "ok";
        return out;
      }
    }
  }

  out.status = kStatusFreeProposal;
  out.proposal_id = state.next_proposal_id++;
  out.decision_id = state.next_decision_id++;
  // Never publish trimmed local to_index — TM round-trips this as grant tip.
  out.release_through_index =
    static_cast<int32_t>(chosen->to.route_occurrence);
  state.latest_proposal_by_amr[in.amr_id] = out.proposal_id;

  PendingProposal pending;
  pending.proposal_id = out.proposal_id;
  pending.decision_id = out.decision_id;
  pending.amr_id = in.amr_id;
  pending.task_id = canonical_task;
  pending.execution_order_id = in.execution_order_id;
  pending.action_id = chosen->action_id;
  pending.group_id = out.group_id;
  pending.path_revision = path.path_revision;
  pending.from_index = chosen->from_index;
  pending.to_index = chosen->to_index;
  pending.from_occurrence = chosen->from;
  pending.to_occurrence = chosen->to;
  pending.route_version = state.route_version;
  pending.occupancy_version = state.occupancy_version;
  pending.commitment_version = state.commitment_version;
  pending.group_structure_version = out.group_structure_version;
  pending.route_geometry_version = out.route_geometry_version;
  pending.authority_version = out.authority_version;
  pending.occupancy_clearance_version = out.occupancy_clearance_version;
  pending.policy_priority_version = out.policy_priority_version;
  pending.required_conflict_ids = chosen->reserved_conflict_ids;
  if (pending.required_conflict_ids.empty()) {
    for (const auto & rec : cluster_records) {
      if (!rec.physically_cleared && rec.participant(in.amr_id)) {
        pending.required_conflict_ids.push_back(rec.conflict_id);
      }
    }
  }
  pending.action = *chosen;
  std::unordered_set<std::string> stamped_amrs{in.amr_id};
  for (const auto & cid : pending.required_conflict_ids) {
    if (const auto r = state.conflicts.get(cid)) {
      pending.conflict_versions[cid] = r->record_version;
      for (const auto & a : r->participant_amrs()) {
        stamped_amrs.insert(a);
      }
    }
  }
  for (const auto & a : stamped_amrs) {
    if (const auto sp = cache.get(a)) {
      pending.route_stamps[a] = make_route_stamp(*sp);
    }
  }
  pending.group_authority_stamp = group_authority_stamp(state, in.amr_id);
  state.pending_proposals[out.proposal_id] = pending;
  state.groups.set_pending_extension(in.amr_id, false);

  out.action_id = pending.action_id;
  out.from_occurrence = pending.from_occurrence;
  out.to_occurrence = pending.to_occurrence;
  out.required_conflict_ids = pending.required_conflict_ids;
  out.proposed_horizon_m =
    path_length_to_index(path, static_cast<std::size_t>(chosen->to_index)) -
    path_length_to_index(path, static_cast<std::size_t>(chosen->from_index));
  out.decision_reason = sched.debug_summary;
  out.message = "FREE_PROPOSAL — TM must validate and commit";
  return out;
}

std::vector<std::string> wake_pending_demand(
  ProposalEngineState & state,
  const std::vector<std::string> & affected_amrs)
{
  std::vector<std::string> woken;
  for (const auto & amr : affected_amrs) {
    state.pending_demand.insert(amr);
    state.groups.set_pending_extension(amr, true);
    if (auto * g = state.groups.group_for_amr(amr)) {
      state.scheduler.invalidate(g->group_id);
      for (const auto & m : g->member_amrs) {
        if (state.pending_demand.count(m) ||
          g->pending_extension_by_amr[m])
        {
          woken.push_back(m);
          state.groups.set_pending_extension(m, true);
        }
      }
    } else {
      woken.push_back(amr);
    }
  }
  std::sort(woken.begin(), woken.end());
  woken.erase(std::unique(woken.begin(), woken.end()), woken.end());
  return woken;
}

std::string apply_commit(
  ProposalEngineState & state,
  const uint64_t proposal_id,
  const uint64_t grant_id,
  const std::string & amr_id,
  const std::string & status,
  const int32_t release_through_index,
  const RouteOccurrence & from_occ,
  const RouteOccurrence & to_occ,
  const uint32_t path_revision,
  const std::string & task_id,
  ScheduleCache * cache)
{
  const auto existing = state.grants_by_id.find(grant_id);
  if (existing != state.grants_by_id.end()) {
    if (existing->second.amr_id == amr_id &&
      existing->second.proposal_id == proposal_id &&
      existing->second.release_through_index == release_through_index &&
      existing->second.path_revision == path_revision &&
      status == kGrantCommitted)
    {
      return "DUPLICATE";
    }
    if (status == kGrantCommitted) {
      return "REJECTED_MISMATCH";
    }
  }

  if (status == kGrantCommitted) {
    const auto pit = state.pending_proposals.find(proposal_id);
    if (pit == state.pending_proposals.end()) {
      const auto latest = state.latest_proposal_by_amr.find(amr_id);
      if (latest != state.latest_proposal_by_amr.end() &&
        proposal_id < latest->second)
      {
        return "REJECTED_STALE";
      }
      return "REJECTED_MISMATCH";
    }
    const auto & p = pit->second;
    if (p.amr_id != amr_id ||
      (path_revision != 0 && p.path_revision != path_revision) ||
      (!task_id.empty() && !p.task_id.empty() && p.task_id != task_id))
    {
      return "REJECTED_MISMATCH";
    }
    // Exact proposed range or a formal subset — never expand.
    // release_through_index is route_occurrence (same space as from/to occ).
    if (release_through_index <
        static_cast<int32_t>(p.from_occurrence.route_occurrence) ||
      release_through_index >
        static_cast<int32_t>(p.to_occurrence.route_occurrence))
    {
      return "REJECTED_MISMATCH";
    }
    // Scoped staleness (conflicts / routes / authority this proposal used).
    if (proposal_is_stale(state, cache, p)) {
      return "REJECTED_STALE";
    }
    if (to_occ.path_revision != 0 &&
      to_occ.path_revision != p.path_revision)
    {
      return "REJECTED_MISMATCH";
    }
    if (to_occ.route_occurrence !=
      static_cast<uint32_t>(release_through_index))
    {
      return "REJECTED_MISMATCH";
    }

    CommittedGrantView g;
    g.amr_id = amr_id;
    g.grant_id = grant_id;
    g.proposal_id = proposal_id;
    g.release_through_index = release_through_index;
    g.from_occurrence = from_occ;
    if (from_occ.path_revision == 0 && from_occ.route_occurrence == 0 &&
      from_occ.x == 0.0 && from_occ.y == 0.0)
    {
      g.from_occurrence = p.from_occurrence;
    }
    g.to_occurrence = to_occ;
    g.path_revision = path_revision ? path_revision : p.path_revision;
    // Prefer pending canonical task_id; request must already have matched above.
    g.task_id = p.task_id.empty() ? task_id : p.task_id;
    g.execution_order_id = p.execution_order_id;
    g.action_id = p.action_id;
    g.required_conflict_ids = p.required_conflict_ids;
    g.commitment_version = ++state.commitment_version;
    const PassageAction winner_action = p.action;  // p dies with the erase below
    g.committed_at_route_version = state.route_version;
    g.committed_at_occupancy_version = state.occupancy_version;
    state.commitments[amr_id] = g;
    state.grants_by_id[grant_id] = g;
    state.active_go.insert(amr_id);
    state.pending_proposals.erase(proposal_id);
    state.unresolved_authority.erase(amr_id);
    state.pending_demand.erase(amr_id);

    LedgerGrant lg;
    lg.amr_id = g.amr_id;
    lg.grant_id = g.grant_id;
    lg.proposal_id = g.proposal_id;
    lg.release_through_index = g.release_through_index;
    lg.from_occurrence = g.from_occurrence;
    lg.to_occurrence = g.to_occurrence;
    lg.path_revision = g.path_revision;
    lg.task_id = g.task_id;
    lg.execution_order_id = g.execution_order_id;
    lg.action_id = g.action_id;
    lg.required_conflict_ids = g.required_conflict_ids;
    lg.commitment_version = g.commitment_version;
    state.ledger.put_grant(lg);

    if (auto * grp = state.groups.group_for_amr(amr_id)) {
      // Fairness: only members that are waiting AND whose own action is
      // incompatible with the winner's lost an exclusive passage.
      CompatibilitySnapshot snap;
      if (cache) {
        for (const auto & m : grp->member_amrs) {
          if (const auto mp = cache->get(m)) {
            snap.paths[m] = *mp;
          }
        }
      }
      std::vector<std::string> others;
      std::vector<std::string> losers;
      for (const auto & m : grp->member_amrs) {
        if (m == amr_id) {
          continue;
        }
        others.push_back(m);
        const auto pe = grp->pending_extension_by_amr.find(m);
        if (!state.pending_demand.count(m) &&
          (pe == grp->pending_extension_by_amr.end() || !pe->second))
        {
          continue;
        }
        for (const auto & a : grp->actions) {
          if (a.amr_id == m && !actions_compatible(winner_action, a, snap)) {
            losers.push_back(m);
            break;
          }
        }
      }
      if (losers.empty()) {
        state.groups.on_passage_committed_reset_bypass(grp->group_id, amr_id);
      } else {
        state.groups.on_exclusive_passage_committed(grp->group_id, amr_id, losers);
      }
      grp->referenced_grant_ids.push_back(grant_id);
      wake_pending_demand(state, others);
    }

    if (cache) {
      GrantedBase base;
      base.task_id = g.task_id;
      base.x = to_occ.x;
      base.y = to_occ.y;
      cache->set_granted_base(amr_id, base);
    }
    return "ACCEPTED";
  }

  if (status == kGrantCompleted) {
    return retire_grant(state, amr_id, grant_id, cache) ?
      "ACCEPTED" : "REJECTED_MISMATCH";
  }

  if (status == kGrantRejected || status == kGrantStale ||
    status == kGrantSuperseded)
  {
    state.pending_proposals.erase(proposal_id);
    return "ACCEPTED";
  }
  return "REJECTED_MISMATCH";
}

bool apply_clearance(
  ProposalEngineState & state,
  const std::string & reporter_amr,
  const std::string & conflict_id,
  const bool physically_cleared)
{
  if (!physically_cleared || conflict_id.empty() || reporter_amr.empty()) {
    return false;
  }
  const auto rec = state.conflicts.get(conflict_id);
  if (!rec) {
    return false;
  }
  // Already cleared — idempotent, do not bump occupancy version.
  if (rec->physically_cleared) {
    return true;
  }
  // Only the clearing obligor (blocker) may clear; waiters cannot.
  if (!rec->blocker_amr.empty()) {
    if (reporter_amr != rec->blocker_amr) {
      return false;
    }
  } else if (!rec->participant(reporter_amr)) {
    return false;
  }
  const bool cleared = state.conflicts.mark_physically_cleared(conflict_id);
  if (!cleared) {
    return false;
  }
  ++state.occupancy_version;
  {
    std::vector<std::string> affected = rec->participant_amrs();
    wake_pending_demand(state, affected);
    if (auto * g = state.groups.group_for_amr(reporter_amr)) {
      state.groups.bump_occupancy_version(g->group_id);
    }
  }
  for (auto it = state.unresolved_authority.begin();
    it != state.unresolved_authority.end(); )
  {
    if (it->second == conflict_id) {
      it = state.unresolved_authority.erase(it);
    } else {
      ++it;
    }
  }
  return true;
}

bool retire_grant(
  ProposalEngineState & state,
  const std::string & amr_id,
  const uint64_t grant_id,
  ScheduleCache * cache)
{
  const auto git = state.grants_by_id.find(grant_id);
  if (git == state.grants_by_id.end() || git->second.amr_id != amr_id) {
    return false;
  }
  const auto cit = state.commitments.find(amr_id);
  const bool is_current =
    cit != state.commitments.end() && cit->second.grant_id == grant_id;
  state.grants_by_id.erase(git);
  if (!is_current) {
    // Older grant retirement must not clear newer authority.
    return true;
  }
  state.commitments.erase(cit);
  state.active_go.erase(amr_id);
  state.unresolved_authority.erase(amr_id);
  state.ledger.retire_grant(amr_id, grant_id);
  ++state.commitment_version;
  wake_pending_demand(state, {amr_id});
  if (cache) {
    cache->clear_granted_base(amr_id);
  }
  return true;
}

bool clear_authority_conflict(
  ProposalEngineState & state,
  const std::string & amr_id)
{
  return state.unresolved_authority.erase(amr_id) > 0;
}

}  // namespace ecbs_planner
