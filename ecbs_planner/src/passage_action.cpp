#include "ecbs_planner/passage_action.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <sstream>

#include "ecbs_planner/path_utils.hpp"

namespace ecbs_planner
{
namespace
{

int32_t clamp_idx(const TimedPath & path, int32_t i)
{
  if (path.waypoints.empty()) {
    return 0;
  }
  return std::max(
    0, std::min(i, static_cast<int32_t>(path.waypoints.size()) - 1));
}

int32_t index_for_extension(
  const TimedPath & path, int32_t from, double desired_m)
{
  from = clamp_idx(path, from);
  if (path.waypoints.empty()) {
    return from;
  }
  double acc = 0.0;
  for (int32_t i = from + 1;
    i < static_cast<int32_t>(path.waypoints.size()); ++i)
  {
    const auto & a = path.waypoints[static_cast<std::size_t>(i - 1)];
    const auto & b = path.waypoints[static_cast<std::size_t>(i)];
    acc += std::hypot(b.x - a.x, b.y - a.y);
    if (acc >= desired_m) {
      return i;
    }
  }
  return static_cast<int32_t>(path.waypoints.size()) - 1;
}

RouteOccurrence occ_at(const TimedPath & path, int32_t idx)
{
  idx = clamp_idx(path, idx);
  const auto & w = path.waypoints[static_cast<std::size_t>(idx)];
  return make_occurrence(
    path.path_revision, w.route_occurrence,
    w.x, w.y, w.level_name, w.index.value_or(-1), 0.0);
}

std::string make_action_id(
  const std::string & amr, int32_t from, int32_t to, bool approach)
{
  std::ostringstream oss;
  oss << (approach ? "approach:" : "pass:") << amr << ":" << from << ":" << to;
  return oss.str();
}

std::optional<int32_t> local_for_occ(const TimedPath & path, uint32_t occ)
{
  if (const auto local = local_index_for_occurrence(path, occ)) {
    return static_cast<int32_t>(*local);
  }
  if (path.waypoints.empty()) {
    return std::nullopt;
  }
  const uint32_t front = path.waypoints.front().route_occurrence;
  const uint32_t back = path.waypoints.back().route_occurrence;
  if (occ < front) {
    return 0;
  }
  if (occ > back) {
    return std::nullopt;
  }
  // Sparse gap: nearest occurrence at or before tip.
  int32_t best = 0;
  for (std::size_t i = 0; i < path.waypoints.size(); ++i) {
    if (path.waypoints[i].route_occurrence <= occ) {
      best = static_cast<int32_t>(i);
    }
  }
  return best;
}

}  // namespace

bool verify_safe_hold(
  const PassageAction & action,
  const TimedPath & path,
  const std::vector<ConflictRecord> & active,
  const bool exit_space_free)
{
  if (path.waypoints.empty() || action.to_index < action.from_index) {
    return false;
  }
  if (!exit_space_free) {
    return false;
  }
  // Stopping at to must not leave the body inside an uncleared exclusive overlap
  // without space past exit.
  for (const auto & cid : action.reserved_conflict_ids) {
    for (const auto & rec : active) {
      if (rec.conflict_id != cid || rec.physically_cleared) {
        continue;
      }
      const auto * p = rec.participant(action.amr_id);
      if (!p) {
        continue;
      }
      const auto exit_l = local_for_occ(path, p->exit.route_occurrence);
      if (!exit_l || action.to_index < *exit_l) {
        // Stop before clearing claimed overlap — not a valid passage hold.
        return false;
      }
    }
  }
  // Stop must not sit inside another active exclusive overlap entry for self.
  for (const auto & rec : active) {
    if (rec.physically_cleared || !rec.participant(action.amr_id)) {
      continue;
    }
    if (std::find(
        action.reserved_conflict_ids.begin(),
        action.reserved_conflict_ids.end(),
        rec.conflict_id) != action.reserved_conflict_ids.end())
    {
      continue;
    }
    const auto * p = rec.participant(action.amr_id);
    if (!p) {
      continue;
    }
    const auto e = local_for_occ(path, p->entry.route_occurrence);
    const auto x = local_for_occ(path, p->exit.route_occurrence);
    if (!e || !x) {
      continue;
    }
    if (action.to_index > *e && action.to_index < *x) {
      return false;
    }
  }
  return true;
}

std::vector<PassageAction> build_passage_actions(const PassageActionBuildInput & in)
{
  std::vector<PassageAction> out;
  if (!in.path || in.path->waypoints.empty()) {
    return out;
  }
  // Stamp 0..n-1 when unstamped so occurrence↔local mapping is defined.
  TimedPath stamped = *in.path;
  ensure_route_occurrences(stamped);
  const auto & path = stamped;
  const int32_t from = clamp_idx(path, in.from_index);
  const int32_t target = index_for_extension(path, from, in.desired_extension_m);

  // Collect overlaps this robot still needs, sorted by entry occurrence.
  struct OverlapHit
  {
    std::string conflict_id;
    int32_t wait_i{0};
    int32_t entry_i{0};
    int32_t exit_i{0};
  };
  std::vector<OverlapHit> hits;
  for (const auto & rec : in.relevant) {
    if (rec.physically_cleared ||
      rec.lifecycle == ConflictLifecycle::Cleared ||
      rec.lifecycle == ConflictLifecycle::Retired)
    {
      continue;
    }
    const auto * p = rec.participant(in.amr_id);
    if (!p) {
      continue;
    }
    // Conflict geometry is immutable route_occurrence — map to local indices
    // before comparing with from/to (trimmed cache vectors).
    const auto wait_l = local_for_occ(path, p->wait.route_occurrence);
    const auto entry_l = local_for_occ(path, p->entry.route_occurrence);
    const auto exit_l = local_for_occ(path, p->exit.route_occurrence);
    if (!entry_l || !exit_l) {
      continue;
    }
    OverlapHit h;
    h.conflict_id = rec.conflict_id;
    h.wait_i = wait_l ? *wait_l : *entry_l;
    h.entry_i = *entry_l;
    h.exit_i = *exit_l;
    if (h.wait_i > h.entry_i) {
      h.wait_i = h.entry_i;
    }
    hits.push_back(h);
  }
  std::sort(hits.begin(), hits.end(),
    [](const OverlapHit & a, const OverlapHit & b) {
      return a.entry_i < b.entry_i;
    });

  // Approach-to-wait: if still before first wait point. Cap by desired
  // extension (query_horizon_m) — uncapped wait_i opened 20–50 m bases and
  // parked peers in mutual WAIT with no rolling tip.
  if (!hits.empty() && from < hits.front().wait_i) {
    const int32_t approach_to = std::min(hits.front().wait_i, target);
    if (approach_to > from) {
      PassageAction approach;
      approach.amr_id = in.amr_id;
      approach.task_id = in.task_id;
      approach.execution_order_id = in.execution_order_id;
      approach.path_revision = path.path_revision;
      approach.from_index = from;
      approach.to_index = approach_to;
      approach.from = occ_at(path, approach.from_index);
      approach.to = occ_at(path, approach.to_index);
      approach.is_approach = true;
      approach.action_id = make_action_id(
        in.amr_id, approach.from_index, approach.to_index, true);
      approach.safe_hold_verified = true;
      approach.exec_state = PassageActionState::Eligible;
      out.push_back(approach);
    }
  }

  // Passage actions: merge consecutive overlaps when no safe mid stop.
  std::size_t i = 0;
  while (i < hits.size()) {
    if (from >= hits[i].exit_i) {
      ++i;
      continue;
    }
    std::size_t j = i;
    std::vector<std::string> claimed = {hits[i].conflict_id};
    int32_t hold_at = hits[i].exit_i;
    // Extend through next overlaps if no safe stop between exit and next entry.
    while (j + 1 < hits.size()) {
      const int32_t gap = hits[j + 1].entry_i - hits[j].exit_i;
      if (gap <= 1) {
        claimed.push_back(hits[j + 1].conflict_id);
        hold_at = hits[j + 1].exit_i;
        ++j;
        continue;
      }
      break;
    }
    // Need space past exit for footprint; if exit blocked, hold before entry.
    int32_t to = hold_at;
    if (in.exit_blocked) {
      to = std::max(from, hits[i].wait_i);
      PassageAction hold;
      hold.amr_id = in.amr_id;
      hold.task_id = in.task_id;
      hold.execution_order_id = in.execution_order_id;
      hold.path_revision = path.path_revision;
      hold.from_index = from;
      hold.to_index = to;
      hold.from = occ_at(path, hold.from_index);
      hold.to = occ_at(path, hold.to_index);
      hold.is_approach = true;
      hold.reserved_conflict_ids = claimed;
      hold.action_id = make_action_id(in.amr_id, hold.from_index, hold.to_index, true);
      hold.hold_reason = "exit_occupied";
      hold.safe_hold_verified = false;
      hold.exec_state = PassageActionState::Held;
      out.push_back(hold);
      break;
    }
    // Cap by desired extension target — never past unsafe boundary already handled.
    to = std::min(to, target);
    if (to <= from) {
      ++i;
      continue;
    }
    PassageAction pass;
    pass.amr_id = in.amr_id;
    pass.task_id = in.task_id;
    pass.execution_order_id = in.execution_order_id;
    pass.path_revision = path.path_revision;
    pass.from_index = std::max(from, hits[i].wait_i);
    pass.to_index = to;
    pass.from = occ_at(path, pass.from_index);
    pass.to = occ_at(path, pass.to_index);
    pass.reserved_conflict_ids = claimed;
    pass.downstream_space_m = in.footprint_length_m;
    pass.action_id = make_action_id(
      in.amr_id, pass.from_index, pass.to_index, false);
    pass.safe_hold_verified = verify_safe_hold(pass, path, in.relevant, true);
    pass.exec_state = pass.safe_hold_verified ?
      PassageActionState::Eligible : PassageActionState::Held;
    if (!pass.safe_hold_verified) {
      pass.hold_reason = "unsafe_hold";
    }
    out.push_back(pass);
    i = j + 1;
  }

  // Free extension only when no uncleared overlap still sits ahead of from.
  // At/before wait with no eligible pass must not invent a FREE past the hold.
  const bool has_forward = std::any_of(
    out.begin(), out.end(),
    [from](const PassageAction & a) { return a.to_index > from; });
  const bool pending_overlap = std::any_of(
    hits.begin(), hits.end(),
    [from](const OverlapHit & h) { return from < h.entry_i; });
  if (!has_forward && !pending_overlap && target > from) {
    PassageAction free_a;
    free_a.amr_id = in.amr_id;
    free_a.task_id = in.task_id;
    free_a.execution_order_id = in.execution_order_id;
    free_a.path_revision = path.path_revision;
    free_a.from_index = from;
    free_a.to_index = target;
    free_a.from = occ_at(path, from);
    free_a.to = occ_at(path, target);
    free_a.is_approach = false;
    free_a.action_id = make_action_id(in.amr_id, from, target, false);
    free_a.safe_hold_verified = true;
    free_a.exec_state = PassageActionState::Eligible;
    out.push_back(free_a);
  }
  return out;
}

}  // namespace ecbs_planner
