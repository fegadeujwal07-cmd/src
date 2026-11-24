#include "ecbs_planner/passage_arbitration.hpp"

#include <algorithm>
#include <sstream>

#include "ecbs_planner/wait_reason.hpp"

namespace ecbs_planner
{
namespace
{

bool is_opposite_record(const ConflictRecord & r)
{
  return r.wait_reason == kWaitReasonOpposite ||
         r.topology == kWaitReasonOpposite ||
         r.topology == kTopoOpposite;
}

bool is_b2b_record(const ConflictRecord & r)
{
  return r.wait_reason == kWaitReasonBackToBack ||
         r.topology == kTopoSameWay ||
         r.topology == kWaitReasonBackToBack;
}

/// Easiness for \p id on one record — flags are stored per robot identity.
int easiness_for_participant(const ConflictRecord & r, const std::string & id)
{
  const auto * p = r.participant(id);
  if (!p) {
    return 0;
  }
  return easiness_score(p->blocks_other, p->blocked_by_other);
}

/// Robots locked by unresolved opposite pairs (pair-local; unrelated easiness
/// on other overlaps cannot cancel these holds).
std::unordered_set<std::string> hard_held_by_opposite(
  const std::vector<ConflictRecord> & records,
  const std::unordered_set<std::string> & members)
{
  std::unordered_set<std::string> held;
  for (const auto & r : records) {
    if (r.physically_cleared || !is_opposite_record(r)) {
      continue;
    }
    std::vector<std::string> in_pair;
    for (const auto & id : r.participant_amrs()) {
      if (members.count(id)) {
        in_pair.push_back(id);
      }
    }
    if (in_pair.size() < 2) {
      continue;
    }
    // Decisive Case B on this pair → not a hard opposite for these two.
    bool pair_has_easiness = false;
    for (const auto & id : in_pair) {
      if (easiness_for_participant(r, id) != 0) {
        pair_has_easiness = true;
        break;
      }
    }
    if (pair_has_easiness) {
      continue;
    }
    for (const auto & id : in_pair) {
      held.insert(id);
    }
  }
  return held;
}

}  // namespace

int cluster_easiness_for(
  const std::vector<ConflictRecord> & records,
  const std::unordered_set<std::string> & members,
  const std::string & id)
{
  bool go = false;
  bool yield = false;
  for (const auto & r : records) {
    if (r.physically_cleared || !r.participant(id)) {
      continue;
    }
    bool peer_in_cluster = false;
    for (const auto & peer : r.participant_amrs()) {
      if (peer != id && members.count(peer)) {
        peer_in_cluster = true;
        break;
      }
    }
    if (!peer_in_cluster) {
      continue;
    }
    const int s = easiness_for_participant(r, id);
    go = go || s > 0;
    yield = yield || s < 0;
  }
  return go == yield ? 0 : (go ? 1 : -1);
}

namespace
{

/// Longitudinal metres from overlap entry toward exit (shared directed geometry).
double progress_metres_into_overlap(
  const ConflictRecord & r,
  const std::string & id,
  const RobotPriorityFacts & fact)
{
  const auto * p = r.participant(id);
  if (!p) {
    return -1.0;
  }
  const double ex = p->exit.x - p->entry.x;
  const double ey = p->exit.y - p->entry.y;
  const double len2 = ex * ex + ey * ey;
  if (len2 < 1e-9) {
    return -1.0;  // cannot establish front/rear on degenerate geometry
  }
  double px = p->entry.x;
  double py = p->entry.y;
  if (fact.progress_xy_valid) {
    px = fact.progress_x;
    py = fact.progress_y;
  } else if (fact.progress_valid) {
    // Approximate along entry→exit using occurrence fraction when XY missing.
    const uint32_t entry = p->entry.route_occurrence;
    const uint32_t exit = p->exit.route_occurrence;
    if (exit > entry) {
      const double t = std::min(
        1.0,
        std::max(
          0.0,
          static_cast<double>(fact.progress_occurrence - entry) /
            static_cast<double>(exit - entry)));
      return t * std::sqrt(len2);
    }
    return -1.0;
  } else {
    return -1.0;
  }
  const double t = std::max(
    0.0, std::min(1.0, ((px - p->entry.x) * ex + (py - p->entry.y) * ey) / len2));
  return t * std::sqrt(len2);
}

/// Shared-segment B2B: compare metres along the same directed overlap.
std::string b2b_front_on_record(
  const ConflictRecord & r,
  const std::vector<std::string> & eligible,
  const std::unordered_map<std::string, RobotPriorityFacts> & facts)
{
  std::string front;
  double best_m = -1.0;
  bool found = false;
  bool ambiguous = false;
  for (const auto & id : eligible) {
    if (!r.participant(id)) {
      continue;
    }
    const auto it = facts.find(id);
    if (it == facts.end()) {
      continue;
    }
    const double into = progress_metres_into_overlap(r, id, it->second);
    if (into < 0.0) {
      ambiguous = true;
      continue;
    }
    if (!found || into > best_m + 1e-3) {
      best_m = into;
      front = id;
      found = true;
    } else if (std::fabs(into - best_m) <= 1e-3) {
      ambiguous = true;
    }
  }
  if (!found || ambiguous) {
    return "";  // retain hold when front/rear cannot be established
  }
  return front;
}

int priority_for(
  const std::string & id,
  const std::unordered_map<std::string, RobotPriorityFacts> & facts,
  const std::unordered_map<std::string, std::string> & current_task_by_robot)
{
  const auto it = facts.find(id);
  if (it == facts.end()) {
    return 0;
  }
  const auto task_it = current_task_by_robot.find(id);
  if (task_it != current_task_by_robot.end()) {
    if (!it->second.task_id.empty() &&
      !task_it->second.empty() &&
      it->second.task_id != task_it->second)
    {
      return 0;
    }
  }
  return task_priority_rank(it->second.task_priority);
}

}  // namespace

int b2b_front_rear_from_progress(
  const uint32_t a_progress_occ,
  const uint32_t b_progress_occ)
{
  if (a_progress_occ == b_progress_occ) {
    return 0;
  }
  return a_progress_occ > b_progress_occ ? 1 : -1;
}

PassageDecision arbitrate_passage(
  const ConflictCluster & cluster,
  const std::vector<ConflictRecord> & records,
  const std::unordered_map<std::string, RobotPriorityFacts> & facts,
  const std::unordered_set<std::string> & already_granted_go,
  const std::unordered_map<std::string, std::string> & current_task_by_robot)
{
  PassageDecision out;
  if (cluster.members.empty()) {
    out.reason = "empty_cluster";
    out.preference_kind = "no_preference";
    out.debug_summary = "empty_cluster";
    return out;
  }

  std::vector<std::string> members(cluster.members.begin(), cluster.members.end());
  std::sort(members.begin(), members.end());

  std::unordered_set<std::string> go_set;
  for (const auto & id : already_granted_go) {
    if (cluster.members.count(id)) {
      go_set.insert(id);
    }
  }

  // Per-pair opposite HardHold before candidate selection.
  const auto opposite_held = hard_held_by_opposite(records, cluster.members);
  if (!opposite_held.empty() && go_set.empty()) {
    out.unresolved_opposite = true;
    out.reason = kStatusUnresolvedOpposite;
    out.preference_kind = "opposite_hold";
    out.debug_summary = "unresolved_opposite HardHold per-pair";
    if (cluster.has_cycle) {
      out.recovery_required = true;
    }
  }

  // A. Feasibility + mandatory blockers; opposite-held robots are not eligible.
  std::vector<std::string> eligible;
  for (const auto & id : members) {
    if (opposite_held.count(id) && go_set.count(id) == 0) {
      out.wait_robots.push_back(id);
      continue;
    }
    const auto it = facts.find(id);
    if (it != facts.end()) {
      if (!it->second.healthy || !it->second.available ||
        !it->second.physically_feasible)
      {
        out.wait_robots.push_back(id);
        continue;
      }
    }
    bool blocked = false;
    for (const auto & r : records) {
      if (!r.mandatory || r.physically_cleared || r.waiter_amr != id) {
        continue;
      }
      if (already_granted_go.count(r.blocker_amr) == 0) {
        blocked = true;
        break;
      }
    }
    if (blocked && already_granted_go.count(id) == 0) {
      out.wait_robots.push_back(id);
      continue;
    }
    eligible.push_back(id);
  }

  // If only opposite-held remain, stop — no easiness elsewhere may advance them.
  if (eligible.empty()) {
    for (const auto & id : members) {
      if (std::find(out.wait_robots.begin(), out.wait_robots.end(), id) ==
        out.wait_robots.end())
      {
        out.wait_robots.push_back(id);
      }
    }
    if (out.unresolved_opposite) {
      out.go_robots.clear();
      return out;
    }
    out.preference_kind = "insufficient_information";
    out.reason = "hold_no_feasible_winner";
    out.debug_summary = out.reason;
    return out;
  }

  // C. B2B: deterministic order over B2B records, then progress into overlap.
  if (go_set.empty()) {
    std::vector<ConflictRecord> b2b_recs;
    for (const auto & r : records) {
      if (!r.physically_cleared && is_b2b_record(r)) {
        b2b_recs.push_back(r);
      }
    }
    std::sort(b2b_recs.begin(), b2b_recs.end(),
      [](const ConflictRecord & a, const ConflictRecord & b) {
        return a.conflict_id < b.conflict_id;
      });
    for (const auto & r : b2b_recs) {
      const std::string front = b2b_front_on_record(r, eligible, facts);
      if (front.empty() || opposite_held.count(front)) {
        continue;
      }
      go_set.insert(front);
      out.preference_kind = "b2b";
      out.reason = "b2b_front_from_overlap_progress";
      for (const auto & p : r.participant_amrs()) {
        if (p != front && cluster.members.count(p)) {
          out.mandatory_assignments.push_back({r.conflict_id, {p, front}});
        }
      }
      break;
    }
  }

  // D. Case B / easiness (never for opposite-held robots — already excluded).
  if (go_set.empty()) {
    int best = 0;
    int best_count = 0;
    std::string winner;
    for (const auto & id : eligible) {
      const int s = cluster_easiness_for(records, cluster.members, id);
      if (s > best) {
        best = s;
        best_count = 1;
        winner = id;
      } else if (s == best && best > 0) {
        ++best_count;
      }
    }
    if (best > 0 && best_count == 1) {
      go_set.insert(winner);
      out.preference_kind = "easiness";
      out.reason = "easiness_go";
    } else if (best > 0 && best_count > 1) {
      out.preference_kind = "contradictory";
      out.reason = "easiness_tie";
    } else {
      // E. Task priority among remaining eligible (task-matched).
      int best_pri = -1;
      int pri_count = 0;
      std::string pri_winner;
      for (const auto & id : eligible) {
        const int pr = priority_for(id, facts, current_task_by_robot);
        if (pr > best_pri) {
          best_pri = pr;
          pri_count = 1;
          pri_winner = id;
        } else if (pr == best_pri) {
          ++pri_count;
        }
      }
      if (best_pri > 0 && pri_count == 1) {
        go_set.insert(pri_winner);
        out.preference_kind = "priority";
        out.reason = "task_priority_after_easiness";
      } else if (!eligible.empty() && opposite_held.empty()) {
        // Bounded fairness: prefer longest waiting among feasible candidates.
        std::string fair = eligible.front();
        double best_age = -1.0;
        for (const auto & id : eligible) {
          const auto it = facts.find(id);
          const double age = it == facts.end() ? 0.0 : it->second.waiting_age_sec;
          if (age > best_age + 1e-6 ||
            (std::fabs(age - best_age) <= 1e-6 && id < fair))
          {
            best_age = age;
            fair = id;
          }
        }
        go_set.insert(fair);
        out.preference_kind = best_age > 0.0 ? "waiting_age" : "stable_tie_break";
        out.reason = best_pri <= 0 ?
          "no_easiness_no_priority_fair" : "priority_tie_fair";
      } else {
        out.preference_kind = "insufficient_information";
        out.reason = "hold_no_feasible_winner";
      }
    }
  }

  if (cluster.has_cycle && go_set.empty() && !out.unresolved_opposite) {
    out.recovery_required = true;
    out.reason = "circular_wait";
    out.preference_kind = "insufficient_information";
  }

  // Mandatory waits: non-GO participants wait for each GO on shared overlaps.
  // Never assign a GO that is opposite-held.
  if (!go_set.empty()) {
    for (const auto & r : records) {
      if (r.physically_cleared || r.participants.size() < 2) {
        continue;
      }
      std::string go_in;
      std::string wait_in;
      for (const auto & p : r.participant_amrs()) {
        if (go_set.count(p) && opposite_held.count(p) == 0) {
          go_in = p;
        } else if (cluster.members.count(p) && go_set.count(p) == 0) {
          wait_in = p;
        }
      }
      if (!go_in.empty() && !wait_in.empty()) {
        out.mandatory_assignments.push_back({r.conflict_id, {wait_in, go_in}});
      }
    }
  }

  for (const auto & id : members) {
    if (go_set.count(id) && opposite_held.count(id) == 0) {
      out.go_robots.push_back(id);
    } else if (std::find(out.wait_robots.begin(), out.wait_robots.end(), id) ==
      out.wait_robots.end())
    {
      out.wait_robots.push_back(id);
    }
  }

  std::ostringstream dbg;
  dbg << "cluster=" << cluster.id << " go=";
  for (const auto & g : out.go_robots) {
    dbg << g << ",";
  }
  dbg << " wait=";
  for (const auto & w : out.wait_robots) {
    dbg << w << ",";
  }
  dbg << " kind=" << out.preference_kind << " reason=" << out.reason;
  out.debug_summary = dbg.str();
  return out;
}

}  // namespace ecbs_planner
