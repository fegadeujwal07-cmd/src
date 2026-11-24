#include "ecbs_planner/group_passage_scheduler.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <unordered_set>
#include <utility>

namespace ecbs_planner
{
namespace
{

bool is_b2b_topology(const std::string & topo)
{
  // Same-direction shared passage only — not crossings / plus / opposite.
  return topo.find("b2b") != std::string::npos ||
         topo.find("B2B") != std::string::npos ||
         topo.find("same_way") != std::string::npos ||
         topo.find("back_to_back") != std::string::npos ||
         topo.find("behind") != std::string::npos;
}

/// Fraction of [from,to] already covered by progress (own-route occurrence).
double passage_progress_frac(
  const PassageAction & a, uint32_t progress_occ)
{
  const uint32_t lo = a.from.route_occurrence;
  const uint32_t hi = a.to.route_occurrence;
  if (hi <= lo) {
    return progress_occ >= hi ? 1.0 : 0.0;
  }
  if (progress_occ <= lo) {
    return 0.0;
  }
  if (progress_occ >= hi) {
    return 1.0;
  }
  return static_cast<double>(progress_occ - lo) /
         static_cast<double>(hi - lo);
}

uint32_t progress_for(
  const PassageAction & a, const ScheduleRequest & req)
{
  const auto it = req.facts.find(a.amr_id);
  if (it != req.facts.end() && it->second.progress_valid) {
    return it->second.progress_occurrence;
  }
  return a.from.route_occurrence;
}

}  // namespace

bool GroupPassageScheduler::prefer_action(
  const PassageAction & a,
  const PassageAction & b,
  const ScheduleRequest & req) const
{
  // Rule order among feasible: bypass → easiness → priority → age → id
  // (matches arbitrate_passage; easiness from conflict_store participant flags).
  const int ba = a.bypass_count;
  const int bb = b.bypass_count;
  const bool a_promoted = ba >= req.max_bypass_count;
  const bool b_promoted = bb >= req.max_bypass_count;
  if (a_promoted != b_promoted) {
    return a_promoted;
  }
  const auto ea = req.easiness_by_amr.find(a.amr_id);
  const auto eb = req.easiness_by_amr.find(b.amr_id);
  const int sa = ea == req.easiness_by_amr.end() ? 0 : ea->second;
  const int sb = eb == req.easiness_by_amr.end() ? 0 : eb->second;
  if (sa != sb) {
    return sa > sb;
  }
  const auto fa = req.facts.find(a.amr_id);
  const auto fb = req.facts.find(b.amr_id);
  const int pa = fa == req.facts.end() ?
    0 : task_priority_rank(fa->second.task_priority);
  const int pb = fb == req.facts.end() ?
    0 : task_priority_rank(fb->second.task_priority);
  if (pa != pb) {
    return pa > pb;
  }
  if (std::fabs(a.waiting_age_sec - b.waiting_age_sec) > 1e-6) {
    return a.waiting_age_sec > b.waiting_age_sec;
  }
  if (a.amr_id != b.amr_id) {
    return a.amr_id < b.amr_id;
  }
  return a.action_id < b.action_id;
}

ScheduleResult GroupPassageScheduler::schedule(ScheduleRequest & req)
{
  ScheduleResult out;
  if (!req.group) {
    out.wait_reason_code = kWaitHardHold;
    out.debug_summary = "no_group";
    out.hard_hold = true;
    return out;
  }
  PassageGroup & group = *req.group;
  SchedulerSnapshotKey key;
  key.group_structure_version = group.group_structure_version;
  key.route_geometry_version = group.route_geometry_version;
  key.authority_version = group.authority_version;
  key.occupancy_clearance_version = group.occupancy_clearance_version;
  key.policy_priority_version = group.policy_priority_version;

  auto & frontier = frontiers_[group.group_id];
  if (!(frontier.key == key)) {
    frontier = SearchFrontier{};
    frontier.key = key;
  }
  return run_search(req, frontier);
}

ScheduleResult GroupPassageScheduler::run_search(
  ScheduleRequest & req, SearchFrontier & frontier)
{
  ScheduleResult out;
  PassageGroup & group = *req.group;

  // Annotate fairness onto actions.
  for (auto & a : group.actions) {
    const auto it = group.fairness.find(a.amr_id);
    if (it != group.fairness.end()) {
      a.bypass_count = it->second.bypass_count;
      a.waiting_age_sec = it->second.waiting_age_sec;
    }
    const auto fit = req.facts.find(a.amr_id);
    if (fit != req.facts.end()) {
      a.task_priority = fit->second.task_priority;
      if (a.waiting_age_sec <= 0.0) {
        a.waiting_age_sec = fit->second.waiting_age_sec;
      }
    }
  }

  // Compulsory extras: B2B front→rear. (Held actions are handled by the
  // held-blocker rule at candidate selection, not as graph edges.)
  std::vector<DependencyEdge> compulsory;
  // B2B only for verified same-direction shared passages. Front robot =
  // further along its own passage interval (progress), never raw cross-robot
  // from_index comparison (those index spaces are unrelated).
  for (std::size_t i = 0; i < group.actions.size(); ++i) {
    for (std::size_t j = i + 1; j < group.actions.size(); ++j) {
      const auto & a = group.actions[i];
      const auto & b = group.actions[j];
      if (a.amr_id == b.amr_id) {
        continue;
      }
      std::string shared_b2b;
      for (const auto & c : a.reserved_conflict_ids) {
        const auto topo = group.conflict_topology.find(c);
        if (topo == group.conflict_topology.end() || !is_b2b_topology(topo->second)) {
          continue;
        }
        if (std::find(
            b.reserved_conflict_ids.begin(),
            b.reserved_conflict_ids.end(), c) !=
          b.reserved_conflict_ids.end())
        {
          shared_b2b = c;
          break;
        }
      }
      if (shared_b2b.empty()) {
        continue;
      }
      const double fa = passage_progress_frac(a, progress_for(a, req));
      const double fb = passage_progress_frac(b, progress_for(b, req));
      const bool a_front = fa > fb + 1e-9 ?
        true : (fb > fa + 1e-9 ? false : prefer_action(a, b, req));
      DependencyEdge e;
      e.from_action_id = a_front ? a.action_id : b.action_id;
      e.to_action_id = a_front ? b.action_id : a.action_id;
      e.clearance_condition = "b2b_front_clears";
      e.compulsory = true;
      e.reason = "b2b";
      compulsory.push_back(e);
    }
  }

  auto mandatory = build_mandatory_graph(group.actions, compulsory);
  const auto forced = mandatory.forced_cycles();
  if (!forced.empty()) {
    out.hard_hold = true;
    out.wait_reason_code = kWaitHardHold;
    out.debug_summary = "forced_cycle";
    for (const auto & comp : forced) {
      out.forced_cycle_action_ids.insert(
        out.forced_cycle_action_ids.end(), comp.begin(), comp.end());
    }
    // Unrelated actions outside forced cycle may still be eligible.
  }

  std::unordered_set<std::string> forced_set(
    out.forced_cycle_action_ids.begin(), out.forced_cycle_action_ids.end());

  // Collect undecided incompatible pairs for PBS branching.
  std::vector<std::pair<std::string, std::string>> pairs;
  for (std::size_t i = 0; i < group.actions.size(); ++i) {
    for (std::size_t j = i + 1; j < group.actions.size(); ++j) {
      const auto & a = group.actions[i];
      const auto & b = group.actions[j];
      if (a.amr_id == b.amr_id) {
        continue;
      }
      if (forced_set.count(a.action_id) || forced_set.count(b.action_id)) {
        continue;
      }
      if (a.exec_state == PassageActionState::Held ||
        b.exec_state == PassageActionState::Held)
      {
        continue;
      }
      if (!actions_compatible(a, b, req.snapshot)) {
        pairs.emplace_back(a.action_id, b.action_id);
      }
    }
  }
  std::sort(pairs.begin(), pairs.end());

  const auto t0 = std::chrono::steady_clock::now();
  int nodes = frontier.nodes_expanded;
  std::vector<DependencyEdge> best_choices = frontier.cached_valid_order;
  bool found = frontier.has_verified_incumbent;

  // Resume: try continuing from cached order first.
  auto try_choices = [&](const std::vector<DependencyEdge> & choices) -> bool {
      auto cand = build_candidate_graph(mandatory, choices);
      if (!cand.scheduling_cycles().empty()) {
        return false;
      }
      // Verify each selected first-mover has safe hold.
      for (const auto & e : choices) {
        const auto it = std::find_if(
          group.actions.begin(), group.actions.end(),
          [&](const PassageAction & a) {
            return a.action_id == e.from_action_id;
          });
        if (it == group.actions.end() || !it->safe_hold_verified) {
          return false;
        }
      }
      return true;
    };

  if (!found && !best_choices.empty() && try_choices(best_choices)) {
    found = true;
  }

  // Branch on undecided pairs (stable order).
  std::function<void(std::size_t, std::vector<DependencyEdge> &)> dfs;
  dfs = [&](std::size_t pi, std::vector<DependencyEdge> & cur) {
      if (found && frontier.search_complete) {
        return;
      }
      ++nodes;
      if (nodes > req.budget.max_nodes) {
        return;
      }
      if (req.budget.use_wall_clock) {
        const auto ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - t0).count();
        if (ms > req.budget.max_wall_ms) {
          return;
        }
      }
      if (pi >= pairs.size()) {
        if (try_choices(cur)) {
          best_choices = cur;
          found = true;
          frontier.search_complete = true;
        }
        return;
      }
      const auto & pr = pairs[pi];
      auto find_act = [&](const std::string & id) -> const PassageAction * {
          for (const auto & a : group.actions) {
            if (a.action_id == id) {
              return &a;
            }
          }
          return nullptr;
        };
      const PassageAction * fa = find_act(pr.first);
      const PassageAction * fb = find_act(pr.second);
      // Prefer_action chooses which robot should go first; try that branch
      // before the reverse so priority/easiness actually win the search.
      const bool first_preferred =
        fa && fb && prefer_action(*fa, *fb, req);
      const std::pair<std::string, std::string> order[2] = {
        first_preferred ?
          std::make_pair(pr.first, pr.second) :
          std::make_pair(pr.second, pr.first),
        first_preferred ?
          std::make_pair(pr.second, pr.first) :
          std::make_pair(pr.first, pr.second),
      };
      for (const auto & ord : order) {
        DependencyEdge e;
        e.from_action_id = ord.first;
        e.to_action_id = ord.second;
        e.from_scheduling_choice = true;
        e.clearance_condition = "passage_order";
        e.reason = "pbs_choice";
        cur.push_back(e);
        if (try_choices(cur) || pi + 1 < pairs.size()) {
          auto cand = build_candidate_graph(mandatory, cur);
          if (cand.scheduling_cycles().empty()) {
            dfs(pi + 1, cur);
          }
        }
        cur.pop_back();
        if (found && frontier.search_complete) {
          return;
        }
      }
    };

  if (!frontier.search_complete && nodes <= req.budget.max_nodes) {
    std::vector<DependencyEdge> cur = frontier.cached_valid_order;
    // shortcut: if no undecided pairs, incumbent is all safe eligible actions
    if (pairs.empty()) {
      found = true;
      frontier.search_complete = true;
      best_choices.clear();
    } else {
      dfs(0, cur);
    }
  }

  frontier.nodes_expanded = nodes;
  frontier.cached_valid_order = best_choices;
  frontier.has_verified_incumbent = found;

  if (!found) {
    out.search_in_progress = !frontier.search_complete;
    out.wait_reason_code = out.hard_hold ? kWaitHardHold : kWaitSearch;
    out.debug_summary = out.hard_hold ? "hard_hold" : "search_budget_resume";
    // Still emit approach-only actions that conflict with nothing.
    for (const auto & a : group.actions) {
      if (a.is_approach && a.safe_hold_verified &&
        a.exec_state == PassageActionState::Eligible &&
        !forced_set.count(a.action_id))
      {
        bool ok = true;
        for (const auto & o : group.actions) {
          if (o.action_id == a.action_id) {
            continue;
          }
          if (!actions_compatible(a, o, req.snapshot)) {
            ok = false;
            break;
          }
        }
        if (ok) {
          out.eligible_actions.push_back(a);
        }
      }
    }
    if (!out.eligible_actions.empty() &&
      std::any_of(
        out.eligible_actions.begin(), out.eligible_actions.end(),
        [&](const PassageAction & a) {
          return a.amr_id == req.requester_amr;
        }))
    {
      out.wait_reason_code.clear();
    }
    return out;
  }

  // Build eligible set: actions with no unmet predecessor in chosen order.
  std::unordered_set<std::string> must_wait;
  for (const auto & e : mandatory.edges()) {
    if (!e.external_blocker) {
      must_wait.insert(e.to_action_id);
    }
  }
  for (const auto & e : best_choices) {
    must_wait.insert(e.to_action_id);
  }
  // Held-blocker rule: a Held / unverified action B (hold_reason in
  // {exit_occupied, unsafe_or_held, unsafe_hold}) whose robot has reached its
  // held interval (progress >= B.from; unknown progress fails closed) is a
  // physical obstacle. Any action A of another robot with
  // !actions_compatible(A, B) must not be a candidate: A waits (WAIT_HARDHOLD).
  // Held actions on a different, non-overlapping passage are compatible and
  // never block.
  auto held_blocked = [&](const PassageAction & a) {
      for (const auto & b : group.actions) {
        if (b.amr_id == a.amr_id ||
          !(b.exec_state == PassageActionState::Held || !b.safe_hold_verified))
        {
          continue;
        }
        const std::string & why = b.hold_reason.empty() ? "unsafe_or_held" : b.hold_reason;
        if (why != "exit_occupied" && why != "unsafe_or_held" && why != "unsafe_hold") {
          continue;
        }
        if (progress_for(b, req) < b.from.route_occurrence) {
          continue;
        }
        if (!actions_compatible(a, b, req.snapshot)) {
          return true;
        }
      }
      return false;
    };
  // First actions (not waiting) that are compatible concurrently.
  std::vector<PassageAction> candidates;
  bool requester_held_blocked = false;
  for (const auto & a : group.actions) {
    if (forced_set.count(a.action_id)) {
      continue;
    }
    if (!a.safe_hold_verified || a.exec_state == PassageActionState::Held) {
      continue;
    }
    if (held_blocked(a)) {
      requester_held_blocked = requester_held_blocked || a.amr_id == req.requester_amr;
      continue;
    }
    if (must_wait.count(a.action_id)) {
      // Still eligible if all predecessors are Done — not tracked here; treat as wait.
      continue;
    }
    candidates.push_back(a);
  }
  std::sort(candidates.begin(), candidates.end(),
    [&](const PassageAction & x, const PassageAction & y) {
      return prefer_action(x, y, req);
    });

  for (const auto & c : candidates) {
    bool ok = true;
    for (const auto & e : out.eligible_actions) {
      if (!actions_compatible(c, e, req.snapshot)) {
        ok = false;
        break;
      }
    }
    if (ok) {
      out.eligible_actions.push_back(c);
    }
  }

  frontier.incumbent_action_ids.clear();
  for (const auto & a : out.eligible_actions) {
    frontier.incumbent_action_ids.push_back(a.action_id);
  }

  const bool requester_ok = std::any_of(
    out.eligible_actions.begin(), out.eligible_actions.end(),
    [&](const PassageAction & a) {
      return a.amr_id == req.requester_amr;
    });
  if (!requester_ok) {
    if (out.hard_hold && forced_set.size() > 0) {
      bool requester_forced = false;
      for (const auto & a : group.actions) {
        if (a.amr_id == req.requester_amr && forced_set.count(a.action_id)) {
          requester_forced = true;
          break;
        }
      }
      out.wait_reason_code = requester_forced ? kWaitHardHold : kWaitClearance;
    } else {
      out.wait_reason_code = requester_held_blocked ? kWaitHardHold : kWaitClearance;
    }
    out.debug_summary = requester_held_blocked ? "held_blocker" : "requester_not_incumbent";
  } else {
    out.wait_reason_code.clear();
    out.debug_summary = "verified_incumbent";
  }
  group.selected_precedences = best_choices;
  return out;
}

}  // namespace ecbs_planner
