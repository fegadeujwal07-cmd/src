#include "ecbs_planner/conflict_group_builder.hpp"

#include <algorithm>
#include <functional>
#include <queue>

namespace ecbs_planner
{

uint64_t PassageGroupRegistry::stable_id(
  const std::unordered_set<std::string> & members)
{
  std::vector<std::string> sorted(members.begin(), members.end());
  std::sort(sorted.begin(), sorted.end());
  std::size_t h = 0;
  for (const auto & m : sorted) {
    h ^= std::hash<std::string>{}(m) + 0x9e3779b9 + (h << 6) + (h >> 2);
  }
  return static_cast<uint64_t>(h);
}

void PassageGroupRegistry::rebuild_from_conflicts(
  const ConflictStore & store,
  const uint64_t route_geometry_version,
  const uint64_t authority_version,
  const uint64_t occupancy_clearance_version,
  const uint64_t policy_priority_version)
{
  // Preserve fairness + pending + grant refs by member set.
  std::unordered_map<std::string, FairnessState> fairness_by_amr;
  std::unordered_map<std::string, bool> pending_by_amr;
  std::unordered_map<std::string, std::vector<uint64_t>> grants_by_amr;
  for (const auto & g : groups_) {
    for (const auto & kv : g.fairness) {
      fairness_by_amr[kv.first] = kv.second;
    }
    for (const auto & kv : g.pending_extension_by_amr) {
      pending_by_amr[kv.first] = kv.second;
    }
    for (const auto & amr : g.member_amrs) {
      grants_by_amr[amr] = g.referenced_grant_ids;
    }
  }

  std::unordered_map<std::string, std::unordered_set<std::string>> undirected;
  std::unordered_map<std::string, std::vector<std::string>> conflicts_by_amr;
  std::unordered_map<std::string, std::string> topology_by_conflict;
  std::unordered_set<std::string> nodes;
  for (const auto & rec : store.all()) {
    if (rec.physically_cleared &&
      rec.lifecycle == ConflictLifecycle::Cleared &&
      !rec.retain_beyond_horizon)
    {
      continue;
    }
    if (rec.lifecycle == ConflictLifecycle::Retired) {
      continue;
    }
    topology_by_conflict[rec.conflict_id] = rec.topology;
    const auto amrs = rec.participant_amrs();
    for (const auto & a : amrs) {
      nodes.insert(a);
      conflicts_by_amr[a].push_back(rec.conflict_id);
    }
    for (std::size_t i = 0; i < amrs.size(); ++i) {
      for (std::size_t j = i + 1; j < amrs.size(); ++j) {
        undirected[amrs[i]].insert(amrs[j]);
        undirected[amrs[j]].insert(amrs[i]);
      }
    }
  }

  std::unordered_set<std::string> visited;
  std::vector<PassageGroup> next;
  for (const auto & seed : nodes) {
    if (visited.count(seed)) {
      continue;
    }
    PassageGroup g;
    std::queue<std::string> q;
    q.push(seed);
    visited.insert(seed);
    while (!q.empty()) {
      const auto u = q.front();
      q.pop();
      g.member_amrs.insert(u);
      for (const auto & v : undirected[u]) {
        if (visited.insert(v).second) {
          q.push(v);
        }
      }
    }
    std::unordered_set<std::string> cids;
    for (const auto & m : g.member_amrs) {
      for (const auto & c : conflicts_by_amr[m]) {
        cids.insert(c);
      }
      if (fairness_by_amr.count(m)) {
        g.fairness[m] = fairness_by_amr[m];
      } else {
        g.fairness[m] = FairnessState{};
      }
      if (pending_by_amr.count(m)) {
        g.pending_extension_by_amr[m] = pending_by_amr[m];
      }
      for (const auto & gid : grants_by_amr[m]) {
        if (std::find(
            g.referenced_grant_ids.begin(),
            g.referenced_grant_ids.end(), gid) ==
          g.referenced_grant_ids.end())
        {
          g.referenced_grant_ids.push_back(gid);
        }
      }
    }
    g.conflict_ids.assign(cids.begin(), cids.end());
    std::sort(g.conflict_ids.begin(), g.conflict_ids.end());
    for (const auto & c : g.conflict_ids) {
      g.conflict_topology[c] = topology_by_conflict[c];
    }

    g.route_geometry_version = route_geometry_version;
    g.authority_version = authority_version;
    g.occupancy_clearance_version = occupancy_clearance_version;
    g.policy_priority_version = policy_priority_version;
    next.push_back(std::move(g));
  }

  // Stable group ids: a component keeps the id of the largest previous group
  // it overlaps (merge → largest absorbed id; split → largest part keeps it),
  // so a stored group_id (PendingProposal, scheduler frontier) survives
  // membership changes. Greedy by previous-group size; ties are deterministic.
  struct Claim
  {
    std::size_t prev;
    std::size_t next;
    std::size_t overlap;
  };
  std::vector<Claim> claims;
  for (std::size_t n = 0; n < next.size(); ++n) {
    for (std::size_t p = 0; p < groups_.size(); ++p) {
      std::size_t overlap = 0;
      for (const auto & m : next[n].member_amrs) {
        overlap += groups_[p].member_amrs.count(m);
      }
      if (overlap > 0) {
        claims.push_back({p, n, overlap});
      }
    }
  }
  std::sort(claims.begin(), claims.end(), [&](const Claim & a, const Claim & b) {
      const auto sa = groups_[a.prev].member_amrs.size();
      const auto sb = groups_[b.prev].member_amrs.size();
      if (sa != sb) {
        return sa > sb;
      }
      if (a.overlap != b.overlap) {
        return a.overlap > b.overlap;
      }
      if (groups_[a.prev].group_id != groups_[b.prev].group_id) {
        return groups_[a.prev].group_id < groups_[b.prev].group_id;
      }
      return stable_id(next[a.next].member_amrs) < stable_id(next[b.next].member_amrs);
    });
  std::vector<int> prev_of(next.size(), -1);
  std::vector<char> prev_used(groups_.size(), 0);
  for (const auto & c : claims) {
    if (prev_of[c.next] < 0 && !prev_used[c.prev]) {
      prev_of[c.next] = static_cast<int>(c.prev);
      prev_used[c.prev] = 1;
    }
  }
  std::unordered_set<uint64_t> used_ids;
  for (std::size_t n = 0; n < next.size(); ++n) {
    if (prev_of[n] >= 0) {
      used_ids.insert(groups_[static_cast<std::size_t>(prev_of[n])].group_id);
    }
  }
  for (std::size_t n = 0; n < next.size(); ++n) {
    auto & g = next[n];
    if (prev_of[n] < 0) {
      g.group_id = stable_id(g.member_amrs);
      while (!used_ids.insert(g.group_id).second) {
        ++g.group_id;  // hash collision with a retained id
      }
      g.group_structure_version = 1;
      continue;
    }
    const auto & prev = groups_[static_cast<std::size_t>(prev_of[n])];
    g.group_id = prev.group_id;
    // Structure version: stable unless membership / conflict set changes.
    g.group_structure_version = prev.group_structure_version;
    if (prev.conflict_ids != g.conflict_ids || prev.member_amrs != g.member_amrs) {
      g.group_structure_version += 1;
    }
    g.actions = prev.actions;
    g.mandatory_deps = prev.mandatory_deps;
    g.selected_precedences = prev.selected_precedences;
  }
  std::sort(next.begin(), next.end(),
    [](const PassageGroup & a, const PassageGroup & b) {
      return a.group_id < b.group_id;
    });
  groups_ = std::move(next);
}

PassageGroup * PassageGroupRegistry::group_for_amr(const std::string & amr_id)
{
  for (auto & g : groups_) {
    if (g.member_amrs.count(amr_id)) {
      return &g;
    }
  }
  return nullptr;
}

const PassageGroup * PassageGroupRegistry::group_for_amr(
  const std::string & amr_id) const
{
  for (const auto & g : groups_) {
    if (g.member_amrs.count(amr_id)) {
      return &g;
    }
  }
  return nullptr;
}

void PassageGroupRegistry::set_pending_extension(
  const std::string & amr_id, const bool pending)
{
  if (auto * g = group_for_amr(amr_id)) {
    g->pending_extension_by_amr[amr_id] = pending;
  }
}

std::vector<std::string> PassageGroupRegistry::amrs_with_pending_extension() const
{
  std::vector<std::string> out;
  for (const auto & g : groups_) {
    for (const auto & kv : g.pending_extension_by_amr) {
      if (kv.second) {
        out.push_back(kv.first);
      }
    }
  }
  std::sort(out.begin(), out.end());
  return out;
}

void PassageGroupRegistry::on_exclusive_passage_committed(
  const uint64_t group_id,
  const std::string & winner_amr,
  const std::vector<std::string> & eligible_losers)
{
  for (auto & g : groups_) {
    if (g.group_id != group_id) {
      continue;
    }
    for (const auto & loser : eligible_losers) {
      if (loser == winner_amr) {
        continue;
      }
      g.fairness[loser].bypass_count += 1;
    }
    g.fairness[winner_amr].bypass_count = 0;
    g.policy_priority_version += 1;
    break;
  }
}

void PassageGroupRegistry::on_passage_committed_reset_bypass(
  const uint64_t group_id, const std::string & amr_id)
{
  for (auto & g : groups_) {
    if (g.group_id != group_id) {
      continue;
    }
    g.fairness[amr_id].bypass_count = 0;
    break;
  }
}

void PassageGroupRegistry::bump_occupancy_version(const uint64_t group_id)
{
  for (auto & g : groups_) {
    if (g.group_id == group_id) {
      g.occupancy_clearance_version += 1;
      break;
    }
  }
}

void PassageGroupRegistry::bump_policy_version(const uint64_t group_id)
{
  for (auto & g : groups_) {
    if (g.group_id == group_id) {
      g.policy_priority_version += 1;
      break;
    }
  }
}

}  // namespace ecbs_planner
