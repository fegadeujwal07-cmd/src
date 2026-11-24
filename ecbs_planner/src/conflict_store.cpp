#include "ecbs_planner/conflict_store.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <queue>
#include <sstream>

#include "ecbs_planner/path_topology.hpp"
#include "ecbs_planner/path_utils.hpp"

namespace ecbs_planner
{
namespace
{

std::string join_key(const std::string & a, const std::string & b)
{
  return a < b ? a + "|" + b : b + "|" + a;
}

double cross2(double ax, double ay, double bx, double by)
{
  return ax * by - ay * bx;
}

bool proper_segment_intersection(
  double ax, double ay, double bx, double by,
  double cx, double cy, double dx, double dy)
{
  const double abx = bx - ax;
  const double aby = by - ay;
  const double cdx = dx - cx;
  const double cdy = dy - cy;
  const double acx = cx - ax;
  const double acy = cy - ay;
  const double denom = cross2(abx, aby, cdx, cdy);
  if (std::fabs(denom) < 1e-12) {
    return false;  // parallel / collinear — distance check handles near-miss
  }
  const double t = cross2(acx, acy, cdx, cdy) / denom;
  const double u = cross2(acx, acy, abx, aby) / denom;
  return t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0;
}

}  // namespace

std::vector<std::string> ConflictRecord::participant_amrs() const
{
  std::vector<std::string> out;
  out.reserve(participants.size());
  for (const auto & p : participants) {
    out.push_back(p.amr_id);
  }
  return out;
}

const ParticipantOverlap * ConflictRecord::participant(const std::string & amr_id) const
{
  for (const auto & p : participants) {
    if (p.amr_id == amr_id) {
      return &p;
    }
  }
  return nullptr;
}

ParticipantOverlap * ConflictRecord::participant(const std::string & amr_id)
{
  for (auto & p : participants) {
    if (p.amr_id == amr_id) {
      return &p;
    }
  }
  return nullptr;
}

double point_to_segment_distance(
  const double px, const double py,
  const double ax, const double ay, const double bx, const double by)
{
  const double abx = bx - ax;
  const double aby = by - ay;
  const double apx = px - ax;
  const double apy = py - ay;
  const double ab2 = abx * abx + aby * aby;
  double t = ab2 > 1e-12 ? (apx * abx + apy * aby) / ab2 : 0.0;
  t = std::max(0.0, std::min(1.0, t));
  return std::hypot(px - (ax + t * abx), py - (ay + t * aby));
}

bool segments_conflict(
  const double ax, const double ay, const double bx, const double by,
  const double cx, const double cy, const double dx, const double dy,
  const double eps_m)
{
  if (proper_segment_intersection(ax, ay, bx, by, cx, cy, dx, dy)) {
    return true;
  }
  return point_to_segment_distance(ax, ay, cx, cy, dx, dy) <= eps_m ||
         point_to_segment_distance(bx, by, cx, cy, dx, dy) <= eps_m ||
         point_to_segment_distance(cx, cy, ax, ay, bx, by) <= eps_m ||
         point_to_segment_distance(dx, dy, ax, ay, bx, by) <= eps_m;
}

std::string make_overlap_id(
  const std::string & amr_a,
  const std::string & amr_b,
  const uint32_t rev_a,
  const uint32_t occ_a,
  const uint32_t rev_b,
  const uint32_t occ_b)
{
  const bool swap = amr_b < amr_a;
  const auto & a = swap ? amr_b : amr_a;
  const auto & b = swap ? amr_a : amr_b;
  const uint32_t ra = swap ? rev_b : rev_a;
  const uint32_t oa = swap ? occ_b : occ_a;
  const uint32_t rb = swap ? rev_a : rev_b;
  const uint32_t ob = swap ? occ_a : occ_b;
  std::ostringstream oss;
  oss << "o:" << a << "|" << b
      << ":r" << ra << ":o" << oa
      << ":r" << rb << ":o" << ob;
  return oss.str();
}

std::string make_conflict_id(
  const std::string & waiter,
  const std::string & blocker,
  const uint32_t waiter_rev,
  const uint32_t waiter_occ,
  const uint32_t blocker_rev,
  const uint32_t blocker_occ)
{
  return make_overlap_id(
    waiter, blocker, waiter_rev, waiter_occ, blocker_rev, blocker_occ);
}

bool ConflictStore::content_equal(const ConflictRecord & a, const ConflictRecord & b)
{
  if (a.waiter_amr != b.waiter_amr || a.blocker_amr != b.blocker_amr ||
    a.mandatory != b.mandatory || a.topology != b.topology ||
    a.wait_reason != b.wait_reason || a.tentative != b.tentative ||
    a.physically_cleared != b.physically_cleared ||
    a.clearance_condition != b.clearance_condition ||
    a.participants.size() != b.participants.size())
  {
    return false;
  }
  for (std::size_t i = 0; i < a.participants.size(); ++i) {
    const auto & pa = a.participants[i];
    const auto * pb = b.participant(pa.amr_id);
    if (!pb ||
      !pa.entry.same_slot(pb->entry) || !pa.exit.same_slot(pb->exit) ||
      !pa.wait.same_slot(pb->wait) ||
      pa.blocks_other != pb->blocks_other ||
      pa.blocked_by_other != pb->blocked_by_other ||
      pa.task_id != pb->task_id ||
      pa.path_revision != pb->path_revision)
    {
      return false;
    }
  }
  return true;
}

uint64_t ConflictStore::stable_cluster_id(
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

bool ConflictStore::upsert(const ConflictRecord & record)
{
  if (record.conflict_id.empty()) {
    return false;
  }
  const auto it = by_id_.find(record.conflict_id);
  ConflictRecord stored = record;
  if (it != by_id_.end()) {
    if (it->second.physically_cleared) {
      stored.physically_cleared = true;
      stored.lifecycle = ConflictLifecycle::Cleared;
    }
    if (it->second.lifecycle == ConflictLifecycle::Retired) {
      stored.lifecycle = ConflictLifecycle::Retired;
    } else if (stored.lifecycle == ConflictLifecycle::Active &&
      it->second.lifecycle == ConflictLifecycle::Held)
    {
      stored.lifecycle = ConflictLifecycle::Held;
    }
    stored.group_id = stored.group_id ? stored.group_id : it->second.group_id;
    stored.retain_beyond_horizon = it->second.retain_beyond_horizon ||
      stored.retain_beyond_horizon;
    if (!stored.mandatory && it->second.mandatory) {
      stored.mandatory = true;
      stored.waiter_amr = it->second.waiter_amr;
      stored.blocker_amr = it->second.blocker_amr;
    }
    if (content_equal(it->second, stored) &&
      it->second.lifecycle == stored.lifecycle)
    {
      return false;
    }
    stored.record_version = it->second.record_version + 1;
  } else {
    stored.record_version = 1;
    if (stored.lifecycle == ConflictLifecycle::Active && stored.physically_cleared) {
      stored.lifecycle = ConflictLifecycle::Cleared;
    }
  }
  by_id_[record.conflict_id] = std::move(stored);
  ++store_version_;
  return true;
}

bool ConflictStore::mark_physically_cleared(const std::string & conflict_id)
{
  const auto it = by_id_.find(conflict_id);
  if (it == by_id_.end() || it->second.physically_cleared) {
    return false;
  }
  it->second.physically_cleared = true;
  it->second.mandatory = false;
  it->second.lifecycle = ConflictLifecycle::Cleared;
  ++it->second.record_version;
  ++store_version_;
  return true;
}

bool ConflictStore::erase(const std::string & conflict_id)
{
  if (by_id_.erase(conflict_id) == 0) {
    return false;
  }
  ++store_version_;
  return true;
}

void ConflictStore::erase_participant(const std::string & amr_id)
{
  for (auto it = by_id_.begin(); it != by_id_.end(); ) {
    if (it->second.participant(amr_id) ||
      it->second.waiter_amr == amr_id || it->second.blocker_amr == amr_id)
    {
      it = by_id_.erase(it);
      ++store_version_;
    } else {
      ++it;
    }
  }
}

bool ConflictStore::set_mandatory_dependency(
  const std::string & conflict_id,
  const std::string & waiter,
  const std::string & blocker,
  const bool mandatory)
{
  const auto it = by_id_.find(conflict_id);
  if (it == by_id_.end() || it->second.physically_cleared) {
    return false;
  }
  if (it->second.mandatory == mandatory &&
    it->second.waiter_amr == waiter &&
    it->second.blocker_amr == blocker)
  {
    return false;
  }
  it->second.mandatory = mandatory;
  it->second.waiter_amr = mandatory ? waiter : "";
  it->second.blocker_amr = mandatory ? blocker : "";
  ++it->second.record_version;
  ++store_version_;
  return true;
}

std::optional<ConflictRecord> ConflictStore::get(const std::string & conflict_id) const
{
  const auto it = by_id_.find(conflict_id);
  if (it == by_id_.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::vector<ConflictRecord> ConflictStore::all() const
{
  std::vector<ConflictRecord> out;
  for (const auto & kv : by_id_) {
    out.push_back(kv.second);
  }
  std::sort(out.begin(), out.end(),
    [](const ConflictRecord & a, const ConflictRecord & b) {
      return a.conflict_id < b.conflict_id;
    });
  return out;
}

std::vector<ConflictRecord> ConflictStore::involving(const std::string & amr_id) const
{
  std::vector<ConflictRecord> out;
  for (const auto & kv : by_id_) {
    if (kv.second.participant(amr_id) ||
      kv.second.waiter_amr == amr_id || kv.second.blocker_amr == amr_id)
    {
      out.push_back(kv.second);
    }
  }
  return out;
}

std::vector<ConflictRecord> ConflictStore::for_cluster_members(
  const std::unordered_set<std::string> & members) const
{
  std::vector<ConflictRecord> out;
  for (const auto & kv : by_id_) {
    const auto & r = kv.second;
    if (r.physically_cleared) {
      continue;
    }
    for (const auto & p : r.participants) {
      if (members.count(p.amr_id)) {
        out.push_back(r);
        break;
      }
    }
  }
  return out;
}

std::vector<DirectedDependency> ConflictStore::outbound_deps(
  const std::string & waiter) const
{
  std::vector<DirectedDependency> out;
  for (const auto & kv : by_id_) {
    const auto & r = kv.second;
    if (!r.mandatory || r.physically_cleared || r.waiter_amr != waiter) {
      continue;
    }
    DirectedDependency d;
    d.waiter = r.waiter_amr;
    d.blocker = r.blocker_amr;
    d.conflict_id = r.conflict_id;
    d.reason = r.wait_reason.empty() ? r.topology : r.wait_reason;
    d.source_version = r.record_version;
    out.push_back(d);
  }
  return out;
}

std::vector<ConflictRecord> ConflictStore::active_for_robot(
  const std::string & amr_id) const
{
  std::vector<ConflictRecord> out;
  for (const auto & r : involving(amr_id)) {
    if (!r.physically_cleared) {
      out.push_back(r);
    }
  }
  return out;
}

std::vector<ConflictCluster> ConflictStore::build_clusters() const
{
  std::unordered_map<std::string, std::unordered_set<std::string>> undirected;
  std::unordered_map<std::string, std::vector<DirectedDependency>> out_edges;
  std::unordered_set<std::string> nodes;

  for (const auto & kv : by_id_) {
    const auto & r = kv.second;
    if (r.physically_cleared) {
      continue;
    }
    const auto amrs = r.participant_amrs();
    for (const auto & p : amrs) {
      nodes.insert(p);
    }
    for (std::size_t i = 0; i < amrs.size(); ++i) {
      for (std::size_t j = i + 1; j < amrs.size(); ++j) {
        undirected[amrs[i]].insert(amrs[j]);
        undirected[amrs[j]].insert(amrs[i]);
      }
    }
    if (r.mandatory && !r.waiter_amr.empty() && !r.blocker_amr.empty()) {
      DirectedDependency d;
      d.waiter = r.waiter_amr;
      d.blocker = r.blocker_amr;
      d.conflict_id = r.conflict_id;
      d.reason = r.wait_reason.empty() ? r.topology : r.wait_reason;
      d.source_version = r.record_version;
      out_edges[r.waiter_amr].push_back(d);
      undirected[r.waiter_amr].insert(r.blocker_amr);
      undirected[r.blocker_amr].insert(r.waiter_amr);
      nodes.insert(r.waiter_amr);
      nodes.insert(r.blocker_amr);
    }
  }

  std::unordered_set<std::string> visited;
  std::vector<ConflictCluster> clusters;
  for (const auto & seed : nodes) {
    if (visited.count(seed)) {
      continue;
    }
    ConflictCluster c;
    std::queue<std::string> q;
    q.push(seed);
    visited.insert(seed);
    while (!q.empty()) {
      const auto u = q.front();
      q.pop();
      c.members.insert(u);
      for (const auto & v : undirected[u]) {
        if (visited.insert(v).second) {
          q.push(v);
        }
      }
    }
    c.id = stable_cluster_id(c.members);
    for (const auto & m : c.members) {
      for (const auto & e : out_edges[m]) {
        if (c.members.count(e.blocker)) {
          c.edges.push_back(e);
        }
      }
    }
    std::unordered_map<std::string, int> indegree;
    std::unordered_map<std::string, std::unordered_set<std::string>> adj;
    for (const auto & m : c.members) {
      indegree[m] = 0;
    }
    for (const auto & e : c.edges) {
      if (adj[e.blocker].insert(e.waiter).second) {
        indegree[e.waiter] += 1;
      }
    }
    std::vector<std::string> queue;
    for (const auto & kv : indegree) {
      if (kv.second == 0) {
        queue.push_back(kv.first);
      }
    }
    std::sort(queue.begin(), queue.end());
    std::size_t seen = 0;
    for (std::size_t i = 0; i < queue.size(); ++i) {
      c.proposed_precedence.push_back(queue[i]);
      ++seen;
      std::vector<std::string> nxt(adj[queue[i]].begin(), adj[queue[i]].end());
      std::sort(nxt.begin(), nxt.end());
      for (const auto & v : nxt) {
        if (--indegree[v] == 0) {
          queue.push_back(v);
        }
      }
    }
    c.has_cycle = seen < c.members.size();
    if (c.has_cycle) {
      std::vector<std::string> rest;
      for (const auto & m : c.members) {
        if (std::find(c.proposed_precedence.begin(), c.proposed_precedence.end(), m) ==
          c.proposed_precedence.end())
        {
          rest.push_back(m);
        }
      }
      std::sort(rest.begin(), rest.end());
      c.proposed_precedence.insert(
        c.proposed_precedence.end(), rest.begin(), rest.end());
    }
    c.version = store_version_;
    clusters.push_back(std::move(c));
  }
  std::sort(clusters.begin(), clusters.end(),
    [](const ConflictCluster & a, const ConflictCluster & b) {
      return a.id < b.id;
    });
  return clusters;
}

void ConflictStore::clear()
{
  by_id_.clear();
  ++store_version_;
}

RegisterMatch match_register_wait(
  const ConflictStore & store,
  const std::string & waiter,
  const std::string & blocker,
  const uint32_t wait_occurrence)
{
  RegisterMatch out;
  const ConflictRecord * best = nullptr;
  uint32_t best_entry = 0;
  const auto records = store.involving(waiter);
  for (const auto & rec : records) {
    const auto * wp = rec.participant(waiter);
    if (!wp || !rec.participant(blocker) || rec.physically_cleared ||
      rec.lifecycle == ConflictLifecycle::Retired)
    {
      continue;
    }
    const uint32_t entry = wp->entry.route_occurrence;
    if (wait_occurrence >= entry && wait_occurrence <= wp->exit.route_occurrence) {
      out.error = "wait at occurrence " + std::to_string(wait_occurrence) +
        " is inside overlap " + rec.conflict_id + " (entry " +
        std::to_string(entry) + ")";
      return out;
    }
    if (wait_occurrence < entry && (!best || entry < best_entry)) {
      best = &rec;
      best_entry = entry;
    }
  }
  if (best) {
    out.conflict_id = best->conflict_id;
  } else {
    out.error = "no active conflict " + waiter + "<->" + blocker +
      " has an entry after wait occurrence " + std::to_string(wait_occurrence);
  }
  return out;
}

std::vector<ConflictRecord> discover_route_overlaps(
  const TimedPath & self_in,
  const std::vector<TimedPath> & others_in,
  const double detect_distance_m,
  const double position_epsilon_m,
  const double safety_distance_m,
  const uint64_t route_version,
  const uint64_t occupancy_version)
{
  std::vector<ConflictRecord> out;
  TimedPath self = self_in;
  ensure_route_occurrences(self);
  std::vector<TimedPath> others = others_in;
  for (auto & o : others) {
    ensure_route_occurrences(o);
  }
  if (self.amr_id.empty() || self.waypoints.empty()) {
    return out;
  }
  // Detection radius = clearance; epsilon only for same-cell when index missing.
  const double detect = detect_distance_m;
  const double cell_eps = position_epsilon_m;

  auto level_ok = [](const TimedPoint & a, const TimedPoint & b) {
      return a.level_name.empty() || b.level_name.empty() ||
             a.level_name == b.level_name;
    };

  // Same nav-graph index, or (index missing) XY within position_epsilon_m.
  auto same_graph_cell =
    [cell_eps, &level_ok](const TimedPoint & a, const TimedPoint & b) {
      if (a.index.has_value() && b.index.has_value()) {
        return *a.index == *b.index;
      }
      if (!level_ok(a, b)) {
        return false;
      }
      return std::hypot(a.x - b.x, a.y - b.y) <= cell_eps;
    };

  for (const auto & other : others) {
    if (other.amr_id.empty() || other.amr_id == self.amr_id ||
      other.waypoints.empty())
    {
      continue;
    }

    std::vector<char> near(self.waypoints.size(), 0);
    std::vector<int> other_at(self.waypoints.size(), -1);

    // Point-point and point-to-segment (covers idle midway on long segments).
    for (std::size_t i = 0; i < self.waypoints.size(); ++i) {
      for (std::size_t j = 0; j < other.waypoints.size(); ++j) {
        if (!level_ok(self.waypoints[i], other.waypoints[j])) {
          continue;
        }
        if (same_graph_cell(self.waypoints[i], other.waypoints[j]) ||
          std::hypot(
            self.waypoints[i].x - other.waypoints[j].x,
            self.waypoints[i].y - other.waypoints[j].y) <= detect)
        {
          near[i] = 1;
          other_at[i] = static_cast<int>(j);
        }
      }
      if (other.waypoints.size() >= 2) {
        for (std::size_t j = 1; j < other.waypoints.size(); ++j) {
          if (!level_ok(self.waypoints[i], other.waypoints[j])) {
            continue;
          }
          if (point_to_segment_distance(
              self.waypoints[i].x, self.waypoints[i].y,
              other.waypoints[j - 1].x, other.waypoints[j - 1].y,
              other.waypoints[j].x, other.waypoints[j].y) <= detect)
          {
            near[i] = 1;
            if (other_at[i] < 0) {
              other_at[i] = static_cast<int>(j);
            }
          }
        }
      }
    }
    // Other points vs self segments (idle self / long self edges).
    if (self.waypoints.size() >= 2) {
      for (std::size_t j = 0; j < other.waypoints.size(); ++j) {
        for (std::size_t i = 1; i < self.waypoints.size(); ++i) {
          if (!level_ok(self.waypoints[i], other.waypoints[j])) {
            continue;
          }
          if (point_to_segment_distance(
              other.waypoints[j].x, other.waypoints[j].y,
              self.waypoints[i - 1].x, self.waypoints[i - 1].y,
              self.waypoints[i].x, self.waypoints[i].y) <= detect)
          {
            near[i] = 1;
            near[i - 1] = 1;
            if (other_at[i] < 0) {
              other_at[i] = static_cast<int>(j);
            }
          }
        }
      }
    }
    // Segment–segment intersection / proximity.
    if (self.waypoints.size() >= 2 && other.waypoints.size() >= 2) {
      for (std::size_t i = 1; i < self.waypoints.size(); ++i) {
        for (std::size_t j = 1; j < other.waypoints.size(); ++j) {
          if (!level_ok(self.waypoints[i], other.waypoints[j])) {
            continue;
          }
          if (segments_conflict(
              self.waypoints[i - 1].x, self.waypoints[i - 1].y,
              self.waypoints[i].x, self.waypoints[i].y,
              other.waypoints[j - 1].x, other.waypoints[j - 1].y,
              other.waypoints[j].x, other.waypoints[j].y, detect))
          {
            near[i] = 1;
            near[i - 1] = 1;
            if (other_at[i] < 0) {
              other_at[i] = static_cast<int>(j);
            }
            if (other_at[i - 1] < 0) {
              other_at[i - 1] = static_cast<int>(j - 1);
            }
          }
        }
      }
    }

    std::size_t i = 0;
    while (i < near.size()) {
      while (i < near.size() && !near[i]) {
        ++i;
      }
      if (i >= near.size()) {
        break;
      }
      const std::size_t lo = i;
      while (i < near.size() && near[i]) {
        ++i;
      }
      const std::size_t hi = i - 1;
      int other_lo = other_at[lo];
      int other_hi = other_at[hi];
      for (std::size_t k = lo; k <= hi; ++k) {
        if (other_at[k] >= 0) {
          other_lo = other_lo < 0 ? other_at[k] : std::min(other_lo, other_at[k]);
          other_hi = other_hi < 0 ? other_at[k] : std::max(other_hi, other_at[k]);
        }
      }
      if (other_lo < 0) {
        other_lo = 0;
      }
      if (other_hi < 0) {
        other_hi = static_cast<int>(other.waypoints.size() - 1);
      }

      PathTopology topo = PathTopology::Unknown;
      if (self.waypoints.size() >= 2 && other.waypoints.size() >= 2) {
        topo = classify_path_overlap(
          self, lo, hi, other,
          static_cast<std::size_t>(other_lo),
          static_cast<std::size_t>(other_hi), detect);
      }
      bool self_blocks = false;
      bool other_blocks = false;
      if (self.waypoints.size() >= 2 && other.waypoints.size() >= 2) {
        path_easiness_flags(
          self.waypoints, other.waypoints, detect, self_blocks, other_blocks);
      }
      const std::size_t wait_i = safety_wait_index(self, lo, 0, safety_distance_m);
      const std::size_t other_wait_i = safety_wait_index(
        other, static_cast<std::size_t>(other_lo), 0, safety_distance_m);

      ParticipantOverlap self_p;
      self_p.amr_id = self.amr_id;
      self_p.task_id = self.task_id;
      self_p.phase_index = static_cast<int32_t>(self.phase_index);
      self_p.path_revision = self.path_revision;
      self_p.entry = make_occurrence(
        self.path_revision, self.waypoints[lo].route_occurrence,
        self.waypoints[lo].x, self.waypoints[lo].y,
        self.waypoints[lo].level_name, self.waypoints[lo].index.value_or(-1));
      self_p.exit = make_occurrence(
        self.path_revision, self.waypoints[hi].route_occurrence,
        self.waypoints[hi].x, self.waypoints[hi].y,
        self.waypoints[hi].level_name, self.waypoints[hi].index.value_or(-1));
      self_p.wait = make_occurrence(
        self.path_revision, self.waypoints[wait_i].route_occurrence,
        self.waypoints[wait_i].x, self.waypoints[wait_i].y,
        self.waypoints[wait_i].level_name,
        self.waypoints[wait_i].index.value_or(-1));
      self_p.blocks_other = self_blocks;
      self_p.blocked_by_other = other_blocks;

      ParticipantOverlap other_p;
      other_p.amr_id = other.amr_id;
      other_p.task_id = other.task_id;
      other_p.phase_index = static_cast<int32_t>(other.phase_index);
      other_p.path_revision = other.path_revision;
      other_p.entry = make_occurrence(
        other.path_revision, other.waypoints[other_lo].route_occurrence,
        other.waypoints[other_lo].x, other.waypoints[other_lo].y,
        other.waypoints[other_lo].level_name,
        other.waypoints[other_lo].index.value_or(-1));
      other_p.exit = make_occurrence(
        other.path_revision, other.waypoints[other_hi].route_occurrence,
        other.waypoints[other_hi].x, other.waypoints[other_hi].y,
        other.waypoints[other_hi].level_name,
        other.waypoints[other_hi].index.value_or(-1));
      other_p.wait = make_occurrence(
        other.path_revision, other.waypoints[other_wait_i].route_occurrence,
        other.waypoints[other_wait_i].x, other.waypoints[other_wait_i].y,
        other.waypoints[other_wait_i].level_name,
        other.waypoints[other_wait_i].index.value_or(-1));
      other_p.blocks_other = other_blocks;
      other_p.blocked_by_other = self_blocks;

      ConflictRecord rec;
      rec.participants = {self_p, other_p};
      rec.fleet_name = self.fleet_name;
      rec.topology = path_topology_wait_reason(topo);
      rec.wait_reason = path_topology_wait_reason(topo);
      rec.mandatory = false;
      rec.clearance_condition =
        "blocker_clears_overlap:" + join_key(self.amr_id, other.amr_id);
      rec.source_route_version = route_version;
      rec.source_occupancy_version = occupancy_version;
      // Stable stamps, not vector indices (those shift when the cache erases
      // the path behind the robot).
      rec.conflict_id = make_overlap_id(
        self.amr_id, other.amr_id,
        self.path_revision, self.waypoints[lo].route_occurrence,
        other.path_revision,
        other.waypoints[static_cast<std::size_t>(other_lo)].route_occurrence);
      out.push_back(std::move(rec));
    }
  }
  return out;
}

}  // namespace ecbs_planner
