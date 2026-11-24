#include "ecbs_planner/global_resolver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <set>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

#include "ecbs_planner/wait_reason.hpp"

namespace ecbs_planner
{
namespace
{

constexpr double kEps = 1e-6;
constexpr double kInf = std::numeric_limits<double>::infinity();

std::string f2(double v)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.2f", v);
  return buf;
}

std::string xy(double x, double y)
{
  return "(" + f2(x) + "," + f2(y) + ")";
}

std::string hex(std::size_t h)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%08llx", static_cast<unsigned long long>(h & 0xffffffffULL));
  return buf;
}

double sq(double v) {return v * v;}

const char * topology_for(OverlapKind k)
{
  switch (k) {
    case OverlapKind::Opposite: return kTopoOpposite;
    case OverlapKind::SameDirection: return kTopoSameWay;
    case OverlapKind::Crossing: return kTopoCross;
  }
  return kTopoOwnership;
}

}  // namespace

const char * overlap_kind_name(OverlapKind k)
{
  switch (k) {
    case OverlapKind::Opposite: return "opposite";
    case OverlapKind::SameDirection: return "same_direction";
    case OverlapKind::Crossing: return "crossing";
  }
  return "unknown";
}

const char * resolver_status_name(ResolverStatus s)
{
  switch (s) {
    case ResolverStatus::Free: return "FREE";
    case ResolverStatus::Wait: return "WAIT";
    case ResolverStatus::RouteComplete: return "ROUTE_COMPLETE";
    case ResolverStatus::UnknownRoute: return "UNKNOWN_ROUTE";
    case ResolverStatus::Recovery: return "RECOVERY_REQUIRED";
  }
  return "UNKNOWN";
}

GlobalResolver::GlobalResolver(ResolverConfig cfg)
: cfg_(cfg)
{
  if (cfg_.sample_step_m <= 0.0) {
    cfg_.sample_step_m = 0.25;
  }
  if (cfg_.merge_gap_m <= 0.0) {
    cfg_.merge_gap_m = 2.0 * cfg_.safety_distance_m + cfg_.conflict_distance_m;
  }
  if (cfg_.junction_clear_m <= 0.0) {
    cfg_.junction_clear_m = cfg_.safety_distance_m;
  }
}

void GlobalResolver::set_junctions(std::vector<std::pair<double, double>> centres)
{
  junctions_ = std::move(centres);
}

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

void GlobalResolver::build_geometry(Route & r, double step)
{
  r.s.assign(r.x.size(), 0.0);
  r.samples.clear();
  for (std::size_t i = 1; i < r.x.size(); ++i) {
    r.s[i] = r.s[i - 1] + std::hypot(r.x[i] - r.x[i - 1], r.y[i] - r.y[i - 1]);
  }
  r.length = r.x.empty() ? 0.0 : r.s.back();
  if (r.x.empty()) {
    return;
  }
  for (std::size_t i = 0; i + 1 < r.x.size(); ++i) {
    const double L = r.s[i + 1] - r.s[i];
    const int n = std::max(1, static_cast<int>(std::ceil(L / step)));
    for (int k = 0; k < n; ++k) {
      const double t = static_cast<double>(k) / n;
      r.samples.push_back(
        {r.s[i] + t * L, r.x[i] + t * (r.x[i + 1] - r.x[i]),
          r.y[i] + t * (r.y[i + 1] - r.y[i])});
    }
  }
  r.samples.push_back({r.length, r.x.back(), r.y.back()});
}

int32_t GlobalResolver::index_at_or_before(const Route & r, double s)
{
  if (r.s.empty()) {
    return -1;
  }
  const auto it = std::upper_bound(r.s.begin(), r.s.end(), s + kEps);
  return static_cast<int32_t>(it - r.s.begin()) - 1;
}

int32_t GlobalResolver::index_at_or_after(const Route & r, double s)
{
  if (r.s.empty()) {
    return -1;
  }
  const auto it = std::lower_bound(r.s.begin(), r.s.end(), s - kEps);
  if (it == r.s.end()) {
    return static_cast<int32_t>(r.s.size()) - 1;
  }
  return static_cast<int32_t>(it - r.s.begin());
}

GlobalResolver::Sample GlobalResolver::point_at(const Route & r, double s)
{
  if (r.x.empty()) {
    return {};
  }
  if (r.x.size() == 1 || s <= 0.0) {
    return {0.0, r.x.front(), r.y.front()};
  }
  if (s >= r.length) {
    return {r.length, r.x.back(), r.y.back()};
  }
  const int32_t i = std::max<int32_t>(0, index_at_or_before(r, s));
  const std::size_t j = std::min<std::size_t>(static_cast<std::size_t>(i) + 1, r.x.size() - 1);
  const double L = r.s[j] - r.s[static_cast<std::size_t>(i)];
  const double t = L > kEps ? (s - r.s[static_cast<std::size_t>(i)]) / L : 0.0;
  return {s, r.x[static_cast<std::size_t>(i)] + t * (r.x[j] - r.x[static_cast<std::size_t>(i)]),
    r.y[static_cast<std::size_t>(i)] + t * (r.y[j] - r.y[static_cast<std::size_t>(i)])};
}

double GlobalResolver::committed_s(const Route & r) const
{
  double c = r.progress_valid ? r.progress_s : 0.0;
  if (r.granted >= 0 && static_cast<std::size_t>(r.granted) < r.s.size()) {
    c = std::max(c, r.s[static_cast<std::size_t>(r.granted)]);
  }
  return c;
}

double GlobalResolver::remaining_from(const Route & r) const
{
  return std::max(0.0, (r.progress_valid ? r.progress_s : 0.0) - cfg_.body_back_m);
}

bool GlobalResolver::parked(const Route & r) const
{
  if (r.standing()) {
    return true;
  }
  return r.progress_valid && r.length - r.progress_s < 0.5 * cfg_.conflict_distance_m;
}

std::vector<GlobalResolver::Overlap> GlobalResolver::find_overlaps(
  const Route & a, const Route & b) const
{
  std::vector<Overlap> out;
  const double cd = cfg_.conflict_distance_m;
  const double cd2 = cd * cd;
  const double ra = remaining_from(a);
  const double rb = remaining_from(b);

  // Spatial hash over each route's remaining samples (cell = cd).
  auto key = [cd](double x, double y) {
      const int64_t ix = static_cast<int64_t>(std::floor(x / cd));
      const int64_t iy = static_cast<int64_t>(std::floor(y / cd));
      return (ix << 32) ^ (iy & 0xffffffffLL);
    };
  auto build = [&](const Route & r, double from) {
      std::unordered_map<int64_t, std::vector<std::size_t>> g;
      for (std::size_t i = 0; i < r.samples.size(); ++i) {
        if (r.samples[i].s + kEps < from) {
          continue;
        }
        g[key(r.samples[i].x, r.samples[i].y)].push_back(i);
      }
      return g;
    };
  auto nearest = [&](
    const std::unordered_map<int64_t, std::vector<std::size_t>> & g,
    const Route & r, double x, double y) -> int64_t {
      const int64_t ix = static_cast<int64_t>(std::floor(x / cd));
      const int64_t iy = static_cast<int64_t>(std::floor(y / cd));
      int64_t best = -1;
      double best_d = cd2 + kEps;
      for (int64_t dx = -1; dx <= 1; ++dx) {
        for (int64_t dy = -1; dy <= 1; ++dy) {
          const auto it = g.find(((ix + dx) << 32) ^ ((iy + dy) & 0xffffffffLL));
          if (it == g.end()) {
            continue;
          }
          for (const auto j : it->second) {
            const double d = sq(r.samples[j].x - x) + sq(r.samples[j].y - y);
            if (d < best_d) {
              best_d = d;
              best = static_cast<int64_t>(j);
            }
          }
        }
      }
      return best;
    };

  const auto grid_a = build(a, ra);
  const auto grid_b = build(b, rb);
  std::vector<int64_t> near_b(a.samples.size(), -1);  // a sample → nearest b
  std::vector<int64_t> near_a(b.samples.size(), -1);  // b sample → nearest a
  for (std::size_t i = 0; i < a.samples.size(); ++i) {
    if (a.samples[i].s + kEps >= ra) {
      near_b[i] = nearest(grid_b, b, a.samples[i].x, a.samples[i].y);
    }
  }
  for (std::size_t j = 0; j < b.samples.size(); ++j) {
    if (b.samples[j].s + kEps >= rb) {
      near_a[j] = nearest(grid_a, a, b.samples[j].x, b.samples[j].y);
    }
  }

  // Contiguous hit runs along each route.
  auto runs_of = [](const std::vector<int64_t> & hits) {
      std::vector<int> run(hits.size(), -1);
      int id = -1;
      bool in = false;
      for (std::size_t i = 0; i < hits.size(); ++i) {
        if (hits[i] >= 0) {
          if (!in) {
            ++id;
            in = true;
          }
          run[i] = id;
        } else {
          in = false;
        }
      }
      return std::make_pair(run, id + 1);
    };
  const auto [run_a, n_a] = runs_of(near_b);
  const auto [run_b, n_b] = runs_of(near_a);
  if (n_a == 0 || n_b == 0) {
    return out;
  }

  struct Acc
  {
    bool any_a{false};
    double a_in{kInf}, a_out{-kInf};
    double b_first{0}, b_last{0};
    bool any_b{false};
    double b_in{kInf}, b_out{-kInf};
  };
  std::map<std::pair<int, int>, Acc> acc;
  for (std::size_t i = 0; i < a.samples.size(); ++i) {
    if (run_a[i] < 0 || near_b[i] < 0) {
      continue;
    }
    const int jr = run_b[static_cast<std::size_t>(near_b[i])];
    if (jr < 0) {
      continue;
    }
    auto & e = acc[{run_a[i], jr}];
    const double bs = b.samples[static_cast<std::size_t>(near_b[i])].s;
    if (!e.any_a) {
      e.b_first = bs;
    }
    e.any_a = true;
    e.a_in = std::min(e.a_in, a.samples[i].s);
    e.a_out = std::max(e.a_out, a.samples[i].s);
    e.b_last = bs;
  }
  for (std::size_t j = 0; j < b.samples.size(); ++j) {
    if (run_b[j] < 0 || near_a[j] < 0) {
      continue;
    }
    const int ir = run_a[static_cast<std::size_t>(near_a[j])];
    if (ir < 0) {
      continue;
    }
    auto it = acc.find({ir, run_b[j]});
    if (it == acc.end()) {
      continue;
    }
    it->second.any_b = true;
    it->second.b_in = std::min(it->second.b_in, b.samples[j].s);
    it->second.b_out = std::max(it->second.b_out, b.samples[j].s);
  }
  for (const auto & kv : acc) {
    const auto & e = kv.second;
    if (!e.any_a) {
      continue;
    }
    Overlap o;
    o.a_in = e.a_in;
    o.a_out = e.a_out;
    o.b_at_a_in = e.b_first;
    o.b_at_a_out = e.b_last;
    if (e.any_b) {
      o.b_in = e.b_in;
      o.b_out = e.b_out;
    } else {
      o.b_in = std::min(e.b_first, e.b_last);
      o.b_out = std::max(e.b_first, e.b_last);
    }
    out.push_back(o);
  }

  // Merge blocks with no safe room between them (on either route).
  std::sort(out.begin(), out.end(), [](const Overlap & l, const Overlap & r) {
      return l.a_in < r.a_in;
    });
  bool merged = true;
  while (merged && out.size() > 1) {
    merged = false;
    for (std::size_t i = 0; i + 1 < out.size() && !merged; ++i) {
      for (std::size_t j = i + 1; j < out.size() && !merged; ++j) {
        const auto & p = out[i];
        const auto & q = out[j];
        const double gap_a = std::max(p.a_in, q.a_in) - std::min(p.a_out, q.a_out);
        const double gap_b = std::max(p.b_in, q.b_in) - std::min(p.b_out, q.b_out);
        if (gap_a < cfg_.merge_gap_m || gap_b < cfg_.merge_gap_m) {
          Overlap m;
          m.a_in = std::min(p.a_in, q.a_in);
          m.a_out = std::max(p.a_out, q.a_out);
          m.b_in = std::min(p.b_in, q.b_in);
          m.b_out = std::max(p.b_out, q.b_out);
          m.b_at_a_in = p.a_in <= q.a_in ? p.b_at_a_in : q.b_at_a_in;
          m.b_at_a_out = p.a_out >= q.a_out ? p.b_at_a_out : q.b_at_a_out;
          out[i] = m;
          out.erase(out.begin() + static_cast<std::ptrdiff_t>(j));
          merged = true;
        }
      }
    }
  }
  return out;
}

// ---------------------------------------------------------------------------
// Registration
// ---------------------------------------------------------------------------

std::size_t GlobalResolver::geometry_hash(const std::vector<ResolverRoutePoint> & points)
{
  std::size_t h = points.size();
  for (const auto & p : points) {
    const auto qx = static_cast<int64_t>(std::llround(p.x * 1000.0));
    const auto qy = static_cast<int64_t>(std::llround(p.y * 1000.0));
    h ^= std::hash<int64_t>{}(qx) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    h ^= std::hash<int64_t>{}(qy) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
  }
  return h;
}

void GlobalResolver::store_and_decide(
  const std::string & amr_id, const std::string & key, const std::string & base_key,
  const std::string & session, Source source,
  const std::vector<ResolverRoutePoint> & points, std::size_t geo_hash)
{
  const auto it = routes_.find(amr_id);
  Route r;
  r.amr_id = amr_id;
  r.key = key;
  r.base_key = base_key;
  r.session = session;
  r.source = source;
  r.geo_hash = geo_hash;
  r.reg_seq = next_reg_seq_++;
  for (const auto & p : points) {
    r.x.push_back(p.x);
    r.y.push_back(p.y);
  }
  build_geometry(r, cfg_.sample_step_m);
  if (it != routes_.end() && it->second.pose_valid) {
    r.pose_x = it->second.pose_x;
    r.pose_y = it->second.pose_y;
    r.pose_valid = true;
  }
  routes_[amr_id] = std::move(r);
  auto & stored = routes_[amr_id];
  // Initial progress: project the last known pose onto the route start.
  if (stored.pose_valid && !stored.standing()) {
    double best = kInf;
    double best_s = 0.0;
    const double hi = std::min(stored.length, 5.0);
    for (const auto & smp : stored.samples) {
      if (smp.s > hi + kEps) {
        break;
      }
      const double d = std::hypot(smp.x - stored.pose_x, smp.y - stored.pose_y);
      if (d < best) {
        best = d;
        best_s = smp.s;
      }
    }
    if (best <= cfg_.max_offroute_m) {
      stored.progress_s = best_s;
      stored.progress_valid = true;
    }
  }
  const char * src = source == Source::Handover ? "RMF handover" :
    (source == Source::Intent ? "TM PathIntent" : "registration");
  log(
    "[ECBS-ROUTE] " + amr_id + " " + src + " key=[" + key + "] wps=" +
    std::to_string(stored.x.size()) + " length=" + f2(stored.length) + "m" +
    (stored.standing() ? " (standing)" : " — deciding all conflicts now"));
  resolve_for(amr_id);
}

bool GlobalResolver::decide_pending_intent(const std::string & amr_id)
{
  const auto pit = pending_intents_.find(amr_id);
  if (pit == pending_intents_.end()) {
    return false;
  }
  const PendingIntent pending = pit->second;
  pending_intents_.erase(pit);
  log(
    "[ECBS-WARN] " + amr_id + " no RMF handover received for [" + pending.base_key +
    "] — deciding on the TM PathIntent (fallback)");
  seen_handover_keys_[amr_id].insert(pending.base_key);
  store_and_decide(
    amr_id, pending.base_key + "#" + pending.session, pending.base_key, pending.session,
    Source::Intent, pending.points, geometry_hash(pending.points));
  return true;
}

bool GlobalResolver::has_pending_intent(const std::string & amr_id) const
{
  return pending_intents_.count(amr_id) > 0;
}

bool GlobalResolver::upsert_route(
  const std::string & amr_id,
  const std::string & route_key,
  const std::vector<ResolverRoutePoint> & points)
{
  if (amr_id.empty() || points.empty()) {
    return false;
  }
  const std::size_t h = geometry_hash(points);
  const auto it = routes_.find(amr_id);
  if (it != routes_.end() && it->second.geo_hash == h && it->second.key == route_key) {
    return false;  // identical registration — keep progress, grant, decisions
  }
  store_and_decide(amr_id, route_key, route_key, "", Source::Generic, points, h);
  return true;
}

std::string GlobalResolver::register_handover(
  const std::string & amr_id,
  const std::string & base_key,
  const std::vector<ResolverRoutePoint> & points)
{
  if (amr_id.empty() || points.size() < 2) {
    return "ignored";
  }
  const auto it = routes_.find(amr_id);
  if (it != routes_.end() && !base_key.empty() && it->second.base_key == base_key) {
    return "unchanged";  // already registered (handover or its PathIntent)
  }
  auto & seen = seen_handover_keys_[amr_id];
  if (!base_key.empty() && seen.count(base_key)) {
    // A stale copy of a route this robot has already moved past (e.g. the
    // fleet adapter re-sending another robot's old path).
    return "ignored_stale";
  }
  if (!base_key.empty()) {
    seen.insert(base_key);
  }
  store_and_decide(
    amr_id, base_key, base_key, "", Source::Handover, points, geometry_hash(points));
  // A PathIntent that arrived before this handover binds now.
  const auto pit = pending_intents_.find(amr_id);
  if (pit != pending_intents_.end() && pit->second.base_key == base_key) {
    const PendingIntent pending = pit->second;
    pending_intents_.erase(pit);
    register_intent(amr_id, pending.base_key, pending.session, pending.points);
  }
  return "decided";
}

bool GlobalResolver::geometry_matches(
  const Route & cur, const std::vector<ResolverRoutePoint> & points) const
{
  if (cur.standing() || points.size() < 2) {
    return false;
  }
  Route other;
  for (const auto & p : points) {
    other.x.push_back(p.x);
    other.y.push_back(p.y);
  }
  build_geometry(other, cfg_.sample_step_m);
  constexpr double kTolM = 0.5;
  if (std::abs(other.length - cur.length) > 2.0 * kTolM) {
    return false;
  }
  auto near_poly = [&](const Route & r, double x, double y) {
      double best = kInf;
      for (const auto & smp : r.samples) {
        best = std::min(best, std::hypot(smp.x - x, smp.y - y));
      }
      return best <= kTolM;
    };
  for (std::size_t i = 0; i < other.x.size(); ++i) {
    if (!near_poly(cur, other.x[i], other.y[i])) {
      return false;
    }
  }
  for (std::size_t i = 0; i < cur.x.size(); ++i) {
    if (!near_poly(other, cur.x[i], cur.y[i])) {
      return false;
    }
  }
  return std::hypot(cur.x.front() - other.x.front(), cur.y.front() - other.y.front()) <= kTolM &&
         std::hypot(cur.x.back() - other.x.back(), cur.y.back() - other.y.back()) <= kTolM;
}

double GlobalResolver::map_s(const Route & from, const Route & to, double s) const
{
  const auto p = point_at(from, s);
  double best = kInf;
  double best_s = std::min(std::max(0.0, s), to.length);
  for (const auto & smp : to.samples) {
    if (smp.s < s - 3.0) {
      continue;
    }
    if (smp.s > s + 3.0) {
      break;
    }
    const double d = std::hypot(smp.x - p.x, smp.y - p.y);
    if (d < best) {
      best = d;
      best_s = smp.s;
    }
  }
  return best_s;
}

void GlobalResolver::rebind(
  const std::string & amr_id, const std::string & session,
  const std::vector<ResolverRoutePoint> & points)
{
  Route & cur = routes_.at(amr_id);
  Route nr;
  nr.amr_id = amr_id;
  nr.key = cur.base_key + "#" + session;
  nr.base_key = cur.base_key;
  nr.session = session;
  nr.source = Source::Intent;
  nr.geo_hash = geometry_hash(points);
  nr.reg_seq = cur.reg_seq;  // FIFO keeps the handover time
  for (const auto & p : points) {
    nr.x.push_back(p.x);
    nr.y.push_back(p.y);
  }
  build_geometry(nr, cfg_.sample_step_m);
  nr.pose_x = cur.pose_x;
  nr.pose_y = cur.pose_y;
  nr.pose_valid = cur.pose_valid;
  nr.progress_valid = cur.progress_valid;
  nr.progress_s = cur.progress_valid ? map_s(cur, nr, cur.progress_s) : 0.0;
  nr.granted = cur.granted >= 0 ?
    index_at_or_before(nr, map_s(cur, nr, cur.s[static_cast<std::size_t>(cur.granted)])) : -1;
  const double step = cfg_.sample_step_m;
  for (auto & kv : decisions_) {
    auto & d = kv.second;
    if (!d.involves(amr_id)) {
      continue;
    }
    auto & p = d.p[0].amr_id == amr_id ? d.p[0] : d.p[1];
    p.entry_s = map_s(cur, nr, p.entry_s);
    p.exit_s = map_s(cur, nr, p.exit_s);
    p.entry_index = std::max<int32_t>(0, index_at_or_before(nr, p.entry_s));
    p.exit_index = index_at_or_after(nr, p.exit_s);
    p.parks_inside = p.exit_s >= nr.length - 1.5 * step;
    if (d.second == amr_id) {
      // Hold at the last PathIntent waypoint not past the decided wait point.
      const double ws = map_s(cur, nr, d.wait_s);
      const int32_t wi = std::max<int32_t>(
        std::max(0, nr.granted), index_at_or_before(nr, ws));
      d.wait_index = wi;
      d.wait_s = nr.s[static_cast<std::size_t>(wi)];
      d.wait_x = nr.x[static_cast<std::size_t>(wi)];
      d.wait_y = nr.y[static_cast<std::size_t>(wi)];
    }
    if (d.first == amr_id) {
      d.release_s = std::min(map_s(cur, nr, d.release_s), nr.length);
      const auto rp = point_at(nr, d.release_s);
      d.release_x = rp.x;
      d.release_y = rp.y;
    }
  }
  cur = std::move(nr);
  log(
    "[ECBS-ROUTE] " + amr_id + " TM PathIntent session=" + session +
    " bound to RMF handover route [" + cur.base_key + "] wps=" + std::to_string(cur.x.size()) +
    " — decisions kept (" + std::to_string(decisions_for(amr_id).size()) + ")");
  mark_changed(amr_id);
}

std::string GlobalResolver::register_intent(
  const std::string & amr_id,
  const std::string & base_key,
  const std::string & session,
  const std::vector<ResolverRoutePoint> & points)
{
  if (amr_id.empty() || points.empty()) {
    return "ignored";
  }
  const std::size_t h = geometry_hash(points);
  const std::string key = base_key + "#" + session;
  const auto it = routes_.find(amr_id);
  const bool task_known = !base_key.empty() && base_key.find("||") == std::string::npos;
  // Stale TM session: this identity was already handed over / registered and
  // the robot has since moved to a newer route. Never let it park or override
  // the newer RMF handover (seen when TM re-seeded the finished task while the
  // next one was staged).
  if (task_known && it != routes_.end() && !it->second.base_key.empty() &&
    it->second.base_key != base_key && seen_handover_keys_[amr_id].count(base_key))
  {
    log(
      "[ECBS-ROUTE] " + amr_id + " TM PathIntent [" + base_key +
      "] is an older session than the current route [" + it->second.base_key +
      "] — ignored (no re-decision)");
    return "stale_session";
  }
  if (points.size() >= 2 && cfg_.decide_on_handover_only && task_known &&
    (it == routes_.end() || it->second.base_key != base_key))
  {
    // Decisions are made on the RMF handover only: park this PathIntent and
    // hold the robot until GoToPlace's handover for the same identity arrives.
    const bool fresh = !pending_intents_.count(amr_id) ||
      pending_intents_[amr_id].base_key != base_key;
    pending_intents_[amr_id] = PendingIntent{base_key, session, points};
    if (fresh) {
      log(
        "[ECBS-ROUTE] " + amr_id + " TM PathIntent [" + base_key +
        "] arrived before its RMF handover — parked; robot holds until the handover is evaluated");
      mark_changed(amr_id);
    }
    return "awaiting_handover";
  }
  pending_intents_.erase(amr_id);
  if (it != routes_.end()) {
    const Route & cur = it->second;
    if (cur.key == key && cur.geo_hash == h) {
      return "unchanged";  // same session reseed (route recovery)
    }
    const bool same_identity = !base_key.empty() && cur.base_key == base_key &&
      base_key.find("||") == std::string::npos;  // task id known
    const bool finished = cur.progress_valid && cur.length - cur.progress_s < 1.0;
    if (same_identity && !finished && points.size() >= 2 &&
      (cur.geo_hash == h || geometry_matches(cur, points)))
    {
      rebind(amr_id, session, points);
      return "rebound";
    }
    if (same_identity && cur.source == Source::Handover) {
      log(
        "[ECBS-WARN] " + amr_id + " PathIntent geometry differs from the RMF handover [" +
        base_key + "] — re-deciding on the path the robot will actually execute");
    }
  }
  if (!base_key.empty()) {
    seen_handover_keys_[amr_id].insert(base_key);
  }
  store_and_decide(amr_id, key, base_key, session, Source::Intent, points, h);
  return "decided";
}

void GlobalResolver::set_standing(const std::string & amr_id, double x, double y)
{
  auto it = routes_.find(amr_id);
  if (it != routes_.end() && it->second.standing() &&
    std::hypot(it->second.x.front() - x, it->second.y.front() - y) < 0.3)
  {
    it->second.pose_x = x;
    it->second.pose_y = y;
    it->second.pose_valid = true;
    return;
  }
  const bool had_decisions = it != routes_.end() && !it->second.standing();
  Route r;
  r.amr_id = amr_id;
  r.key = "standing";
  r.reg_seq = next_reg_seq_++;
  r.x = {x};
  r.y = {y};
  r.pose_x = x;
  r.pose_y = y;
  r.pose_valid = true;
  r.progress_valid = true;
  build_geometry(r, cfg_.sample_step_m);
  routes_[amr_id] = std::move(r);
  if (had_decisions) {
    resolve_for(amr_id);
  }
}

bool GlobalResolver::has_route(const std::string & amr_id) const
{
  return routes_.count(amr_id) > 0;
}

std::string GlobalResolver::route_key(const std::string & amr_id) const
{
  const auto it = routes_.find(amr_id);
  return it == routes_.end() ? std::string{} : it->second.key;
}

void GlobalResolver::set_facts(const std::string & amr_id, const ResolverFacts & facts)
{
  facts_[amr_id] = facts;
}

// ---------------------------------------------------------------------------
// Decisions
// ---------------------------------------------------------------------------

void GlobalResolver::resolve_for(const std::string & amr_id)
{
  const auto rit = routes_.find(amr_id);
  if (rit == routes_.end()) {
    return;
  }
  // Previous decisions of this robot: reuse when the same overlap reappears.
  std::map<std::string, ResolverDecision> previous;
  for (auto it = decisions_.begin(); it != decisions_.end(); ) {
    if (it->second.involves(amr_id)) {
      previous.emplace(it->first, it->second);
      it = decisions_.erase(it);
    } else {
      ++it;
    }
  }

  const Route & me = rit->second;
  std::set<std::string> seen;
  if (!parked(me)) {
    for (const auto & kv : routes_) {
      if (kv.first == amr_id || parked(kv.second)) {
        continue;
      }
      const Route & a = amr_id < kv.first ? me : kv.second;
      const Route & b = amr_id < kv.first ? kv.second : me;
      for (const auto & ov : find_overlaps(a, b)) {
        ResolverDecision d;
        const double step = cfg_.sample_step_m;
        auto fill = [&](ResolverParticipant & p, const Route & r, double in, double out_s) {
            p.amr_id = r.amr_id;
            p.entry_s = in;
            p.exit_s = out_s;
            p.entry_index = std::max<int32_t>(0, index_at_or_before(r, in));
            p.exit_index = index_at_or_after(r, out_s);
            p.parks_inside = out_s >= r.length - 1.5 * step;
          };
        fill(d.p[0], a, ov.a_in, ov.a_out);
        fill(d.p[1], b, ov.b_in, ov.b_out);
        const double len_a = ov.a_out - ov.a_in;
        const double len_b = ov.b_out - ov.b_in;
        const double trend = ov.b_at_a_out - ov.b_at_a_in;
        if (len_a < cfg_.crossing_max_len_m && len_b < cfg_.crossing_max_len_m) {
          d.kind = OverlapKind::Crossing;
        } else if (trend > 2.0 * step) {
          d.kind = OverlapKind::SameDirection;
        } else if (trend < -2.0 * step) {
          d.kind = OverlapKind::Opposite;
        } else {
          d.kind = OverlapKind::Crossing;
        }
        d.conflict_id = a.amr_id + ":" + hex(a.geo_hash) + "@" +
          std::to_string(d.p[0].entry_index) + "|" + b.amr_id + ":" + hex(b.geo_hash) +
          "@" + std::to_string(d.p[1].entry_index);
        if (!seen.insert(d.conflict_id).second) {
          d.conflict_id += "#" + std::to_string(seen.size());
          seen.insert(d.conflict_id);
        }

        const auto prev = previous.find(d.conflict_id);
        bool reused = false;
        if (prev != previous.end() && !prev->second.unresolvable) {
          // Sticky: keep the earlier order while its waiter can still hold.
          ResolverDecision keep = prev->second;
          keep.kind = d.kind;
          keep.p[0] = d.p[0];
          keep.p[1] = d.p[1];
          const auto sit = routes_.find(keep.second);
          if (keep.cleared ||
            (sit != routes_.end() && keep.wait_index >= sit->second.granted))
          {
            d = keep;
            reused = true;
          }
        }
        if (!reused) {
          d.decided_seq = next_decided_seq_++;
          decide(d);
        }
        decisions_[d.conflict_id] = d;
      }
    }
  }

  enforce_waiter_occupancy();
  break_cycles();
  {
    // Waits of this robot's new decisions are final once all its decisions
    // exist (exemptions depend on them): settle them on the nearest waypoint.
    std::set<std::string> fresh;
    for (const auto & kv : decisions_) {
      if (kv.second.involves(amr_id) && !previous.count(kv.first)) {
        fresh.insert(kv.first);
      }
    }
    if (refine_waits(fresh)) {
      enforce_waiter_occupancy();
      break_cycles();
    }
  }

  // Explain new decisions; wake robots whose constraints vanished.
  for (const auto & kv : decisions_) {
    const auto & d = kv.second;
    if (!d.involves(amr_id) || previous.count(kv.first)) {
      continue;
    }
    const auto & pf = d.of(d.first);
    const auto & ps = d.of(d.second);
    std::ostringstream os;
    os << "[ECBS-DECIDE] conflict=" << d.conflict_id << " kind=" << overlap_kind_name(d.kind)
       << " | FIRST=" << d.first << " passes overlap wp" << pf.entry_index << "..wp"
       << pf.exit_index << " | WAIT=" << d.second << " holds at wp" << d.wait_index << " "
       << xy(d.wait_x, d.wait_y) << " before its overlap wp" << ps.entry_index << "..wp"
       << ps.exit_index << ", released when " << d.first << " passes "
       << xy(d.release_x, d.release_y) << " | rule=" << d.rule
       << (d.relaxed_wait ? " | NOTE wait point only clear of the blocker" : "")
       << (d.unresolvable ? " | UNRESOLVABLE" : "");
    log(os.str());
    mark_changed(d.first);
    mark_changed(d.second);
  }
  for (const auto & kv : previous) {
    if (!decisions_.count(kv.first)) {
      log(
        "[ECBS-DECIDE] conflict=" + kv.first + " dropped (route of " + amr_id +
        " changed) — " + kv.second.second + " no longer waits for it");
      mark_changed(kv.second.first);
      mark_changed(kv.second.second);
    }
  }
  mark_changed(amr_id);
}

std::optional<GlobalResolver::WaitChoice> GlobalResolver::find_wait(
  const Route & waiter, const ResolverDecision & d) const
{
  const auto & part = d.of(waiter.amr_id);
  const std::string & blocker = d.other(waiter.amr_id);
  const double cs = committed_s(waiter);
  const double cd2 = sq(cfg_.conflict_distance_m) - kEps;
  const double jc2 = sq(cfg_.junction_clear_m);
  // Arc range [lo, hi] of r's remaining samples within cd of (x, y).
  auto near_range = [&](const Route & r, double x, double y, double & lo, double & hi) {
      lo = kInf;
      hi = -kInf;
      if (r.standing()) {
        if (sq(r.x.front() - x) + sq(r.y.front() - y) < cd2) {
          lo = hi = 0.0;
        }
        return lo <= hi;
      }
      const double from = remaining_from(r);
      for (const auto & smp : r.samples) {
        if (smp.s + kEps < from) {
          continue;
        }
        if (sq(smp.x - x) + sq(smp.y - y) < cd2) {
          lo = std::min(lo, smp.s);
          hi = std::max(hi, smp.s);
        }
      }
      return lo <= hi;
    };
  // Standing at arc sk is harmless for r when the decided order keeps the two
  // apart there: r passes that spot first and the waiter is held before it
  // until r is released past it, or r is held at its own wait point (behind
  // that spot) until the waiter has passed.
  auto ordered_apart = [&](const Route & r, double sk, double lo, double hi) {
      if (r.standing()) {
        return false;
      }
      const double cs_r = committed_s(r);
      const double cdm = cfg_.conflict_distance_m;
      for (const auto & kv : decisions_) {
        const auto & e = kv.second;
        if (kv.first == d.conflict_id || e.unresolvable || e.first.empty() ||
          !e.involves(waiter.amr_id) || !e.involves(r.amr_id))
        {
          continue;
        }
        // Same direction: inside the shared section the follower trails the
        // leader (graph edge leader → follower), so the leader holding there
        // only queues the follower, and the follower reaches the spot only
        // after the leader has passed it.
        if (e.kind == OverlapKind::SameDirection && (!e.cleared || leader_in_section(e))) {
          const auto & pw = e.of(waiter.amr_id);
          const auto & pr = e.of(r.amr_id);
          const bool in_w = sk >= pw.entry_s - cdm - kEps && sk <= pw.exit_s + cdm + kEps;
          const bool in_r = lo >= pr.entry_s - cdm - kEps && hi <= pr.exit_s + cdm + kEps;
          if (in_w && in_r) {
            if (e.first == waiter.amr_id && cs_r + cfg_.body_back_m < lo) {
              return true;  // r follows behind the waiter
            }
            if (e.first == r.amr_id &&
              (e.cleared || (e.wait_s <= sk + kEps && cs <= e.wait_s + kEps)))
            {
              return true;  // r leads; the waiter trails it to the spot
            }
          }
        }
        if (e.cleared) {
          continue;
        }
        if (e.first == r.amr_id) {
          if (e.wait_s <= sk + kEps && cs <= e.wait_s + kEps &&
            hi + cfg_.body_back_m <= e.release_s + kEps)
          {
            return true;
          }
        } else if (sk + kEps < e.release_s && lo > e.wait_s + kEps &&
          cs_r <= e.wait_s + kEps)
        {
          return true;
        }
      }
      return false;
    };
  const auto bit = routes_.find(blocker);
  // -1 on the blocker's path, 0 blocks another robot, 1 near a junction, 2 clear.
  auto tier_of = [&](int32_t k, double sk) -> int {
      const double x = waiter.x[static_cast<std::size_t>(k)];
      const double y = waiter.y[static_cast<std::size_t>(k)];
      double lo = 0.0;
      double hi = 0.0;
      if (bit != routes_.end() && near_range(bit->second, x, y, lo, hi)) {
        return -1;
      }
      for (const auto & kv : routes_) {
        if (kv.first == waiter.amr_id || kv.first == blocker) {
          continue;
        }
        if (near_range(kv.second, x, y, lo, hi) && !ordered_apart(kv.second, sk, lo, hi)) {
          return 0;
        }
      }
      const bool at_robot = std::abs(sk - cs) <= cfg_.body_back_m;
      if (!at_robot) {
        for (const auto & j : junctions_) {
          if (sq(j.first - x) + sq(j.second - y) < jc2) {
            return 1;
          }
        }
      }
      return 2;
    };

  // 1) Regular hold: the NEAREST waypoint >= safety_distance_m before the
  //    entry that blocks nobody. A junction-free spot is preferred only within
  //    a short look-back; a spot that would block another robot is used only
  //    when nothing else exists behind it (relaxed).
  std::optional<WaitChoice> clear_wp;
  std::optional<WaitChoice> relaxed_wp;
  const double look_back = cfg_.junction_clear_m + cfg_.safety_distance_m;
  const double limit = part.entry_s - cfg_.safety_distance_m;
  if (limit >= cs - cfg_.body_back_m - kEps) {
    for (int32_t k = index_at_or_before(waiter, limit); k >= 0; --k) {
      const double sk = waiter.s[static_cast<std::size_t>(k)];
      if (sk < cs - cfg_.body_back_m - kEps || k < waiter.granted) {
        break;
      }
      if (clear_wp && sk < clear_wp->s - look_back - kEps) {
        break;
      }
      const int tier = tier_of(k, sk);
      if (tier == 2) {
        return WaitChoice{k, sk, 2};
      }
      if (tier == 1 && !clear_wp) {
        clear_wp = WaitChoice{k, sk, 1};
      } else if (tier == 0 && !relaxed_wp) {
        relaxed_wp = WaitChoice{k, sk, 0};
      }
    }
  }
  if (clear_wp) {
    return clear_wp;
  }
  if (relaxed_wp) {
    return relaxed_wp;
  }
  // 2) Hold in place: a robot that will not move further (its committed
  //    waypoint — granted tip / where it stands) is a safe hold when that spot
  //    is still outside the overlap and off the blocker's path, even if it is
  //    closer than safety_distance_m (e.g. robots parked nose-to-tail: the
  //    rear one simply stays while the front one drives away).
  const int32_t k0 = waiter.granted >= 0 ?
    waiter.granted : std::max(0, index_at_or_before(waiter, cs));
  const double s0 = waiter.s[static_cast<std::size_t>(k0)];
  if (s0 <= part.entry_s + kEps) {
    const int tier = tier_of(k0, s0);
    if (tier >= 0) {
      return WaitChoice{k0, s0, tier};
    }
  }
  return std::nullopt;
}

bool GlobalResolver::sits_on(const Route & x, const Route & y) const
{
  const double px = x.pose_valid ? x.pose_x : (x.x.empty() ? 0.0 : x.x.front());
  const double py = x.pose_valid ? x.pose_y : (x.y.empty() ? 0.0 : x.y.front());
  const double cd2 = sq(cfg_.conflict_distance_m) - kEps;
  if (y.standing()) {
    return sq(y.x.front() - px) + sq(y.y.front() - py) < cd2;
  }
  // Ahead of y's body only: y's own rear (body_back) is not "y's path".
  const double from = y.progress_valid ? y.progress_s + cfg_.body_back_m : 0.0;
  for (const auto & smp : y.samples) {
    if (smp.s + kEps < from) {
      continue;
    }
    if (sq(smp.x - px) + sq(smp.y - py) < cd2) {
      return true;
    }
  }
  return false;
}

bool GlobalResolver::assign_wait(ResolverDecision & d)
{
  const auto sit = routes_.find(d.second);
  const auto fit = routes_.find(d.first);
  if (sit == routes_.end() || fit == routes_.end()) {
    return false;
  }
  const auto & F = fit->second;
  const auto & pf = d.of(d.first);
  double rel = d.kind == OverlapKind::SameDirection ?
    pf.entry_s + cfg_.safety_distance_m + cfg_.conflict_distance_m :
    pf.exit_s + cfg_.safety_distance_m;
  rel = std::min(rel, F.length);
  d.release_s = rel;
  const auto rp = point_at(F, rel);
  d.release_x = rp.x;
  d.release_y = rp.y;

  const auto w = find_wait(sit->second, d);
  if (!w) {
    // Hold where the robot is committed (cannot do better).
    const auto & S = sit->second;
    const int32_t at = std::max<int32_t>(
      std::max(0, S.granted), index_at_or_before(S, committed_s(S)));
    d.wait_index = at;
    d.wait_s = S.s[static_cast<std::size_t>(at)];
    d.wait_x = S.x[static_cast<std::size_t>(at)];
    d.wait_y = S.y[static_cast<std::size_t>(at)];
    d.relaxed_wait = true;
    return false;
  }
  const auto & S = sit->second;
  d.wait_index = w->index;
  d.wait_s = w->s;
  d.wait_x = S.x[static_cast<std::size_t>(w->index)];
  d.wait_y = S.y[static_cast<std::size_t>(w->index)];
  d.relaxed_wait = w->tier == 0;
  return true;
}

void GlobalResolver::apply_order(
  ResolverDecision & d, const std::string & first, const std::string & rule,
  int strength, bool forced, bool require_wait)
{
  d.first = first;
  d.second = d.other(first);
  d.rule = rule;
  d.strength = strength;
  d.forced = forced;
  if (!assign_wait(d) && !d.unresolvable && require_wait) {
    report_unresolvable(d);
  }
}

void GlobalResolver::decide(ResolverDecision & d)
{
  const std::string & a = d.p[0].amr_id;
  const std::string & b = d.p[1].amr_id;
  const Route & A = routes_.at(a);
  const Route & B = routes_.at(b);
  const double cs_a = committed_s(A);
  const double cs_b = committed_s(B);
  const double to_a = d.p[0].entry_s - cs_a;
  const double to_b = d.p[1].entry_s - cs_b;
  const auto wait_a = find_wait(A, d);
  const auto wait_b = find_wait(B, d);
  const ResolverFacts fa = facts_.count(a) ? facts_.at(a) : ResolverFacts{};
  const ResolverFacts fb = facts_.count(b) ? facts_.at(b) : ResolverFacts{};
  const bool same = d.kind == OverlapKind::SameDirection;
  const bool cross = d.kind == OverlapKind::Crossing;

  // R1 feasibility (forced)
  if (!wait_a && !wait_b) {
    // Neither can hold outside. Not a deadlock unless BOTH bodies stand on
    // the other's path (face to face): the one standing on the other's path
    // moves first; the other stays where it is and then follows / passes.
    const bool on_a = sits_on(A, B);
    const bool on_b = sits_on(B, A);
    if (on_a != on_b) {
      const std::string first = on_a ? a : b;
      apply_order(
        d, first,
        "unblock: " + first + " stands on " + d.other(first) + "'s path and moves first; " +
        d.other(first) + " is not on " + first + "'s path — it stays where it is, then goes",
        100, true, false);
      return;
    }
    if (!on_a && !on_b) {
      const std::string first = to_a <= to_b ? a : b;
      apply_order(
        d, first,
        "committed: both already committed; " + first + " is further in — passes first, " +
        d.other(first) + " keeps its distance", 100, true, false);
      return;
    }
    d.unresolvable = true;
    const std::string first = to_a <= to_b ? a : b;
    apply_order(
      d, first,
      "unresolvable: " + a + " and " + b +
      " stand on each other's path (face to face) — no order by waiting", 100, true);
    report_unresolvable(d);
    return;
  }
  if (!wait_a || !wait_b) {
    const std::string first = !wait_a ? a : b;
    apply_order(
      d, first,
      "feasibility: " + first + " cannot hold safely before the overlap (starts inside "
      "it / already committed into its approach); " + d.other(first) +
      " can — moving " + first + " first is the only deadlock-free order", 100, true);
    return;
  }
  // R2 who-blocks-whom: robot parking inside the overlap goes second.
  if (d.p[0].parks_inside != d.p[1].parks_inside) {
    const std::string parker = d.p[0].parks_inside ? a : b;
    const std::string first = d.other(parker);
    apply_order(
      d, first,
      "blocking: " + parker + " ends its route inside the overlap (would park on " +
      first + "'s path) — " + first + " passes first", 50, false);
    return;
  }
  // R4 health (an unhealthy robot may never clear the overlap).
  if (fa.healthy != fb.healthy) {
    const std::string first = fa.healthy ? a : b;
    apply_order(
      d, first, "health: " + d.other(first) + " is not healthy — " + first + " first",
      40, false);
    return;
  }
  // R3 same direction: robot ahead leads, robot behind follows.
  if (same) {
    if (std::abs(to_a - to_b) > kEps) {
      const std::string first = to_a < to_b ? a : b;
      apply_order(
        d, first,
        "same_direction: " + first + " is ahead (" + f2(std::min(to_a, to_b)) +
        "m to the shared section) — leads; " + d.other(first) +
        " follows at safety distance", 30, false);
      return;
    }
  }
  // Dependency (who blocks whom, transitively): a robot that is already held
  // for another robot before it even reaches this overlap cannot pass first —
  // making the other wait for it creates a chain wait (amr5→amr2→amr1→amr5).
  {
    const std::string hold_a = held_before(a, d.p[0].entry_s, d.conflict_id);
    const std::string hold_b = held_before(b, d.p[1].entry_s, d.conflict_id);
    if (hold_a.empty() != hold_b.empty()) {
      const std::string held = hold_a.empty() ? b : a;
      const std::string by = hold_a.empty() ? hold_b : hold_a;
      const std::string first = d.other(held);
      apply_order(
        d, first,
        "dependency: " + held + " is already held for " + by +
        " before it reaches this overlap — " + first + " passes first (no chain wait)",
        25, false);
      return;
    }
    // Both already held: the robot whose holds are released sooner passes
    // first. Otherwise a later rule (priority / fifo) can make a robot whose
    // blocker is about to clear wait behind one still held far longer — its
    // own dependency clears and it keeps waiting (late base allocation).
    if (!hold_a.empty() && !hold_b.empty()) {
      const double rem_a = hold_release_remaining(a, d.p[0].entry_s, d.conflict_id);
      const double rem_b = hold_release_remaining(b, d.p[1].entry_s, d.conflict_id);
      if (std::abs(rem_a - rem_b) > cfg_.safety_distance_m) {
        const std::string first = rem_a < rem_b ? a : b;
        const std::string later = d.other(first);
        apply_order(
          d, first,
          "dependency: both held — " + first + "'s hold is released after " +
          f2(std::min(rem_a, rem_b)) + "m of blocker travel, " + later + "'s after " +
          f2(std::max(rem_a, rem_b)) + "m — " + first + " passes first (no chain wait)",
          25, false);
        return;
      }
    }
  }
  // R4 near (crossing): robot already at its approach goes first.
  if (cross) {
    const double near_m = cfg_.safety_distance_m + cfg_.conflict_distance_m;
    const bool na = to_a <= near_m;
    const bool nb = to_b <= near_m;
    if (na != nb) {
      const std::string first = na ? a : b;
      apply_order(
        d, first, "near: " + first + " is already at the crossing approach", 20, false);
      return;
    }
  }
  // R4 free pass: one clears before the other even reaches its wait point.
  if (!same && cfg_.free_pass_ratio > 0.0) {
    auto clear_dist = [&](const ResolverParticipant & p, const Route & r, double cs) {
        return std::min(p.exit_s + cfg_.safety_distance_m, r.length) - cs;
      };
    const double clr_a = clear_dist(d.p[0], A, cs_a);
    const double clr_b = clear_dist(d.p[1], B, cs_b);
    const double app_a = wait_a->s - cs_a;
    const double app_b = wait_b->s - cs_b;
    const bool a_pass = clr_a * cfg_.free_pass_ratio <= app_b;
    const bool b_pass = clr_b * cfg_.free_pass_ratio <= app_a;
    if (a_pass || b_pass) {
      const std::string first = (a_pass && (!b_pass || clr_a <= clr_b)) ? a : b;
      const double c = first == a ? clr_a : clr_b;
      const double ap = first == a ? app_b : app_a;
      apply_order(
        d, first,
        "free_pass: " + first + " clears the overlap in " + f2(c) + "m, before " +
        d.other(first) + " reaches its wait point (" + f2(ap) + "m) — nobody stops",
        10, false);
      return;
    }
  }
  // R4 priority → load → ETA → FIFO → id (zone_manager ranking).
  if (fa.priority_rank != fb.priority_rank) {
    const std::string first = fa.priority_rank > fb.priority_rank ? a : b;
    apply_order(
      d, first, "priority: " + first + " has the higher task priority", 15, false);
    return;
  }
  if (fa.loaded != fb.loaded) {
    const std::string first = fa.loaded ? a : b;
    apply_order(d, first, "load: " + first + " is loaded", 12, false);
    return;
  }
  if (std::abs(to_a - to_b) > cfg_.safety_distance_m) {
    const std::string first = to_a < to_b ? a : b;
    apply_order(
      d, first, "eta: " + first + " reaches the overlap first (" +
      f2(std::min(to_a, to_b)) + "m vs " + f2(std::max(to_a, to_b)) + "m)", 5, false);
    return;
  }
  if (A.reg_seq != B.reg_seq) {
    const std::string first = A.reg_seq < B.reg_seq ? a : b;
    apply_order(d, first, "fifo: " + first + " registered its route first", 2, false);
    return;
  }
  apply_order(d, a, "tie: robot id order", 1, false);
}

void GlobalResolver::enforce_waiter_occupancy()
{
  // A waiter holding inside another overlap occupies it: it must pass that
  // overlap first (it is already there).
  for (int pass = 0; pass < 4; ++pass) {
    bool changed = false;
    for (auto & kv : decisions_) {
      const auto & d = kv.second;
      if (!d.relaxed_wait || d.cleared || d.unresolvable) {
        continue;
      }
      for (auto & ev : decisions_) {
        auto & e = ev.second;
        if (ev.first == kv.first || !e.involves(d.second) || e.cleared || e.unresolvable ||
          e.other(d.second) == d.first)
        {
          continue;
        }
        const auto & pe = e.of(d.second);
        if (d.wait_s < pe.entry_s - cfg_.conflict_distance_m ||
          d.wait_s > pe.exit_s + cfg_.conflict_distance_m)
        {
          continue;
        }
        if (e.first == d.second) {
          continue;
        }
        if (e.forced) {
          report_unresolvable(e);
          continue;
        }
        apply_order(
          e, d.second,
          "waiter_occupies: " + d.second + " holds at its wait point inside this "
          "overlap (for " + kv.first + ") — it passes here first", 100, true);
        changed = true;
      }
    }
    if (!changed) {
      break;
    }
  }
}

std::string GlobalResolver::held_before(
  const std::string & amr, double entry_s, const std::string & skip_id) const
{
  for (const auto & kv : decisions_) {
    const auto & e = kv.second;
    if (kv.first == skip_id || e.second != amr || e.cleared || e.unresolvable) {
      continue;
    }
    if (e.wait_s <= entry_s + kEps) {
      return e.first;
    }
  }
  return {};
}

double GlobalResolver::hold_release_remaining(
  const std::string & amr, double entry_s, const std::string & skip_id) const
{
  double remaining = 0.0;
  for (const auto & kv : decisions_) {
    const auto & e = kv.second;
    if (kv.first == skip_id || e.second != amr || e.cleared || e.unresolvable ||
      e.wait_s > entry_s + kEps)
    {
      continue;
    }
    const auto it = routes_.find(e.first);
    const double progress =
      (it != routes_.end() && it->second.progress_valid) ? it->second.progress_s : 0.0;
    remaining = std::max(remaining, std::max(0.0, e.release_s - progress));
  }
  return remaining;
}

bool GlobalResolver::leader_in_section(const ResolverDecision & d) const
{
  if (d.kind != OverlapKind::SameDirection) {
    return false;
  }
  const auto it = routes_.find(d.first);
  if (it == routes_.end() || !it->second.progress_valid) {
    return true;
  }
  return it->second.progress_s + kEps < d.of(d.first).exit_s;
}

std::vector<std::string> GlobalResolver::find_cycle() const
{
  // Passage dependency graph. Node (conflict, robot) = "robot completes its
  // passage of that overlap". Edge u→v: v cannot complete before u.
  //  - wait: first completes d → waiter's FIRST passage after its wait point
  //    (the hold may sit before an earlier overlap of the waiter)
  //  - same direction: leader → follower until the leader left the section
  //  - route order: each robot completes its passages in entry order
  std::map<std::string, std::vector<std::pair<std::string, std::string>>> adj;
  auto node = [](const std::string & cid, const std::string & amr) {
      return cid + "||" + amr;
    };
  struct Passage
  {
    double entry;
    double exit;
    std::string cid;
  };
  std::map<std::string, std::vector<Passage>> per_robot;
  auto active = [&](const ResolverDecision & d) {
      return !d.unresolvable && (!d.cleared || leader_in_section(d));
    };
  for (const auto & kv : decisions_) {
    const auto & d = kv.second;
    if (!active(d)) {
      continue;
    }
    for (const auto & p : d.p) {
      per_robot[p.amr_id].push_back({p.entry_s, p.exit_s, kv.first});
      adj[node(kv.first, p.amr_id)];
    }
  }
  for (auto & kv : per_robot) {
    std::sort(kv.second.begin(), kv.second.end(), [](const Passage & l, const Passage & r) {
        return l.entry < r.entry;
      });
  }
  for (const auto & kv : decisions_) {
    const auto & d = kv.second;
    if (!active(d)) {
      continue;
    }
    if (d.cleared) {
      adj[node(kv.first, d.first)].push_back({node(kv.first, d.second), kv.first});
      continue;
    }
    std::string target = kv.first;
    for (const auto & p : per_robot[d.second]) {
      if (p.exit > d.wait_s + kEps) {
        target = p.cid;
        break;
      }
    }
    adj[node(kv.first, d.first)].push_back({node(target, d.second), kv.first});
  }
  for (const auto & kv : per_robot) {
    const auto & v = kv.second;
    for (std::size_t i = 0; i + 1 < v.size(); ++i) {
      adj[node(v[i].cid, kv.first)].push_back({node(v[i + 1].cid, kv.first), ""});
    }
  }
  std::map<std::string, int> color;  // 0 white 1 grey 2 black
  std::map<std::string, std::pair<std::string, std::string>> parent;  // node → (prev, edge)
  for (const auto & start : adj) {
    if (color[start.first] != 0) {
      continue;
    }
    std::vector<std::pair<std::string, std::size_t>> stack{{start.first, 0}};
    color[start.first] = 1;
    while (!stack.empty()) {
      auto & [u, i] = stack.back();
      const auto & out = adj[u];
      if (i >= out.size()) {
        color[u] = 2;
        stack.pop_back();
        continue;
      }
      const auto [v, edge] = out[i++];
      if (color[v] == 0) {
        color[v] = 1;
        parent[v] = {u, edge};
        stack.push_back({v, 0});
      } else if (color[v] == 1) {
        std::vector<std::string> ids;
        if (!edge.empty()) {
          ids.push_back(edge);
        }
        std::string cur = u;
        while (cur != v) {
          const auto pit = parent.find(cur);
          if (pit == parent.end()) {
            break;
          }
          if (!pit->second.second.empty()) {
            ids.push_back(pit->second.second);
          }
          cur = pit->second.first;
        }
        return ids;
      }
    }
  }
  return {};
}

void GlobalResolver::break_cycles()
{
  std::set<std::string> flipped;
  const std::size_t max_iter = 2 * decisions_.size() + 2;
  for (std::size_t iter = 0; iter < max_iter; ++iter) {
    const auto cyc = find_cycle();
    if (cyc.empty()) {
      return;
    }
    std::vector<std::string> cand;
    for (const auto & id : cyc) {
      const auto & d = decisions_.at(id);
      if (!d.forced && !d.cleared && !flipped.count(id)) {
        cand.push_back(id);
      }
    }
    std::sort(cand.begin(), cand.end(), [&](const std::string & l, const std::string & r) {
        const auto & dl = decisions_.at(l);
        const auto & dr = decisions_.at(r);
        if (dl.strength != dr.strength) {
          return dl.strength < dr.strength;
        }
        return dl.decided_seq > dr.decided_seq;
      });
    bool done = false;
    for (const auto & id : cand) {
      auto & d = decisions_.at(id);
      const Route & new_waiter = routes_.at(d.first);
      if (!find_wait(new_waiter, d)) {
        continue;
      }
      std::string chain;
      for (const auto & c : cyc) {
        chain += (chain.empty() ? "" : " -> ") + c;
      }
      const std::string old_rule = d.rule;
      apply_order(
        d, d.second,
        "deadlock_avoidance: order flipped to break dependency cycle [" + chain +
        "] (was " + old_rule + ")", d.strength, false);
      flipped.insert(id);
      log("[ECBS-GROUP] flipped " + id + " → " + d.first + " first, " + d.second + " waits");
      mark_changed(d.first);
      mark_changed(d.second);
      done = true;
      break;
    }
    if (!done) {
      for (const auto & id : cyc) {
        auto & d = decisions_.at(id);
        if (!d.cleared) {
          report_unresolvable(d);
        }
      }
      return;
    }
  }
}

std::vector<std::string> GlobalResolver::retry_unresolvable()
{
  std::vector<std::string> recovered;
  for (auto & kv : decisions_) {
    if (!kv.second.unresolvable) {
      continue;
    }
    ResolverDecision d = kv.second;
    d.unresolvable = false;
    d.forced = false;
    decide(d);  // report_unresolvable stays quiet: already reported
    if (!d.unresolvable) {
      log(
        "[ECBS-RECOVER] conflict=" + kv.first + " resolvable again → FIRST=" + d.first +
        ", " + d.second + " holds at wp" + std::to_string(d.wait_index) + " | rule=" + d.rule);
      kv.second = d;
      replan_reported_.erase(kv.first);
      mark_changed(d.first);
      mark_changed(d.second);
      recovered.push_back(kv.first);
    }
  }
  if (!recovered.empty()) {
    enforce_waiter_occupancy();
    break_cycles();
    refine_waits();
  }
  return recovered;
}

bool GlobalResolver::refine_waits(const std::set<std::string> & quiet_ids)
{
  bool any = false;
  for (int pass = 0; pass < 3; ++pass) {
    bool changed = false;
    for (auto & kv : decisions_) {
      auto & d = kv.second;
      if (d.cleared || d.unresolvable || d.first.empty()) {
        continue;
      }
      const auto sit = routes_.find(d.second);
      if (sit == routes_.end()) {
        continue;
      }
      const auto w = find_wait(sit->second, d);
      if (!w || w->index == d.wait_index) {
        continue;
      }
      const Route & S = sit->second;
      const int32_t old = d.wait_index;
      d.wait_index = w->index;
      d.wait_s = w->s;
      d.wait_x = S.x[static_cast<std::size_t>(w->index)];
      d.wait_y = S.y[static_cast<std::size_t>(w->index)];
      d.relaxed_wait = w->tier == 0;
      if (!quiet_ids.count(kv.first)) {
        const auto & ps = d.of(d.second);
        log(
          "[ECBS-WAIT] conflict=" + kv.first + " " + d.second + " now holds at wp" +
          std::to_string(d.wait_index) + " " + xy(d.wait_x, d.wait_y) + " (was wp" +
          std::to_string(old) + ") — nearest safe waypoint before its overlap wp" +
          std::to_string(ps.entry_index) + "..wp" + std::to_string(ps.exit_index) +
          ", still waits for " + d.first);
      }
      mark_changed(d.second);
      changed = true;
    }
    if (!changed) {
      break;
    }
    any = true;
  }
  return any;
}

std::string GlobalResolver::loser_by_priority(const std::string & a, const std::string & b) const
{
  const ResolverFacts fa = facts_.count(a) ? facts_.at(a) : ResolverFacts{};
  const ResolverFacts fb = facts_.count(b) ? facts_.at(b) : ResolverFacts{};
  if (fa.healthy != fb.healthy) {
    return fa.healthy ? a : b;  // zone_manager: replan the healthy robot
  }
  if (fa.priority_rank != fb.priority_rank) {
    return fa.priority_rank < fb.priority_rank ? a : b;
  }
  if (fa.loaded != fb.loaded) {
    return fa.loaded ? b : a;
  }
  const auto ra = routes_.find(a);
  const auto rb = routes_.find(b);
  if (ra != routes_.end() && rb != routes_.end() && ra->second.reg_seq != rb->second.reg_seq) {
    return ra->second.reg_seq > rb->second.reg_seq ? a : b;
  }
  return a < b ? b : a;
}

void GlobalResolver::report_unresolvable(ResolverDecision & d)
{
  d.unresolvable = true;
  if (replan_reported_[d.conflict_id]) {
    return;
  }
  replan_reported_[d.conflict_id] = true;
  ResolverReplan rp;
  rp.conflict_id = d.conflict_id;
  rp.robots = {d.p[0].amr_id, d.p[1].amr_id};
  rp.candidate = loser_by_priority(d.p[0].amr_id, d.p[1].amr_id);
  rp.reason = "no deadlock-free passage order by waiting (" +
    std::string(overlap_kind_name(d.kind)) + ") — replan " + rp.candidate;
  replans_.push_back(rp);
  log("[ECBS-UNRESOLVABLE] conflict=" + d.conflict_id + " robots=" + d.p[0].amr_id + "," +
    d.p[1].amr_id + " → both hold; replan candidate " + rp.candidate);
  mark_changed(d.p[0].amr_id);
  mark_changed(d.p[1].amr_id);
}

// ---------------------------------------------------------------------------
// Progress / clearance
// ---------------------------------------------------------------------------

std::vector<ResolverEvent> GlobalResolver::update_pose(
  const std::string & amr_id, double x, double y)
{
  std::vector<ResolverEvent> events;
  auto it = routes_.find(amr_id);
  if (it == routes_.end()) {
    set_standing(amr_id, x, y);
    return events;
  }
  Route & r = it->second;
  r.pose_x = x;
  r.pose_y = y;
  r.pose_valid = true;
  if (r.standing()) {
    if (std::hypot(r.x.front() - x, r.y.front() - y) > 0.3) {
      r.x.front() = x;
      r.y.front() = y;
      build_geometry(r, cfg_.sample_step_m);
    }
    return events;
  }
  const double lo = r.progress_valid ? r.progress_s - 1.0 : 0.0;
  double hi = r.granted >= 0 ?
    r.s[static_cast<std::size_t>(r.granted)] + 1.0 :
    (r.progress_valid ? r.progress_s + 3.0 : 5.0);
  hi = std::max(hi, lo + 1.0);
  double best = kInf;
  double best_s = r.progress_s;
  for (const auto & smp : r.samples) {
    if (smp.s < lo - kEps) {
      continue;
    }
    if (smp.s > hi + kEps) {
      break;
    }
    const double d = std::hypot(smp.x - x, smp.y - y);
    if (d < best) {
      best = d;
      best_s = smp.s;
    }
  }
  if (best > cfg_.max_offroute_m) {
    return events;
  }
  r.progress_s = r.progress_valid ? std::max(r.progress_s, best_s) : best_s;
  r.progress_valid = true;

  // Clearance: first robot passed its release point → waiter released.
  for (auto & kv : decisions_) {
    auto & d = kv.second;
    if (d.first != amr_id || d.cleared || d.unresolvable) {
      continue;
    }
    const bool at_end = d.release_s >= r.length - kEps &&
      r.progress_s >= r.length - cfg_.body_back_m;
    if (r.progress_s + kEps >= d.release_s || at_end) {
      d.cleared = true;
      const std::string text = "[ECBS-CLEAR] " + amr_id + " cleared conflict=" + kv.first +
        " (passed " + xy(d.release_x, d.release_y) + ") → " + d.second + " released from wp" +
        std::to_string(d.wait_index);
      log(text);
      events.push_back({d.second, amr_id, kv.first, text});
      mark_changed(d.second);
      mark_changed(amr_id);
    }
  }
  // Retire decisions both robots are done with.
  for (auto dit = decisions_.begin(); dit != decisions_.end(); ) {
    const auto & d = dit->second;
    bool retire = false;
    if (d.cleared && d.second == amr_id) {
      const auto & ps = d.of(amr_id);
      retire = r.progress_s >= std::min(ps.exit_s + cfg_.safety_distance_m, r.length) -
        kEps;
    }
    if (retire) {
      dit = decisions_.erase(dit);
    } else {
      ++dit;
    }
  }
  return events;
}

// ---------------------------------------------------------------------------
// Release
// ---------------------------------------------------------------------------

GlobalResolver::Cap GlobalResolver::compute_cap(
  const Route & r, int32_t tip, int32_t target) const
{
  Cap cap;
  cap.index = target;
  const double cd = cfg_.conflict_distance_m;
  // 1) Decided waits / unresolvable holds.
  std::vector<std::pair<int32_t, WaitDependency>> deps;
  for (const auto & kv : decisions_) {
    const auto & d = kv.second;
    if (!d.involves(r.amr_id) || d.cleared) {
      continue;
    }
    if (d.unresolvable) {
      cap.recovery = true;
      WaitDependency dep;
      dep.amr_id = d.other(r.amr_id);
      dep.topology = topology_for(d.kind);
      const int32_t hold = std::max<int32_t>(tip, 0);
      dep.wait_point.x = r.x[static_cast<std::size_t>(hold)];
      dep.wait_point.y = r.y[static_cast<std::size_t>(hold)];
      dep.wait_point.index = static_cast<uint64_t>(hold);
      deps.push_back({hold, dep});
      cap.conflict_ids.push_back(kv.first);
      cap.index = std::min(cap.index, std::max<int32_t>(tip, 0));
      cap.reason = "unresolvable conflict " + kv.first + " — replan required";
      continue;
    }
    if (d.second != r.amr_id) {
      continue;
    }
    WaitDependency dep;
    dep.amr_id = d.first;
    dep.topology = topology_for(d.kind);
    dep.wait_point.x = d.wait_x;
    dep.wait_point.y = d.wait_y;
    dep.wait_point.index = static_cast<uint64_t>(std::max(0, d.wait_index));
    dep.clear_points.push_back({d.release_x, d.release_y});
    dep.other_blocks_self = true;
    deps.push_back({d.wait_index, dep});
    cap.conflict_ids.push_back(kv.first);
    if (d.wait_index < cap.index) {
      cap.index = std::max(d.wait_index, tip);
      cap.reason = "hold at wp" + std::to_string(d.wait_index) + " for " + d.first +
        " (" + overlap_kind_name(d.kind) + ", " + d.rule + ")";
    }
  }

  // 2) No foreign base: never come within cd of another robot's occupancy.
  const double base_s = std::max(
    r.progress_valid ? r.progress_s : 0.0,
    tip >= 0 ? r.s[static_cast<std::size_t>(tip)] : 0.0);
  const double px = r.pose_valid ? r.pose_x : point_at(r, base_s).x;
  const double py = r.pose_valid ? r.pose_y : point_at(r, base_s).y;
  for (const auto & kv : routes_) {
    if (kv.first == r.amr_id) {
      continue;
    }
    const Route & o = kv.second;
    std::vector<std::pair<double, double>> occ;
    if (o.standing()) {
      occ.push_back({o.x.front(), o.y.front()});
    } else {
      const double from = remaining_from(o);
      const double to = std::max(committed_s(o), from + cfg_.body_back_m);
      for (const auto & smp : o.samples) {
        if (smp.s + kEps < from) {
          continue;
        }
        if (smp.s > to + kEps) {
          break;
        }
        occ.push_back({smp.x, smp.y});
      }
      if (o.pose_valid) {
        occ.push_back({o.pose_x, o.pose_y});
      }
    }
    if (occ.empty()) {
      continue;
    }
    auto dist_to = [&](double x, double y) {
        double best = kInf;
        for (const auto & p : occ) {
          best = std::min(best, std::hypot(p.first - x, p.second - y));
        }
        return best;
      };
    const double d0 = dist_to(px, py);
    const double limit_s = r.s[static_cast<std::size_t>(cap.index)];
    double hit_s = -1.0;
    for (const auto & smp : r.samples) {
      if (smp.s <= base_s + kEps) {
        continue;
      }
      if (smp.s > limit_s + kEps) {
        break;
      }
      const double dd = dist_to(smp.x, smp.y);
      if (dd < cd - kEps && dd < d0 - 0.05) {
        hit_s = smp.s;
        break;
      }
    }
    if (hit_s < 0.0) {
      continue;
    }
    const int32_t g = std::max(
      {tip, 0, index_at_or_before(r, hit_s - cfg_.safety_distance_m)});
    WaitDependency dep;
    dep.amr_id = o.amr_id;
    dep.topology = o.standing() || parked(o) ? kTopoOwnership : kTopoSameWay;
    const int32_t gi = std::max(0, g);
    dep.wait_point.x = r.x[static_cast<std::size_t>(gi)];
    dep.wait_point.y = r.y[static_cast<std::size_t>(gi)];
    dep.wait_point.index = static_cast<uint64_t>(gi);
    dep.other_blocks_self = true;
    deps.push_back({g, dep});
    if (g < cap.index) {
      cap.index = g;
      cap.reason = "occupied: " + o.amr_id +
        (o.standing() || parked(o) ? " is standing on the route" : "'s base is ahead") +
        " — hold " + f2(cfg_.safety_distance_m) + "m before it";
    }
  }
  cap.index = std::max(cap.index, tip);
  // Binding constraint first, then the rest that apply within the target.
  std::stable_sort(deps.begin(), deps.end(), [](const auto & l, const auto & rr) {
      return l.first < rr.first;
    });
  for (const auto & dv : deps) {
    if (dv.first <= target) {
      cap.deps.push_back(dv.second);
    }
  }
  return cap;
}

ResolverAnswer GlobalResolver::query(
  const std::string & amr_id, int32_t current_tip, double desired_m,
  const std::string & base_key)
{
  ResolverAnswer out;
  const auto pit = pending_intents_.find(amr_id);
  if (pit != pending_intents_.end() && (base_key.empty() || pit->second.base_key == base_key)) {
    out.status = ResolverStatus::Wait;
    out.through_index = std::max(current_tip, 0);
    out.from_index = out.through_index;
    out.reason = "awaiting RMF handover evaluation for [" + pit->second.base_key + "]";
    return out;
  }
  const auto it = routes_.find(amr_id);
  if (it == routes_.end()) {
    out.status = ResolverStatus::UnknownRoute;
    out.reason = "route not registered — TM must seed PathIntent";
    return out;
  }
  Route & r = it->second;
  if (!base_key.empty() && !r.base_key.empty() && r.base_key != base_key &&
    !r.standing())
  {
    // RMF already handed over a newer route; the old TM session must not
    // extend on it. Hold at the issued tip until the new PathIntent binds.
    out.status = ResolverStatus::Wait;
    out.through_index = std::max(current_tip, 0);
    out.from_index = out.through_index;
    out.reason = "route superseded by RMF handover [" + r.base_key +
      "] — holding until its PathIntent binds";
    return out;
  }
  if (r.standing()) {
    out.status = ResolverStatus::RouteComplete;
    out.through_index = 0;
    out.reason = "standing / single-waypoint route — nothing to release";
    return out;
  }
  const int32_t last = static_cast<int32_t>(r.x.size()) - 1;
  // TM's committed tip is authoritative for what this session was issued.
  r.granted = std::max(-1, std::min(current_tip, last));
  const int32_t tip = r.granted;
  if (tip >= last) {
    out.status = ResolverStatus::RouteComplete;
    out.through_index = last;
    out.reason = "whole route already released";
    return out;
  }
  const double prog = r.progress_valid ? r.progress_s : 0.0;
  const int32_t start_index = std::max(0, index_at_or_before(r, prog));
  const int32_t base_index = std::max(tip, start_index);
  const double base_s = std::max(r.s[static_cast<std::size_t>(base_index)], prog);
  const double target_s = std::min(r.length, base_s + std::max(desired_m, 0.5));
  int32_t target = index_at_or_before(r, target_s);
  target = std::max(target, base_index + 1);  // always offer the next waypoint
  target = std::min(target, last);

  const auto cap = compute_cap(r, tip, target);
  const int32_t from = std::max(tip, 0);
  out.from_index = from;
  out.from_x = r.x[static_cast<std::size_t>(from)];
  out.from_y = r.y[static_cast<std::size_t>(from)];
  out.conflict_ids = cap.conflict_ids;

  if (cap.recovery) {
    out.status = ResolverStatus::Recovery;
    out.through_index = std::max(tip, 0);
    out.waiting_for = cap.deps;
    out.reason = cap.reason;
    return out;
  }
  const bool advances = cap.index > tip &&
    !(tip < 0 && cap.index <= start_index && !cap.deps.empty());
  if (!advances) {
    out.status = ResolverStatus::Wait;
    const int32_t hold = std::max({cap.index, tip, 0});
    out.through_index = std::max(tip, 0);
    out.waiting_for = cap.deps;
    for (auto & dep : out.waiting_for) {
      dep.wait_point.index = static_cast<uint64_t>(hold);
      dep.wait_point.x = r.x[static_cast<std::size_t>(hold)];
      dep.wait_point.y = r.y[static_cast<std::size_t>(hold)];
    }
    out.reason = cap.reason.empty() ? "no advance possible" : cap.reason;
    return out;
  }
  out.status = ResolverStatus::Free;
  out.through_index = cap.index;
  out.through_x = r.x[static_cast<std::size_t>(cap.index)];
  out.through_y = r.y[static_cast<std::size_t>(cap.index)];
  out.proposed_m = r.s[static_cast<std::size_t>(cap.index)] - r.s[static_cast<std::size_t>(from)];
  out.proposal_id = next_proposal_id_++;
  out.reason = cap.index < target ? "approach — " + cap.reason : "clear";
  r.pending_proposal = out.proposal_id;
  r.pending_to = cap.index;
  return out;
}

std::string GlobalResolver::commit(
  const std::string & amr_id, uint64_t proposal_id, int32_t through_index,
  const std::string & status)
{
  const auto it = routes_.find(amr_id);
  if (it == routes_.end()) {
    return "REJECTED_MISMATCH";
  }
  Route & r = it->second;
  if (status != "COMMITTED") {
    if (r.pending_proposal == proposal_id) {
      r.pending_proposal = 0;
      r.pending_to = -1;
    }
    return "ACCEPTED";
  }
  if (through_index <= r.granted) {
    return "DUPLICATE";
  }
  if (proposal_id != r.pending_proposal || through_index > r.pending_to) {
    return "REJECTED_MISMATCH";
  }
  // Re-validate against decisions made since the proposal (no foreign base).
  const auto cap = compute_cap(r, r.granted, through_index);
  if (cap.recovery || cap.index < through_index) {
    log(
      "[ECBS-COMMIT] " + amr_id + " proposal " + std::to_string(proposal_id) +
      " through wp" + std::to_string(through_index) + " rejected as stale (" + cap.reason + ")");
    r.pending_proposal = 0;
    r.pending_to = -1;
    return "REJECTED_STALE";
  }
  r.granted = through_index;
  r.pending_proposal = 0;
  r.pending_to = -1;
  return "ACCEPTED";
}

// ---------------------------------------------------------------------------
// Introspection
// ---------------------------------------------------------------------------

std::vector<ResolverDecision> GlobalResolver::decisions() const
{
  std::vector<ResolverDecision> out;
  for (const auto & kv : decisions_) {
    out.push_back(kv.second);
  }
  return out;
}

std::vector<ResolverDecision> GlobalResolver::decisions_for(const std::string & amr_id) const
{
  std::vector<ResolverDecision> out;
  for (const auto & kv : decisions_) {
    if (kv.second.involves(amr_id)) {
      out.push_back(kv.second);
    }
  }
  return out;
}

int32_t GlobalResolver::granted_index(const std::string & amr_id) const
{
  const auto it = routes_.find(amr_id);
  return it == routes_.end() ? -1 : it->second.granted;
}

double GlobalResolver::progress_s(const std::string & amr_id) const
{
  const auto it = routes_.find(amr_id);
  return it == routes_.end() ? 0.0 : it->second.progress_s;
}

std::optional<ResolverRoutePoint> GlobalResolver::waypoint(
  const std::string & amr_id, int32_t index) const
{
  const auto it = routes_.find(amr_id);
  if (it == routes_.end() || index < 0 ||
    static_cast<std::size_t>(index) >= it->second.x.size())
  {
    return std::nullopt;
  }
  return ResolverRoutePoint{
    it->second.x[static_cast<std::size_t>(index)],
    it->second.y[static_cast<std::size_t>(index)]};
}

std::vector<ResolverRoutePoint> GlobalResolver::route_points(const std::string & amr_id) const
{
  std::vector<ResolverRoutePoint> out;
  const auto it = routes_.find(amr_id);
  if (it == routes_.end()) {
    return out;
  }
  for (std::size_t i = 0; i < it->second.x.size(); ++i) {
    out.push_back({it->second.x[i], it->second.y[i]});
  }
  return out;
}

void GlobalResolver::log(const std::string & line)
{
  log_.push_back(line);
}

void GlobalResolver::mark_changed(const std::string & amr_id)
{
  if (std::find(changed_.begin(), changed_.end(), amr_id) == changed_.end()) {
    changed_.push_back(amr_id);
  }
}

std::vector<std::string> GlobalResolver::take_log()
{
  std::vector<std::string> out;
  out.swap(log_);
  return out;
}

std::vector<ResolverReplan> GlobalResolver::take_replans()
{
  std::vector<ResolverReplan> out;
  out.swap(replans_);
  return out;
}

std::vector<std::string> GlobalResolver::take_changed()
{
  std::vector<std::string> out;
  out.swap(changed_);
  return out;
}

}  // namespace ecbs_planner
