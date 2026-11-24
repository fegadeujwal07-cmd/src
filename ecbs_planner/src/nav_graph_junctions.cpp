#include "ecbs_planner/nav_graph_junctions.hpp"

#include <cmath>
#include <limits>
#include <unordered_set>

#include <yaml-cpp/yaml.h>

#include "ecbs_planner/path_utils.hpp"

namespace ecbs_planner
{
namespace
{

constexpr std::size_t kNpos = static_cast<std::size_t>(-1);

/// Match radius when scanning path waypoints for a T/cross centre.
constexpr double kCentreOnPathM = 0.75;

}  // namespace

std::uint64_t NavGraphJunctions::cell_key(double x, double y)
{
  const auto ix = static_cast<std::int32_t>(std::floor(x / kCellM));
  const auto iy = static_cast<std::int32_t>(std::floor(y / kCellM));
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(ix)) << 32) |
         static_cast<std::uint64_t>(static_cast<std::uint32_t>(iy));
}

void NavGraphJunctions::rebuild_hash_locked()
{
  hash_.clear();
  hash_.reserve(junctions_.size() * 2);
  for (std::size_t i = 0; i < junctions_.size(); ++i) {
    hash_[cell_key(junctions_[i].x, junctions_[i].y)].push_back(i);
  }
}

void NavGraphJunctions::set_from_edges(
  const std::vector<std::pair<double, double>> & vertices,
  const std::vector<std::pair<std::size_t, std::size_t>> & edges)
{
  std::vector<std::unordered_set<std::size_t>> nbr(vertices.size());
  for (const auto & e : edges) {
    if (e.first >= vertices.size() || e.second >= vertices.size()) {
      continue;
    }
    if (e.first == e.second) {
      continue;
    }
    nbr[e.first].insert(e.second);
    nbr[e.second].insert(e.first);
  }

  std::vector<JunctionPoint> next;
  next.reserve(vertices.size() / 8 + 4);
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    const std::size_t deg = nbr[i].size();
    if (deg < 3) {
      continue;
    }
    JunctionPoint jp;
    jp.x = vertices[i].first;
    jp.y = vertices[i].second;
    jp.kind = (deg >= 4) ? JunctionKind::Cross : JunctionKind::TSection;
    next.push_back(jp);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  vertices_ = vertices;
  edges_ = edges;
  junctions_ = std::move(next);
  rebuild_hash_locked();
}

bool NavGraphJunctions::load_from_yaml(
  const std::string & yaml_path, const std::string & level)
{
  if (yaml_path.empty() || level.empty()) {
    return false;
  }
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const std::exception &) {
    return false;
  }
  const YAML::Node level_node = root["levels"][level];
  if (!level_node || !level_node["vertices"]) {
    return false;
  }

  std::vector<std::pair<double, double>> verts;
  for (const auto & vertex_node : level_node["vertices"]) {
    if (!vertex_node.IsSequence() || vertex_node.size() < 2) {
      continue;
    }
    try {
      verts.emplace_back(vertex_node[0].as<double>(), vertex_node[1].as<double>());
    } catch (...) {
      continue;
    }
  }
  if (verts.empty()) {
    return false;
  }

  std::vector<std::pair<std::size_t, std::size_t>> eds;
  if (level_node["lanes"]) {
    for (const auto & lane_node : level_node["lanes"]) {
      if (!lane_node.IsSequence() || lane_node.size() < 2) {
        continue;
      }
      try {
        const auto a = static_cast<std::size_t>(lane_node[0].as<int>());
        const auto b = static_cast<std::size_t>(lane_node[1].as<int>());
        eds.emplace_back(a, b);
      } catch (...) {
        continue;
      }
    }
  }
  set_from_edges(verts, eds);
  return loaded();
}

bool NavGraphJunctions::loaded() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return !vertices_.empty();
}

bool NavGraphJunctions::empty() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return junctions_.empty();
}

std::size_t NavGraphJunctions::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return junctions_.size();
}

std::size_t NavGraphJunctions::vertex_count() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return vertices_.size();
}

std::size_t NavGraphJunctions::edge_count() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return edges_.size();
}

std::vector<JunctionPoint> NavGraphJunctions::junctions() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return junctions_;
}

std::optional<JunctionPoint> NavGraphJunctions::find_near(
  double x, double y, double match_m) const
{
  if (junctions_.empty() || match_m < 0.0) {
    return std::nullopt;
  }
  const double m2 = match_m * match_m;
  const int span = std::max(1, static_cast<int>(std::ceil(match_m / kCellM)) + 1);
  const auto cx = static_cast<std::int32_t>(std::floor(x / kCellM));
  const auto cy = static_cast<std::int32_t>(std::floor(y / kCellM));

  std::optional<JunctionPoint> best;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (int dx = -span; dx <= span; ++dx) {
    for (int dy = -span; dy <= span; ++dy) {
      const auto key =
        (static_cast<std::uint64_t>(
           static_cast<std::uint32_t>(cx + dx)) <<
        32) |
        static_cast<std::uint64_t>(static_cast<std::uint32_t>(cy + dy));
      const auto it = hash_.find(key);
      if (it == hash_.end()) {
        continue;
      }
      for (const std::size_t ji : it->second) {
        const auto & j = junctions_[ji];
        const double ddx = j.x - x;
        const double ddy = j.y - y;
        const double d2 = ddx * ddx + ddy * ddy;
        if (d2 <= m2 && d2 < best_d2) {
          best_d2 = d2;
          best = j;
        }
      }
    }
  }
  return best;
}

bool NavGraphJunctions::near_junction(double x, double y, double match_m) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return find_near(x, y, match_m).has_value();
}

std::optional<JunctionKind> NavGraphJunctions::kind_near(
  double x, double y, double match_m) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (auto j = find_near(x, y, match_m)) {
    return j->kind;
  }
  return std::nullopt;
}

std::size_t NavGraphJunctions::junction_anchor_on_path(
  const TimedPath & path,
  std::size_t wait_i,
  double wait_x,
  double wait_y,
  std::size_t min_i,
  double safety_m) const
{
  if (junctions_.empty() || path.waypoints.empty() || safety_m < 0.0) {
    return kNpos;
  }
  const std::size_t n = path.waypoints.size();
  wait_i = std::min(wait_i, n - 1);
  min_i = std::min(min_i, wait_i);

  // Hard rule uses the Euclidean safety disc around the T/cross centre only
  // (not segment proximity — that would pull already-safe waits back in).
  auto near = find_near(wait_x, wait_y, safety_m);
  if (!near) {
    near = find_near(
      path.waypoints[wait_i].x, path.waypoints[wait_i].y, safety_m);
  }
  if (!near) {
    return kNpos;
  }

  // Anchor = path waypoint closest to that junction centre (search whole path
  // from min_i so we back up from the true centre, not a nearby wait).
  std::size_t best = min_i;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (std::size_t i = min_i; i < n; ++i) {
    const double dx = path.waypoints[i].x - near->x;
    const double dy = path.waypoints[i].y - near->y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best = i;
    }
  }
  return best;
}

std::pair<std::size_t, ClearPoint> NavGraphJunctions::point_before_junction(
  const TimedPath & path,
  std::size_t junction_i,
  std::size_t min_i,
  double safety_m) const
{
  const std::size_t n = path.waypoints.size();
  junction_i = std::min(junction_i, n - 1);
  min_i = std::min(min_i, junction_i);
  const std::size_t at = safety_wait_index(path, junction_i, min_i, safety_m);
  ClearPoint p;
  p.x = path.waypoints[at].x;
  p.y = path.waypoints[at].y;
  return {at, p};
}

std::size_t NavGraphJunctions::snap_wait_off_junction(
  const TimedPath & path,
  std::size_t wait_i,
  std::size_t min_i,
  double safety_m) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (junctions_.empty() || path.waypoints.empty()) {
    return wait_i;
  }
  const std::size_t n = path.waypoints.size();
  wait_i = std::min(wait_i, n - 1);
  min_i = std::min(min_i, wait_i);
  const std::size_t ji = junction_anchor_on_path(
    path, wait_i, path.waypoints[wait_i].x, path.waypoints[wait_i].y,
    min_i, safety_m);
  if (ji == kNpos) {
    return wait_i;
  }
  // Already ≥ safety_m before the centre along the path → leave it.
  if (path_length_between(path, wait_i, ji) + 1e-9 >= safety_m && wait_i < ji) {
    return wait_i;
  }
  return point_before_junction(path, ji, min_i, safety_m).first;
}

std::optional<std::pair<std::size_t, ClearPoint>>
NavGraphJunctions::snap_wait_point_off_junction(
  const TimedPath & path,
  std::size_t wait_i,
  double wait_x,
  double wait_y,
  std::size_t min_i,
  double safety_m) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (junctions_.empty() || path.waypoints.empty()) {
    return std::nullopt;
  }
  const std::size_t n = path.waypoints.size();
  wait_i = std::min(wait_i, n - 1);
  min_i = std::min(min_i, wait_i);
  const std::size_t ji = junction_anchor_on_path(
    path, wait_i, wait_x, wait_y, min_i, safety_m);
  if (ji == kNpos) {
    return std::nullopt;
  }
  // Already a safe approach stop (≥ safety_m before, along path or Euclidean).
  const double dx = wait_x - path.waypoints[ji].x;
  const double dy = wait_y - path.waypoints[ji].y;
  if (wait_i < ji &&
    (path_length_between(path, wait_i, ji) + 1e-9 >= safety_m ||
    std::hypot(dx, dy) + 1e-9 >= safety_m))
  {
    return std::nullopt;
  }
  return point_before_junction(path, ji, min_i, safety_m);
}

std::optional<JunctionApproach> NavGraphJunctions::approach_near_base_end(
  const TimedPath & path,
  std::size_t start,
  std::size_t base_hi,
  double safety_m) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (junctions_.empty() || path.waypoints.empty() || safety_m < 0.0) {
    return std::nullopt;
  }
  const std::size_t n = path.waypoints.size();
  start = std::min(start, n - 1);
  base_hi = std::min(base_hi, n - 1);
  // Base reaching the path end: the robot parks at its goal, nothing to gate.
  if (base_hi <= start || base_hi + 1 >= n) {
    return std::nullopt;
  }
  const double end_m = path_length_between(path, start, base_hi);
  constexpr double kEps = 1e-9;

  double along_m = 0.0;
  double last_x = std::numeric_limits<double>::quiet_NaN();
  double last_y = last_x;
  for (std::size_t i = start; i < n; ++i) {
    if (i > start) {
      along_m += std::hypot(
        path.waypoints[i].x - path.waypoints[i - 1].x,
        path.waypoints[i].y - path.waypoints[i - 1].y);
    }
    if (along_m >= end_m + safety_m - kEps) {
      break;  // every later junction is far enough beyond the base end
    }
    const auto near = find_near(
      path.waypoints[i].x, path.waypoints[i].y, kCentreOnPathM);
    if (!near) {
      continue;
    }
    // Several waypoints may sit on one centre: judge it once (first waypoint).
    if (near->x == last_x && near->y == last_y) {
      continue;
    }
    last_x = near->x;
    last_y = near->y;
    // Inside the base by safety_m or more → full base, no gate.
    if (along_m <= end_m - safety_m + kEps) {
      continue;
    }
    const auto before = point_before_junction(path, i, start, safety_m);
    // The stop must still be ahead of the robot.
    if (before.first < start) {
      return std::nullopt;
    }
    JunctionApproach out;
    out.junction_i = i;
    out.kind = near->kind;
    out.wait_i = before.first;
    out.wait_point = before.second;
    return out;
  }
  return std::nullopt;
}

}  // namespace ecbs_planner

