// ZoneTracker: pose + planned path -> current zone, next zones, and zone sequence.
#include "monitor/zone_tracker.hpp"

#include <algorithm>     // min/max, find, sort helpers
#include <functional>    // std::greater for the rear-zone frontier
#include <optional>
#include <queue>         // priority_queue for the lane walk
#include <cmath>           // hypot, floor, ceil, isfinite
#include <cctype>          // tolower, isdigit for robot id normalization
#include <limits>          // numeric_limits for distance/area sentinel values
#include <regex>           // extract digits from robot names
#include <sstream>         // parse "x,y" planned path strings
#include <unordered_set>   // dedup zone ids during grid lookup

#include "rclcpp/rclcpp.hpp"

namespace monitor
{
namespace
{

// Return the ROS logger used by all ZoneTracker log statements.
rclcpp::Logger zone_tracker_logger()
{
  return rclcpp::get_logger("zone_tracker");
}

// Trim leading/trailing whitespace (matches Python str.strip()).
std::string trim_copy(const std::string & input)
{
  const auto start = input.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) {
    return "";
  }
  const auto end = input.find_last_not_of(" \t\r\n");
  return input.substr(start, end - start + 1);
}

}  // namespace

// Normalize fleet/monitor robot names to a consistent amr-prefixed lowercase id.
std::string ZoneTracker::normalize_robot_id(const std::string & robot_name)
{
  std::string name = robot_name;
  // Lowercase so amr001 and AMR001 match.
  std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
  name = trim_copy(name);
  if (name.empty()) {
    return "";
  }
  // Already normalized form from monitor topics.
  if (name.rfind("amr", 0) == 0) {
    return name;
  }

  // Pure numeric names like "001" become "amr001".
  bool is_digit = !name.empty();
  for (char c : name) {
    if (!std::isdigit(static_cast<unsigned char>(c))) {
      is_digit = false;
      break;
    }
  }
  if (is_digit) {
    return "amr" + name;
  }

  // Names like "robot_001" -> first digit run -> "amr001".
  std::regex re("(\\d+)");
  std::smatch match;
  if (std::regex_search(name, match, re) && match.size() > 1) {
    return "amr" + match.str(1);
  }
  return name;
}

// Return true when (px, py) lies inside the axis-aligned rectangle corners.
bool ZoneTracker::point_in_rectangle(
  double px,
  double py,
  const std::vector<std::pair<double, double>> & rect)
{
  if (rect.size() < 4) {
    return false;
  }
  // Compute bbox from corner list (not necessarily ordered).
  double min_x = rect[0].first;
  double max_x = rect[0].first;
  double min_y = rect[0].second;
  double max_y = rect[0].second;
  for (const auto & corner : rect) {
    min_x = std::min(min_x, corner.first);
    max_x = std::max(max_x, corner.first);
    min_y = std::min(min_y, corner.second);
    max_y = std::max(max_y, corner.second);
  }
  return px >= min_x && px <= max_x && py >= min_y && py <= max_y;
}

// Full scan: pick the smallest-area zone bbox that contains the point.
std::string ZoneTracker::find_zone_at_point(
  double px,
  double py,
  const std::unordered_map<std::string, std::vector<std::pair<double, double>>> & zone_rects)
{
  std::string best_id;
  double best_area = std::numeric_limits<double>::infinity();
  for (const auto & entry : zone_rects) {
    const auto & corners = entry.second;
    if (!point_in_rectangle(px, py, corners)) {
      continue;
    }
    // Recompute bbox to measure overlapping zone area.
    double min_x = corners[0].first;
    double max_x = corners[0].first;
    double min_y = corners[0].second;
    double max_y = corners[0].second;
    for (const auto & corner : corners) {
      min_x = std::min(min_x, corner.first);
      max_x = std::max(max_x, corner.first);
      min_y = std::min(min_y, corner.second);
      max_y = std::max(max_y, corner.second);
    }
    const double area = (max_x - min_x) * (max_y - min_y);
    if (area < best_area) {
      best_area = area;
      best_id = entry.first;
    }
  }
  return best_id;
}

// Build axis-aligned bounding boxes for each zone from its vertex index list.
void ZoneTracker::build_zone_rects(
  const std::vector<Vertex> & vertices,
  const std::unordered_map<std::string, std::vector<int>> & zone_vertex_indices,
  std::unordered_map<std::string, std::vector<std::pair<double, double>>> & zone_rects)
{
  zone_rects.clear();
  for (const auto & entry : zone_vertex_indices) {
    std::vector<double> xs;
    std::vector<double> ys;
    // Collect x/y from each waypoint/boundary vertex index.
    for (int idx : entry.second) {
      if (idx < 0 || idx >= static_cast<int>(vertices.size())) {
        continue;
      }
      xs.push_back(vertices[static_cast<std::size_t>(idx)].x);
      ys.push_back(vertices[static_cast<std::size_t>(idx)].y);
    }
    if (xs.empty() || ys.empty()) {
      continue;
    }
    const double min_x = *std::min_element(xs.begin(), xs.end());
    const double max_x = *std::max_element(xs.begin(), xs.end());
    const double min_y = *std::min_element(ys.begin(), ys.end());
    const double max_y = *std::max_element(ys.begin(), ys.end());
    // Store four corners of the bbox (axis-aligned rectangle).
    zone_rects[entry.first] = {
      {min_x, min_y},
      {max_x, min_y},
      {max_x, max_y},
      {min_x, max_y},
    };
  }
}

// Flatten vertices_ into parallel xy/label arrays for fast runtime lookups.
void ZoneTracker::rebuild_precomputed_helpers()
{
  vertices_xy_.clear();
  vertex_zone_labels_.clear();
  vertices_xy_.reserve(vertices_.size());
  vertex_zone_labels_.reserve(vertices_.size());
  for (const auto & vertex : vertices_) {
    vertices_xy_.emplace_back(vertex.x, vertex.y);
    vertex_zone_labels_.push_back(vertex.zone_labels);
  }
}

// Pack grid cell coordinates into a single hash key.
int64_t ZoneTracker::cell_key(int cx, int cy) const
{
  return (static_cast<int64_t>(cx) << 32) ^ static_cast<uint32_t>(cy);
}

// Convert world coordinates to grid cell indices.
bool ZoneTracker::world_to_cell(double x, double y, int & cx, int & cy) const
{
  if (!grid_.enabled || grid_.cell_size <= 0.0) {
    return false;
  }
  cx = static_cast<int>(std::floor((x - grid_.origin_x) / grid_.cell_size));
  cy = static_cast<int>(std::floor((y - grid_.origin_y) / grid_.cell_size));
  return true;
}

// Build uniform grid indexing zones and vertices for sub-linear lookups.
void ZoneTracker::build_spatial_index()
{
  grid_ = SpatialGrid{};
  if (vertices_xy_.empty() && zone_rects_.empty()) {
    return;
  }

  // Compute world bounds from all vertices and zone corners.
  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();

  for (const auto & v : vertices_xy_) {
    min_x = std::min(min_x, v.first);
    max_x = std::max(max_x, v.first);
    min_y = std::min(min_y, v.second);
    max_y = std::max(max_y, v.second);
  }
  for (const auto & entry : zone_rects_) {
    for (const auto & corner : entry.second) {
      min_x = std::min(min_x, corner.first);
      max_x = std::max(max_x, corner.first);
      min_y = std::min(min_y, corner.second);
      max_y = std::max(max_y, corner.second);
    }
  }
  if (!std::isfinite(min_x) || !std::isfinite(max_x)) {
    return;
  }

  // Pad slightly so edge points land inside the grid.
  const double pad = 1.0;
  min_x -= pad;
  min_y -= pad;
  max_x += pad;
  max_y += pad;

  const double span = std::max(max_x - min_x, max_y - min_y);
  // Target ~sqrt(N) cells on the longer axis for balanced buckets.
  const double target_cells = std::max(8.0, std::sqrt(static_cast<double>(
      std::max<std::size_t>(1, vertices_xy_.size() + zone_rects_.size()))));
  grid_.cell_size = std::max(1.0, span / target_cells);
  grid_.origin_x = min_x;
  grid_.origin_y = min_y;
  grid_.cols = std::max(1, static_cast<int>(std::ceil((max_x - min_x) / grid_.cell_size)));
  grid_.rows = std::max(1, static_cast<int>(std::ceil((max_y - min_y) / grid_.cell_size)));
  grid_.enabled = true;

  // Register each zone bbox in every grid cell it overlaps.
  for (const auto & entry : zone_rects_) {
    const auto & corners = entry.second;
    if (corners.size() < 4) {
      continue;
    }
    double zmin_x = corners[0].first;
    double zmax_x = corners[0].first;
    double zmin_y = corners[0].second;
    double zmax_y = corners[0].second;
    for (const auto & corner : corners) {
      zmin_x = std::min(zmin_x, corner.first);
      zmax_x = std::max(zmax_x, corner.first);
      zmin_y = std::min(zmin_y, corner.second);
      zmax_y = std::max(zmax_y, corner.second);
    }
    int c0 = 0;
    int r0 = 0;
    int c1 = 0;
    int r1 = 0;
    world_to_cell(zmin_x, zmin_y, c0, r0);
    world_to_cell(zmax_x, zmax_y, c1, r1);
    if (c0 > c1) {
      std::swap(c0, c1);
    }
    if (r0 > r1) {
      std::swap(r0, r1);
    }
    for (int cx = c0; cx <= c1; ++cx) {
      for (int cy = r0; cy <= r1; ++cy) {
        grid_.zone_cells[cell_key(cx, cy)].push_back(entry.first);
      }
    }
  }

  // Register each vertex in its containing grid cell.
  for (std::size_t i = 0; i < vertices_xy_.size(); ++i) {
    int cx = 0;
    int cy = 0;
    if (!world_to_cell(vertices_xy_[i].first, vertices_xy_[i].second, cx, cy)) {
      continue;
    }
    grid_.vertex_cells[cell_key(cx, cy)].push_back(i);
  }
}

// Grid-accelerated zone lookup; falls back to full scan when needed.
std::string ZoneTracker::find_zone_at_point_indexed(double px, double py) const
{
  if (!grid_.enabled) {
    return find_zone_at_point(px, py, zone_rects_);
  }

  int cx = 0;
  int cy = 0;
  if (!world_to_cell(px, py, cx, cy)) {
    return find_zone_at_point(px, py, zone_rects_);
  }

  std::string best_id;
  double best_area = std::numeric_limits<double>::infinity();
  std::unordered_set<std::string> checked;

  // Test one candidate zone id against the point (smallest area wins).
  auto consider = [&](const std::string & zone_id) {
      if (!checked.insert(zone_id).second) {
        return;
      }
      const auto it = zone_rects_.find(zone_id);
      if (it == zone_rects_.end()) {
        return;
      }
      const auto & corners = it->second;
      if (!point_in_rectangle(px, py, corners)) {
        return;
      }
      double min_x = corners[0].first;
      double max_x = corners[0].first;
      double min_y = corners[0].second;
      double max_y = corners[0].second;
      for (const auto & corner : corners) {
        min_x = std::min(min_x, corner.first);
        max_x = std::max(max_x, corner.first);
        min_y = std::min(min_y, corner.second);
        max_y = std::max(max_y, corner.second);
      }
      const double area = (max_x - min_x) * (max_y - min_y);
      if (area < best_area) {
        best_area = area;
        best_id = zone_id;
      }
    };

  // Check home cell + 8 neighbors for edge straddling.
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      const auto it = grid_.zone_cells.find(cell_key(cx + dx, cy + dy));
      if (it == grid_.zone_cells.end()) {
        continue;
      }
      for (const auto & zone_id : it->second) {
        consider(zone_id);
      }
    }
  }

  if (!best_id.empty()) {
    return best_id;
  }
  // Rare miss: fall back to full scan for correctness.
  return find_zone_at_point(px, py, zone_rects_);
}

// When pose is outside all zone bboxes, use nearest vertex's first zone label.
namespace
{
/// Rear-zone tuning. Deliberately few knobs: the graph supplies connectivity,
/// so there is no ray count, angular step or radial step to get wrong.
constexpr double kRearWalkBudgetM = 4.0;   // path length, not straight line
constexpr double kRearMinDistanceM = 0.5;  // skip vertices under the robot
/// A branch is strictly behind the robot while its bearing dot exceeds this.
/// Above 0 so an arm sitting dead abeam does not qualify: at the v4 junction
/// facing east, the south arm measures -0.005 — a twentieth of a degree past
/// perpendicular — and a robot approaching down it would strike amr1's SIDE,
/// not its rear.
constexpr double kRearStrictMin = 0.05;
/// Relaxed threshold, used ONLY when no branch is strictly behind. That happens
/// when the robot sits in a junction facing across the route: at v4 facing
/// south both the west and east arms sit at exactly 0.000, so there is no
/// single zone behind and BOTH flanks have to be claimed instead of picking one
/// arbitrarily. Slightly negative so yaw noise cannot flip such an arm in and
/// out, and with it a startPause/stopPause pair each time.
constexpr double kRearAbeamTolerance = -0.05;
constexpr std::size_t kRearMaxZones = 3;   // a dense junction must not block the map
constexpr double kRearLocateRadiusM = 6.0; // how far to look for a vertex to start from

/// Recompute gate. A parked robot re-walks the graph almost never.
constexpr double kRearReevalDistanceM = 0.1;
constexpr double kRearReevalYawRad = 0.087;  // ~5 degrees

/// Debounce, asymmetric on purpose. Claim in ~80 ms at 25 Hz, release after
/// ~200 ms, so a marginal zone lingers rather than flickering — every flicker
/// would be a startPause/stopPause pair on the wire.
constexpr int kRearAddTicks = 2;
constexpr int kRearDropTicks = 5;

/// Shortest signed angle, so comparing headings across the +/-pi wrap does not
/// read a 1-degree turn as 359 degrees and re-walk the graph every tick.
double wrap_angle(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}
}  // namespace

std::optional<std::size_t> ZoneTracker::nearest_vertex_indexed(
  double px, double py, double search_radius) const
{
  if (vertices_xy_.empty()) {
    return std::nullopt;
  }

  // Without the grid there is nothing to do but scan. With it, widen ring by
  // ring from the robot's own cell and stop as soon as the best candidate is
  // closer than the ring's inner edge — no further ring can beat it. This is
  // what keeps the cost flat as the nav graph grows: a 10k-vertex map has the
  // same few vertices near the robot as an 87-vertex one.
  if (!grid_.enabled) {
    std::size_t best = 0;
    double best_dist = std::numeric_limits<double>::infinity();
    for (std::size_t idx = 0; idx < vertices_xy_.size(); ++idx) {
      const double d = std::hypot(px - vertices_xy_[idx].first, py - vertices_xy_[idx].second);
      if (d < best_dist) {
        best_dist = d;
        best = idx;
      }
    }
    return best_dist <= search_radius ? std::optional<std::size_t>(best) : std::nullopt;
  }

  int cx = 0;
  int cy = 0;
  world_to_cell(px, py, cx, cy);

  const int max_ring = std::max(
    1, static_cast<int>(std::ceil(search_radius / grid_.cell_size)) + 1);

  std::size_t best = 0;
  double best_dist = std::numeric_limits<double>::infinity();

  for (int ring = 0; ring <= max_ring; ++ring) {
    // Everything in this ring is at least (ring - 1) cells away, so once the
    // best candidate beats that bound no wider ring can improve on it.
    if (best_dist < static_cast<double>(ring - 1) * grid_.cell_size) {
      break;
    }
    for (int dy = -ring; dy <= ring; ++dy) {
      for (int dx = -ring; dx <= ring; ++dx) {
        // Ring, not filled square: skip cells already visited.
        if (ring > 0 && std::abs(dx) != ring && std::abs(dy) != ring) {
          continue;
        }
        const auto it = grid_.vertex_cells.find(cell_key(cx + dx, cy + dy));
        if (it == grid_.vertex_cells.end()) {
          continue;
        }
        for (const std::size_t idx : it->second) {
          if (idx >= vertices_xy_.size()) {
            continue;
          }
          const double d =
            std::hypot(px - vertices_xy_[idx].first, py - vertices_xy_[idx].second);
          if (d < best_dist) {
            best_dist = d;
            best = idx;
          }
        }
      }
    }
  }

  if (best_dist > search_radius) {
    return std::nullopt;
  }
  return best;
}

std::vector<std::string> ZoneTracker::find_rear_zones_from_graph(
  double px,
  double py,
  double yaw,
  const std::string & current_zone) const
{
  std::vector<std::string> out;
  if (!map_loaded_ || vertex_adjacency_.empty()) {
    return out;
  }

  const auto start = nearest_vertex_indexed(px, py, kRearLocateRadiusM);
  if (!start.has_value()) {
    return out;
  }

  // Rear direction: opposite the robot's nose.
  const double back_dx = -std::cos(yaw);
  const double back_dy = -std::sin(yaw);

  // Bearing of a vertex relative to the rear vector. +1 straight back, 0 exactly
  // abeam, -1 straight ahead.
  const auto rearness = [&](std::size_t idx) {
      const double vx = vertices_xy_[idx].first - px;
      const double vy = vertices_xy_[idx].second - py;
      const double straight = std::hypot(vx, vy);
      if (straight < 1e-6) {
        return -1.0;
      }
      return (vx / straight) * back_dx + (vy / straight) * back_dy;
    };

  // Searched PER BRANCH, not as one flood fill.
  //
  // A single flood produced two wrong answers on the real map (measured
  // 2026-09-10, amr1 at the v4 junction facing east):
  //
  //   * it walked THROUGH a neighbour it had already judged not-rear (v14, dead
  //     abeam at dot -0.005), carried on two hops down the parallel aisle, and
  //     claimed zone 5 from there. Zone 5 is beside the robot, not behind it —
  //     nothing there can rear-end amr1 without first coming round the junction,
  //     which the forward reservations already cover.
  //   * it kept walking past the first zone it found, so a straight lane with
  //     ~1.3 m zone spacing returned three zones (15, 14, then 14 again further
  //     out) for what is one path behind the robot.
  //
  // So: only enter a branch whose FIRST step is rearward, and stop that branch
  // at the first zone that is not the robot's own. One path behind gives one
  // zone; a junction gives one per rearward arm.
  std::unordered_map<std::string, double> zone_best;

  // Run strictly first; only if NOTHING is behind the robot fall back to the
  // abeam threshold. Doing it in one relaxed pass claimed a robot's side arm as
  // "rear" on every heading — amr1 facing east at v4 picked up the south arm
  // (zone 6) alongside the west arm it actually reverses into.
  const auto sweep = [&](double threshold) {
      for (const std::size_t first : vertex_adjacency_[*start]) {
        if (first >= vertices_xy_.size()) {
          continue;
        }
        if (rearness(first) <= threshold) {
          continue;
        }

        // Walk this branch only, never re-entering the start vertex.
        std::unordered_map<std::size_t, double> best_cost;
        std::priority_queue<
          std::pair<double, std::size_t>,
          std::vector<std::pair<double, std::size_t>>,
          std::greater<>> frontier;

        const double first_step = std::hypot(
          vertices_xy_[first].first - vertices_xy_[*start].first,
          vertices_xy_[first].second - vertices_xy_[*start].second);
        best_cost[first] = first_step;
        frontier.emplace(first_step, first);

        while (!frontier.empty()) {
          const auto [cost, idx] = frontier.top();
          frontier.pop();

          const auto seen = best_cost.find(idx);
          if (seen != best_cost.end() && cost > seen->second) {
            continue;  // stale queue entry
          }
          if (cost > kRearWalkBudgetM) {
            continue;
          }

          bool found_zone = false;
          const double vx = vertices_xy_[idx].first - px;
          const double vy = vertices_xy_[idx].second - py;
          if (std::hypot(vx, vy) >= kRearMinDistanceM &&
            rearness(idx) > threshold &&
            idx < vertex_zone_labels_.size())
          {
            for (const auto & label : vertex_zone_labels_[idx]) {
              if (label.empty() || label == current_zone) {
                continue;
              }
              found_zone = true;
              const auto zit = zone_best.find(label);
              if (zit == zone_best.end() || cost < zit->second) {
                zone_best[label] = cost;
              }
            }
          }

          // The branch has delivered its adjacent zone; anything further along
          // it is the same path, not another way in.
          if (found_zone) {
            continue;
          }

          for (const std::size_t next : vertex_adjacency_[idx]) {
            if (next >= vertices_xy_.size() || next == *start) {
              continue;
            }
            const double step = std::hypot(
              vertices_xy_[next].first - vertices_xy_[idx].first,
              vertices_xy_[next].second - vertices_xy_[idx].second);
            const double next_cost = cost + step;
            if (next_cost > kRearWalkBudgetM) {
              continue;
            }
            const auto nit = best_cost.find(next);
            if (nit == best_cost.end() || next_cost < nit->second) {
              best_cost[next] = next_cost;
              frontier.emplace(next_cost, next);
            }
          }
        }
      }
    };

  sweep(kRearStrictMin);
  if (zone_best.empty()) {
    sweep(kRearAbeamTolerance);
  }

  // Nearest first, capped: a junction with many arms must not let one parked
  // robot reserve half the map.
  std::vector<std::pair<double, std::string>> ranked;
  ranked.reserve(zone_best.size());
  for (const auto & [zone, cost] : zone_best) {
    ranked.emplace_back(cost, zone);
  }
  std::sort(ranked.begin(), ranked.end());
  for (const auto & [cost, zone] : ranked) {
    (void)cost;
    if (out.size() >= kRearMaxZones) {
      break;
    }
    out.push_back(zone);
  }
  std::sort(out.begin(), out.end());
  return out;
}

std::vector<std::string> ZoneTracker::debounce_rear_zones(
  RobotZoneState & state,
  const std::vector<std::string> & raw)
{
  const std::unordered_set<std::string> raw_set(raw.begin(), raw.end());

  for (const auto & zone : raw) {
    state.rear_missed.erase(zone);
    const int seen = ++state.rear_seen[zone];
    if (seen >= kRearAddTicks &&
      std::find(state.rear_zones.begin(), state.rear_zones.end(), zone) ==
      state.rear_zones.end())
    {
      state.rear_zones.push_back(zone);
    }
  }

  for (auto it = state.rear_zones.begin(); it != state.rear_zones.end(); ) {
    if (raw_set.count(*it) > 0) {
      ++it;
      continue;
    }
    state.rear_seen.erase(*it);
    if (++state.rear_missed[*it] >= kRearDropTicks) {
      state.rear_missed.erase(*it);
      it = state.rear_zones.erase(it);
    } else {
      ++it;
    }
  }

  std::sort(state.rear_zones.begin(), state.rear_zones.end());
  return state.rear_zones;
}

std::string ZoneTracker::nearest_vertex_zone(double px, double py) const
{
  if (vertices_xy_.empty()) {
    return "";
  }

  if (!grid_.enabled) {
    // No grid: linear scan over all vertices.
    double best_dist = std::numeric_limits<double>::infinity();
    std::string fallback;
    for (std::size_t idx = 0; idx < vertices_xy_.size(); ++idx) {
      const double dist = std::hypot(
        px - vertices_xy_[idx].first,
        py - vertices_xy_[idx].second);
      if (dist < best_dist) {
        best_dist = dist;
        if (idx < vertex_zone_labels_.size() && !vertex_zone_labels_[idx].empty()) {
          fallback = vertex_zone_labels_[idx].front();
        }
      }
    }
    return fallback;
  }

  int cx = 0;
  int cy = 0;
  if (!world_to_cell(px, py, cx, cy)) {
    double best_dist = std::numeric_limits<double>::infinity();
    std::string fallback;
    for (std::size_t idx = 0; idx < vertices_xy_.size(); ++idx) {
      const double dist = std::hypot(
        px - vertices_xy_[idx].first,
        py - vertices_xy_[idx].second);
      if (dist < best_dist) {
        best_dist = dist;
        if (idx < vertex_zone_labels_.size() && !vertex_zone_labels_[idx].empty()) {
          fallback = vertex_zone_labels_[idx].front();
        }
      }
    }
    return fallback;
  }

  double best_dist = std::numeric_limits<double>::infinity();
  std::string fallback;
  const int max_ring = std::max(grid_.cols, grid_.rows) + 1;

  // Expand square rings around the pose cell until nearest vertex is found.
  for (int ring = 0; ring <= max_ring; ++ring) {
    bool any_cell = false;
    for (int dx = -ring; dx <= ring; ++dx) {
      for (int dy = -ring; dy <= ring; ++dy) {
        if (ring > 0 && std::abs(dx) != ring && std::abs(dy) != ring) {
          continue;
        }
        const auto it = grid_.vertex_cells.find(cell_key(cx + dx, cy + dy));
        if (it == grid_.vertex_cells.end()) {
          continue;
        }
        any_cell = true;
        for (const std::size_t idx : it->second) {
          const double dist = std::hypot(
            px - vertices_xy_[idx].first,
            py - vertices_xy_[idx].second);
          if (dist < best_dist) {
            best_dist = dist;
            if (idx < vertex_zone_labels_.size() && !vertex_zone_labels_[idx].empty()) {
              fallback = vertex_zone_labels_[idx].front();
            } else {
              fallback.clear();
            }
          }
        }
      }
    }
    // Once we found candidates in this ring (and farther rings can't beat
    // a point inside previous ring by more than cell diagonal), we can stop
    // after the first ring that produced a hit when ring==0, or after a hit
    // when ring>0 with best within current ring radius.
    if (any_cell && std::isfinite(best_dist) && ring > 0) {
      const double ring_reach = (ring - 0.5) * grid_.cell_size;
      if (best_dist <= ring_reach) {
        break;
      }
    }
    if (any_cell && ring == 0 && std::isfinite(best_dist)) {
      // Continue one more ring for boundary safety, then stop if still best.
      continue;
    }
    if (any_cell && ring == 1 && std::isfinite(best_dist)) {
      break;
    }
  }

  if (!std::isfinite(best_dist)) {
    // Absolute fallback
    for (std::size_t idx = 0; idx < vertices_xy_.size(); ++idx) {
      const double dist = std::hypot(
        px - vertices_xy_[idx].first,
        py - vertices_xy_[idx].second);
      if (dist < best_dist) {
        best_dist = dist;
        if (idx < vertex_zone_labels_.size() && !vertex_zone_labels_[idx].empty()) {
          fallback = vertex_zone_labels_[idx].front();
        }
      }
    }
  }
  return fallback;
}

// Map zone id -> first index in sequence for O(1) progress updates.
void ZoneTracker::rebuild_zone_index_map(
  const std::vector<std::string> & zone_sequence,
  std::unordered_map<std::string, int> & zone_to_seq_idx)
{
  zone_to_seq_idx.clear();
  zone_to_seq_idx.reserve(zone_sequence.size());
  for (std::size_t i = 0; i < zone_sequence.size(); ++i) {
    // Keep first occurrence for progress (matches previous std::find behavior).
    zone_to_seq_idx.emplace(zone_sequence[i], static_cast<int>(i));
  }
}

// Fill snapshot next_zone (max 3) and trim zone_sequence from current index.
void ZoneTracker::apply_remaining_and_lookahead(
  ZoneSnapshot & snapshot,
  const std::vector<std::string> & full_sequence,
  int current_idx)
{
  snapshot.next_zone.clear();
  snapshot.zone_sequence.clear();

  if (full_sequence.empty()) {
    return;
  }

  // No known current index: publish full sequence, no lookahead.
  if (current_idx < 0 || current_idx >= static_cast<int>(full_sequence.size())) {
    snapshot.zone_sequence = full_sequence;
    return;
  }

  // Vanish zones already passed: keep current + remaining.
  snapshot.zone_sequence.assign(
    full_sequence.begin() + current_idx, full_sequence.end());

  // next_zone: up to kNextZoneLookahead upcoming zones (or fewer if near end).
  const int start = current_idx + 1;
  const int end = std::min(
    start + kNextZoneLookahead, static_cast<int>(full_sequence.size()));
  for (int i = start; i < end; ++i) {
    snapshot.next_zone.push_back(full_sequence[static_cast<std::size_t>(i)]);
  }
}

// Parse "x,y" strings from MonitoredTaskDetails planned_path into numeric points.
std::vector<std::pair<double, double>> ZoneTracker::parse_planned_path_strings(
  const std::vector<std::string> & planned_path)
{
  std::vector<std::pair<double, double>> points;
  points.reserve(planned_path.size());
  for (const auto & entry : planned_path) {
    std::stringstream ss(entry);
    std::string part_x;
    std::string part_y;
    if (std::getline(ss, part_x, ',') && std::getline(ss, part_y, ',')) {
      try {
        points.emplace_back(std::stod(part_x), std::stod(part_y));
      } catch (...) {
        // Skip malformed entries.
      }
    }
  }
  return points;
}

// Static/test path: build zone sequence without instance spatial index.
std::vector<std::string> ZoneTracker::zone_sequence_from_xy_points(
  const std::vector<std::pair<double, double>> & points,
  const std::vector<std::pair<double, double>> & vertices_xy,
  const std::vector<std::vector<std::string>> & vertex_zone_labels,
  const std::unordered_map<std::string, std::vector<std::pair<double, double>>> & zone_rects,
  int interpolate_steps)
{
  std::vector<std::pair<double, double>> sampled;
  if (points.size() < 2) {
    sampled = points;
  } else {
    // Interpolate along each segment so thin zones are not skipped.
    const int steps = std::max(1, interpolate_steps);
    sampled.reserve(points.size() * static_cast<std::size_t>(steps));
    for (std::size_t i = 0; i + 1 < points.size(); ++i) {
      const double x0 = points[i].first;
      const double y0 = points[i].second;
      const double x1 = points[i + 1].first;
      const double y1 = points[i + 1].second;
      for (int step = 0; step < steps; ++step) {
        const double t = static_cast<double>(step) / steps;
        sampled.emplace_back(x0 + t * (x1 - x0), y0 + t * (y1 - y0));
      }
    }
    sampled.push_back(points.back());
  }

  std::vector<std::string> point_zones;
  point_zones.reserve(sampled.size());
  for (const auto & point : sampled) {
    // Bbox lookup with linear nearest-vertex fallback (static/test API).
    std::string zone_id = find_zone_at_point(point.first, point.second, zone_rects);
    if (zone_id.empty() && !vertices_xy.empty()) {
      double best_dist = std::numeric_limits<double>::infinity();
      std::string fallback;
      for (std::size_t idx = 0; idx < vertices_xy.size(); ++idx) {
        const double dist = std::hypot(
          point.first - vertices_xy[idx].first,
          point.second - vertices_xy[idx].second);
        if (dist < best_dist) {
          best_dist = dist;
          if (idx < vertex_zone_labels.size() && !vertex_zone_labels[idx].empty()) {
            fallback = vertex_zone_labels[idx].front();
          }
        }
      }
      zone_id = fallback;
    }
    if (!zone_id.empty()) {
      point_zones.push_back(zone_id);
    }
  }

  std::vector<std::string> sequence;
  sequence.reserve(point_zones.size());
  // Collapse consecutive duplicates into ordered zone sequence.
  for (const auto & zone_id : point_zones) {
    if (sequence.empty() || sequence.back() != zone_id) {
      sequence.push_back(zone_id);
    }
  }
  return sequence;
}

// Instance path: same as static version but uses spatial index for lookups.
std::vector<std::string> ZoneTracker::zone_sequence_from_xy_points_indexed(
  const std::vector<std::pair<double, double>> & points,
  int interpolate_steps) const
{
  std::vector<std::pair<double, double>> sampled;
  if (points.size() < 2) {
    sampled = points;
  } else {
    // Interpolate along each segment so thin zones are not skipped.
    const int steps = std::max(1, interpolate_steps);
    sampled.reserve(points.size() * static_cast<std::size_t>(steps));
    for (std::size_t i = 0; i + 1 < points.size(); ++i) {
      const double x0 = points[i].first;
      const double y0 = points[i].second;
      const double x1 = points[i + 1].first;
      const double y1 = points[i + 1].second;
      for (int step = 0; step < steps; ++step) {
        const double t = static_cast<double>(step) / steps;
        sampled.emplace_back(x0 + t * (x1 - x0), y0 + t * (y1 - y0));
      }
    }
    sampled.push_back(points.back());
  }

  std::vector<std::string> point_zones;
  point_zones.reserve(sampled.size());
  // Resolve each sampled point to a zone id (bbox hit or nearest vertex).
  for (const auto & point : sampled) {
    std::string zone_id = find_zone_at_point_indexed(point.first, point.second);
    if (zone_id.empty()) {
      zone_id = nearest_vertex_zone(point.first, point.second);
    }
    if (!zone_id.empty()) {
      point_zones.push_back(zone_id);
    }
  }

  std::vector<std::string> sequence;
  sequence.reserve(point_zones.size());
  // Collapse consecutive duplicates into ordered zone sequence.
  for (const auto & zone_id : point_zones) {
    if (sequence.empty() || sequence.back() != zone_id) {
      sequence.push_back(zone_id);
    }
  }
  return sequence;
}

// Static/test helper: current zone + trimmed sequence from pose (no spatial index).
ZoneSnapshot ZoneTracker::current_and_next_from_pose(
  double px,
  double py,
  const std::vector<std::pair<double, double>> & vertices_xy,
  const std::vector<std::vector<std::string>> & vertex_zone_labels,
  const std::unordered_map<std::string, std::vector<std::pair<double, double>>> & zone_rects,
  const std::vector<std::string> & zone_sequence,
  int seq_progress)
{
  ZoneSnapshot snapshot;

  // Geometric zone at pose; nearest vertex label if outside all bboxes.
  std::string geo = find_zone_at_point(px, py, zone_rects);
  if (geo.empty() && !vertices_xy.empty()) {
    double best_dist = std::numeric_limits<double>::infinity();
    for (std::size_t idx = 0; idx < vertices_xy.size(); ++idx) {
      const double dist = std::hypot(px - vertices_xy[idx].first, py - vertices_xy[idx].second);
      if (dist < best_dist) {
        best_dist = dist;
        if (idx < vertex_zone_labels.size() && !vertex_zone_labels[idx].empty()) {
          geo = vertex_zone_labels[idx].front();
        }
      }
    }
  }

  if (zone_sequence.empty()) {
    snapshot.current_zone = geo;
    return snapshot;
  }

  // Advance monotonic progress when pose matches a zone in the planned sequence.
  int new_seq_progress = seq_progress;
  if (!geo.empty()) {
    const auto it = std::find(zone_sequence.begin(), zone_sequence.end(), geo);
    if (it != zone_sequence.end()) {
      const int idx = static_cast<int>(std::distance(zone_sequence.begin(), it));
      if (new_seq_progress < 0 || idx > new_seq_progress) {
        new_seq_progress = idx;
      }
    }
  }

  std::string current = geo;
  int current_idx = -1;
  // Prefer geo when on-path; otherwise fall back to last known progress index.
  if (!geo.empty()) {
    const auto it = std::find(zone_sequence.begin(), zone_sequence.end(), geo);
    if (it != zone_sequence.end()) {
      current_idx = static_cast<int>(std::distance(zone_sequence.begin(), it));
    } else if (new_seq_progress >= 0 && new_seq_progress < static_cast<int>(zone_sequence.size())) {
      current = zone_sequence[static_cast<std::size_t>(new_seq_progress)];
      current_idx = new_seq_progress;
    }
  } else if (new_seq_progress >= 0 && new_seq_progress < static_cast<int>(zone_sequence.size())) {
    current = zone_sequence[static_cast<std::size_t>(new_seq_progress)];
    current_idx = new_seq_progress;
  } else {
    current = "";
    current_idx = -1;
  }

  snapshot.current_zone = current;
  apply_remaining_and_lookahead(snapshot, zone_sequence, current_idx);
  return snapshot;
}

// Production path: indexed zone lookup + hash map for sequence progress.
ZoneSnapshot ZoneTracker::current_and_next_from_pose_indexed(
  double px,
  double py,
  const std::vector<std::string> & zone_sequence,
  const std::unordered_map<std::string, int> & zone_to_seq_idx,
  int seq_progress) const
{
  ZoneSnapshot snapshot;

  // Indexed bbox lookup, then nearest-vertex fallback.
  std::string geo = find_zone_at_point_indexed(px, py);
  if (geo.empty()) {
    geo = nearest_vertex_zone(px, py);
  }

  if (zone_sequence.empty()) {
    snapshot.current_zone = geo;
    return snapshot;
  }

  // O(1) progress update via zone_to_seq_idx hash map.
  int new_seq_progress = seq_progress;
  if (!geo.empty()) {
    const auto it = zone_to_seq_idx.find(geo);
    if (it != zone_to_seq_idx.end()) {
      const int idx = it->second;
      if (new_seq_progress < 0 || idx > new_seq_progress) {
        new_seq_progress = idx;
      }
    }
  }

  std::string current = geo;
  int current_idx = -1;
  // Resolve current index from geo or sticky seq_progress when geo is off-path.
  if (!geo.empty()) {
    const auto it = zone_to_seq_idx.find(geo);
    if (it != zone_to_seq_idx.end()) {
      current_idx = it->second;
    } else if (new_seq_progress >= 0 && new_seq_progress < static_cast<int>(zone_sequence.size())) {
      current = zone_sequence[static_cast<std::size_t>(new_seq_progress)];
      current_idx = new_seq_progress;
    }
  } else if (new_seq_progress >= 0 && new_seq_progress < static_cast<int>(zone_sequence.size())) {
    current = zone_sequence[static_cast<std::size_t>(new_seq_progress)];
    current_idx = new_seq_progress;
  } else {
    current = "";
    current_idx = -1;
  }

  snapshot.current_zone = current;
  apply_remaining_and_lookahead(snapshot, zone_sequence, current_idx);
  return snapshot;
}

// Load YAML map via NavGraphStore and populate tracker geometry.
bool ZoneTracker::load_map(const std::string & yaml_path, const std::string & level)
{
  RCLCPP_INFO(
    zone_tracker_logger(),
    "Loading zone tracker map from '%s' (level=%s)",
    yaml_path.c_str(),
    level.c_str());

  NavGraphStore store;
  if (!store.load_map(yaml_path, level)) {
    RCLCPP_ERROR(
      zone_tracker_logger(),
      "Failed to load zone tracker map from '%s' (level=%s)",
      yaml_path.c_str(),
      level.c_str());
    return false;
  }
  if (!load_from_store(store)) {
    RCLCPP_ERROR(
      zone_tracker_logger(),
      "Failed to build zone tracker geometry from '%s'",
      yaml_path.c_str());
    return false;
  }
  return true;
}

// Copy vertices/zones from shared store and rebuild spatial helpers.
bool ZoneTracker::load_from_store(const NavGraphStore & store)
{
  vertices_.clear();
  vertices_xy_.clear();
  vertex_zone_labels_.clear();
  zone_rects_.clear();
  vertex_adjacency_.clear();
  grid_ = SpatialGrid{};
  map_loaded_ = false;

  if (!store.loaded()) {
    RCLCPP_WARN(
      zone_tracker_logger(),
      "NavGraphStore is not loaded; cannot adopt into ZoneTracker");
    return false;
  }

  // Mirror NavGraphStore vertices into local Vertex list.
  vertices_.reserve(store.vertices().size());
  for (const auto & v : store.vertices()) {
    Vertex vertex;
    vertex.x = v.x;
    vertex.y = v.y;
    vertex.zone_labels = v.zone_labels;
    vertices_.push_back(std::move(vertex));
  }

  // Merge waypoint + edge indices per zone for bbox construction.
  std::unordered_map<std::string, std::vector<int>> zone_vertex_indices;
  for (const auto & [zone_id, zone] : store.zones()) {
    std::unordered_set<int> unique_indices;
    for (int idx : zone.waypoints) {
      unique_indices.insert(idx);
    }
    for (int idx : zone.edge) {
      unique_indices.insert(idx);
    }
    zone_vertex_indices[zone_id] =
      std::vector<int>(unique_indices.begin(), unique_indices.end());
  }

  if (vertices_.empty() || zone_vertex_indices.empty()) {
    RCLCPP_ERROR(
      zone_tracker_logger(),
      "Zone tracker load failed: vertices=%zu zone_definitions=%zu",
      vertices_.size(),
      zone_vertex_indices.size());
    return false;
  }

  // Lane adjacency for the rear-zone walk. NavGraphStore already parsed the
  // lanes; load_from_store used to drop them on the floor.
  //
  // Stored UNDIRECTED. A lane's direction says which way traffic normally
  // flows, not which way a robot can approach from — and being rear-ended does
  // not require the other robot to be driving the lane legally.
  vertex_adjacency_.assign(vertices_.size(), {});
  for (const auto & lane : store.lanes()) {
    if (lane.from_vertex < 0 || lane.to_vertex < 0) {
      continue;
    }
    const auto from = static_cast<std::size_t>(lane.from_vertex);
    const auto to = static_cast<std::size_t>(lane.to_vertex);
    if (from >= vertices_.size() || to >= vertices_.size() || from == to) {
      continue;
    }
    vertex_adjacency_[from].push_back(to);
    vertex_adjacency_[to].push_back(from);
  }
  for (auto & neighbours : vertex_adjacency_) {
    std::sort(neighbours.begin(), neighbours.end());
    neighbours.erase(
      std::unique(neighbours.begin(), neighbours.end()), neighbours.end());
  }

  build_zone_rects(vertices_, zone_vertex_indices, zone_rects_);
  rebuild_precomputed_helpers();
  build_spatial_index();
  map_loaded_ = !vertices_.empty() && !zone_rects_.empty();
  if (map_loaded_) {
    RCLCPP_INFO(
      zone_tracker_logger(),
      "Zone tracker ready: vertices=%zu zone_rects=%zu spatial_grid=%s (%dx%d cells, cell_size=%.2f m)",
      vertices_.size(),
      zone_rects_.size(),
      grid_.enabled ? "enabled" : "disabled",
      grid_.cols,
      grid_.rows,
      grid_.cell_size);
  } else {
    RCLCPP_ERROR(
      zone_tracker_logger(),
      "Zone tracker load failed after geometry build: vertices=%zu zone_rects=%zu",
      vertices_.size(),
      zone_rects_.size());
  }
  return map_loaded_;
}

// Cache latest robot pose for zone queries (normalized robot id key).
void ZoneTracker::update_robot_pose(
  const std::string & robot_id, double x, double y, double yaw)
{
  const std::string normalized_id = normalize_robot_id(robot_id);
  if (normalized_id.empty()) {
    return;
  }
  auto & state = robot_states_[normalized_id];
  state.x = x;
  state.y = y;
  state.yaw = yaw;
  state.has_pose = true;
}

// Accept planned path as "x,y" strings from MonitoredTaskDetails.
void ZoneTracker::update_robot_planned_path(
  const std::string & robot_id,
  const std::vector<std::string> & planned_path)
{
  update_robot_planned_path_xy(robot_id, parse_planned_path_strings(planned_path));
}

// Build zone sequence from numeric path points and reset progress.
void ZoneTracker::update_robot_planned_path_xy(
  const std::string & robot_id,
  const std::vector<std::pair<double, double>> & planned_points)
{
  const std::string normalized_id = normalize_robot_id(robot_id);
  if (normalized_id.empty()) {
    RCLCPP_WARN(
      zone_tracker_logger(),
      "Ignoring planned path update with empty robot id");
    return;
  }
  if (!map_loaded_) {
    RCLCPP_WARN(
      zone_tracker_logger(),
      "Ignoring planned path for '%s': zone map not loaded",
      normalized_id.c_str());
    return;
  }
  if (planned_points.empty()) {
    RCLCPP_DEBUG(
      zone_tracker_logger(),
      "Ignoring empty planned path for '%s'",
      normalized_id.c_str());
    return;
  }

  auto & state = robot_states_[normalized_id];
  state.has_task = true;
  state.zone_sequence = zone_sequence_from_xy_points_indexed(planned_points);
  rebuild_zone_index_map(state.zone_sequence, state.zone_to_seq_idx);
  state.seq_progress = -1;
  RCLCPP_DEBUG(
    zone_tracker_logger(),
    "Robot '%s' planned path -> %zu zone(s)",
    normalized_id.c_str(),
    state.zone_sequence.size());
}

// Drop task state while keeping last known pose.
void ZoneTracker::clear_robot_task(const std::string & robot_id)
{
  const std::string normalized_id = normalize_robot_id(robot_id);
  if (normalized_id.empty()) {
    return;
  }
  auto it = robot_states_.find(normalized_id);
  if (it == robot_states_.end()) {
    return;
  }
  it->second.has_task = false;
  it->second.zone_sequence.clear();
  it->second.zone_to_seq_idx.clear();
  it->second.seq_progress = -1;
  RCLCPP_DEBUG(
    zone_tracker_logger(),
    "Cleared task state for robot '%s'",
    normalized_id.c_str());
}

// Live zone snapshot for active task: current, next (3), trimmed sequence.
void ZoneTracker::fill_rear_zones(RobotZoneState & state, ZoneSnapshot & snapshot)
{
  if (!state.has_pose || snapshot.current_zone.empty()) {
    snapshot.rear_zones = state.rear_zones;
    return;
  }

  // Only re-walk when the robot has actually moved or turned. A parked robot —
  // the case this whole feature exists for — costs nothing per tick.
  const bool moved =
    !state.has_rear_eval ||
    std::hypot(state.x - state.rear_eval_x, state.y - state.rear_eval_y) >
    kRearReevalDistanceM ||
    std::fabs(wrap_angle(state.yaw - state.rear_eval_yaw)) > kRearReevalYawRad;

  // The gate skips the WALK, never the debounce: the counters must keep
  // advancing or a stationary robot never confirms a zone.
  if (moved) {
    state.has_rear_eval = true;
    state.rear_eval_x = state.x;
    state.rear_eval_y = state.y;
    state.rear_eval_yaw = state.yaw;
    state.rear_raw = find_rear_zones_from_graph(
      state.x, state.y, state.yaw, snapshot.current_zone);
  }

  snapshot.rear_zones = debounce_rear_zones(state, state.rear_raw);

  if (snapshot.rear_zones.empty()) {
    // Visible on purpose. A robot with no rear zone has no rear protection,
    // and a silently missing safety claim is the worst kind.
    RCLCPP_DEBUG(
      zone_tracker_logger(),
      "No rear zone for pose (%.2f, %.2f, yaw %.2f) in zone '%s'",
      state.x, state.y, state.yaw, snapshot.current_zone.c_str());
  }
}

ZoneSnapshot ZoneTracker::get_robot_zones(const std::string & robot_id)
{
  const std::string normalized_id = normalize_robot_id(robot_id);
  const auto it = robot_states_.find(normalized_id);
  if (it == robot_states_.end() || !map_loaded_ || !it->second.has_pose) {
    return {};
  }

  RobotZoneState & state = it->second;
  ZoneSnapshot snapshot = current_and_next_from_pose_indexed(
    state.x, state.y, state.zone_sequence, state.zone_to_seq_idx, state.seq_progress);

  // Monotonic forward progress along planned sequence while task is active.
  if (state.has_task && !snapshot.current_zone.empty()) {
    const auto seq_it = state.zone_to_seq_idx.find(snapshot.current_zone);
    if (seq_it != state.zone_to_seq_idx.end()) {
      const int idx = seq_it->second;
      if (state.seq_progress < 0 || idx > state.seq_progress) {
        state.seq_progress = idx;
      }
    }
  }

  fill_rear_zones(state, snapshot);
  return snapshot;
}

// At task completion: snap to destination zone if pose hasn't reached it yet.
ZoneSnapshot ZoneTracker::get_robot_zones_at_completion(const std::string & robot_id)
{
  const std::string normalized_id = normalize_robot_id(robot_id);
  const auto it = robot_states_.find(normalized_id);
  if (it == robot_states_.end() || !map_loaded_ || !it->second.has_pose) {
    return {};
  }

  RobotZoneState & state = it->second;
  const ZoneSnapshot live = current_and_next_from_pose_indexed(
    state.x, state.y, state.zone_sequence, state.zone_to_seq_idx, state.seq_progress);

  if (!state.has_task || state.zone_sequence.empty()) {
    return live;
  }

  const std::vector<std::string> & full_seq = state.zone_sequence;
  const int dest_idx = static_cast<int>(full_seq.size()) - 1;
  const std::string & dest_zone = full_seq[static_cast<std::size_t>(dest_idx)];

  // Compare live pose zone index against final destination index.
  int live_idx = -1;
  if (!live.current_zone.empty()) {
    const auto live_it = state.zone_to_seq_idx.find(live.current_zone);
    if (live_it != state.zone_to_seq_idx.end()) {
      live_idx = live_it->second;
    }
  }

  if (live_idx >= dest_idx) {
    return live;
  }

  // Pose still short of destination: publish destination as current zone.
  RCLCPP_DEBUG(
    zone_tracker_logger(),
    "Robot '%s' completion snap: live zone '%s' (idx %d) -> destination '%s' (idx %d)",
    normalized_id.c_str(),
    live.current_zone.c_str(),
    live_idx,
    dest_zone.c_str(),
    dest_idx);
  ZoneSnapshot result;
  result.current_zone = dest_zone;
  apply_remaining_and_lookahead(result, full_seq, dest_idx);
  return result;
}

// Idle robot: geometric zone only, sequence is single current zone.
ZoneSnapshot ZoneTracker::get_robot_idle_zones(const std::string & robot_id)
{
  const std::string normalized_id = normalize_robot_id(robot_id);
  const auto it = robot_states_.find(normalized_id);
  if (it == robot_states_.end() || !map_loaded_ || !it->second.has_pose) {
    return {};
  }

  ZoneSnapshot snapshot;
  std::string geo = find_zone_at_point_indexed(it->second.x, it->second.y);
  if (geo.empty()) {
    geo = nearest_vertex_zone(it->second.x, it->second.y);
  }
  if (geo.empty()) {
    return snapshot;
  }

  snapshot.current_zone = geo;
  snapshot.zone_sequence = {geo};
  fill_rear_zones(it->second, snapshot);
  return snapshot;
}

}  // namespace monitor
