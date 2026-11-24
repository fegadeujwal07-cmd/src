// =============================================================================
// 4D / ASRS drop corridor: free X highways + Z spines + pick/drop Z; block rest.
// =============================================================================

#include "execution/drop_corridor.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <queue>
#include <vector>

namespace execution::drop_corridor {
namespace {

constexpr double kSameLineEpsM = 0.01;
constexpr double kOnCorridorTolM = 0.10;
// Main vertical spines ≈ Excel orange columns: keep columns whose y-span is
// within this fraction of the longest Z column on the graph.
constexpr double kSpineSpanRatio = 0.90;

bool IsValidVertex(const Geometry& geometry, int index) {
  if (index < 0 ||
      static_cast<std::size_t>(index) >= geometry.vertices.size()) {
    return false;
  }
  return !std::isnan(geometry.vertices[static_cast<std::size_t>(index)].x);
}

bool IsValidLane(const Geometry& geometry, std::size_t lane) {
  const LaneEnds& ends = geometry.lanes[lane];
  return IsValidVertex(geometry, ends.start) &&
         IsValidVertex(geometry, ends.end);
}

const Point& StartOf(const Geometry& geometry, std::size_t lane) {
  return geometry.vertices[static_cast<std::size_t>(
      geometry.lanes[lane].start)];
}

const Point& EndOf(const Geometry& geometry, std::size_t lane) {
  return geometry.vertices[static_cast<std::size_t>(
      geometry.lanes[lane].end)];
}

bool IsHorizontal(const Point& a, const Point& b) {
  return std::abs(a.y - b.y) < kSameLineEpsM &&
         std::abs(a.x - b.x) >= kSameLineEpsM;
}

bool IsVertical(const Point& a, const Point& b) {
  return std::abs(a.x - b.x) < kSameLineEpsM;
}

bool NearAny(double value, const std::vector<double>& coords, double tol) {
  return std::any_of(coords.begin(), coords.end(),
                     [value, tol](double c) {
                       return std::abs(value - c) < tol;
                     });
}

void PushUnique(std::vector<double>* coords, double value) {
  if (!NearAny(value, *coords, kSameLineEpsM)) {
    coords->push_back(value);
  }
}

std::optional<std::size_t> NearestVertex(const Geometry& geometry,
                                         const Point& point) {
  std::optional<std::size_t> best;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t v = 0; v < geometry.vertices.size(); ++v) {
    if (!IsValidVertex(geometry, static_cast<int>(v))) {
      continue;
    }
    const double distance = Distance(geometry.vertices[v], point);
    if (distance < best_distance) {
      best_distance = distance;
      best = v;
    }
  }
  return best;
}

// Every y that carries a horizontal lane is an X highway (Excel orange rows).
std::vector<double> CollectHighwayYs(const Geometry& geometry) {
  std::vector<double> highways;
  for (std::size_t lane = 0; lane < geometry.lanes.size(); ++lane) {
    if (!IsValidLane(geometry, lane) ||
        !IsHorizontal(StartOf(geometry, lane), EndOf(geometry, lane))) {
      continue;
    }
    PushUnique(&highways, StartOf(geometry, lane).y);
  }
  return highways;
}

// Longest vertical columns = main Z spines (Excel orange vertical highways).
std::vector<double> CollectSpineXs(const Geometry& geometry) {
  struct Span {
    double x = 0.0;
    double ymin = 0.0;
    double ymax = 0.0;
    bool init = false;
  };
  std::vector<Span> spans;
  for (std::size_t lane = 0; lane < geometry.lanes.size(); ++lane) {
    if (!IsValidLane(geometry, lane) ||
        !IsVertical(StartOf(geometry, lane), EndOf(geometry, lane))) {
      continue;
    }
    const Point& a = StartOf(geometry, lane);
    const Point& b = EndOf(geometry, lane);
    const double x = a.x;
    const double y0 = std::min(a.y, b.y);
    const double y1 = std::max(a.y, b.y);
    Span* slot = nullptr;
    for (Span& s : spans) {
      if (std::abs(s.x - x) < kSameLineEpsM) {
        slot = &s;
        break;
      }
    }
    if (slot == nullptr) {
      spans.push_back({x, y0, y1, true});
      continue;
    }
    slot->ymin = std::min(slot->ymin, y0);
    slot->ymax = std::max(slot->ymax, y1);
  }

  double max_span = 0.0;
  for (const Span& s : spans) {
    max_span = std::max(max_span, s.ymax - s.ymin);
  }
  std::vector<double> spines;
  if (max_span <= kSameLineEpsM) {
    return spines;
  }
  for (const Span& s : spans) {
    if ((s.ymax - s.ymin) >= kSpineSpanRatio * max_span) {
      spines.push_back(s.x);
    }
  }
  return spines;
}

bool WithinSpineSpan(const Lines& lines, double y, double tol) {
  return !lines.limit_spines_to_highway_span ||
         (y >= lines.spine_ymin - tol && y <= lines.spine_ymax + tol);
}

bool IsFreeLane(const Point& a, const Point& b, const Lines& lines) {
  if (IsHorizontal(a, b)) {
    return NearAny(a.y, lines.highway_ys, kSameLineEpsM);
  }
  if (!IsVertical(a, b)) {
    return false;
  }
  // The robot's own pick and drop columns are free over their full length.
  if (std::abs(a.x - lines.pickup_column_x) < kSameLineEpsM ||
      std::abs(a.x - lines.drop_column_x) < kSameLineEpsM) {
    return true;
  }
  return NearAny(a.x, lines.spine_xs, kSameLineEpsM) &&
         WithinSpineSpan(lines, a.y, kSameLineEpsM) &&
         WithinSpineSpan(lines, b.y, kSameLineEpsM);
}

// Keep only configured lines that really exist in the graph (a typo in the
// config must not silently free nothing). `horizontal` selects rows vs columns.
std::vector<double> ConfiguredLinesOnGraph(const Geometry& geometry,
                                           const std::vector<double>& wanted,
                                           bool horizontal) {
  std::vector<double> found;
  for (const double value : wanted) {
    for (std::size_t lane = 0; lane < geometry.lanes.size(); ++lane) {
      if (!IsValidLane(geometry, lane)) {
        continue;
      }
      const Point& a = StartOf(geometry, lane);
      const Point& b = EndOf(geometry, lane);
      const bool match = horizontal
          ? (IsHorizontal(a, b) && std::abs(a.y - value) < kSameLineEpsM)
          : (IsVertical(a, b) && std::abs(a.x - value) < kSameLineEpsM);
      if (match) {
        PushUnique(&found, value);
        break;
      }
    }
  }
  return found;
}

std::vector<bool> MarkFreeLanes(const Geometry& geometry, const Lines& lines) {
  std::vector<bool> is_free(geometry.lanes.size(), false);
  for (std::size_t lane = 0; lane < geometry.lanes.size(); ++lane) {
    is_free[lane] = IsValidLane(geometry, lane) &&
                    IsFreeLane(StartOf(geometry, lane), EndOf(geometry, lane),
                               lines);
  }
  return is_free;
}

bool IsReachable(const Geometry& geometry, const std::vector<bool>& is_free,
                 std::size_t from, std::size_t to) {
  std::vector<std::vector<std::size_t>> next(geometry.vertices.size());
  for (std::size_t lane = 0; lane < geometry.lanes.size(); ++lane) {
    if (is_free[lane]) {
      next[static_cast<std::size_t>(geometry.lanes[lane].start)].push_back(
          static_cast<std::size_t>(geometry.lanes[lane].end));
    }
  }
  std::vector<bool> seen(geometry.vertices.size(), false);
  std::queue<std::size_t> frontier;
  frontier.push(from);
  seen[from] = true;
  while (!frontier.empty() && !seen[to]) {
    const std::size_t vertex = frontier.front();
    frontier.pop();
    for (const std::size_t neighbour : next[vertex]) {
      if (!seen[neighbour]) {
        seen[neighbour] = true;
        frontier.push(neighbour);
      }
    }
  }
  return seen[to];
}

Corridor BuildCorridor(const Geometry& geometry, const Lines& lines,
                       const std::vector<bool>& is_free,
                       std::size_t pickup_vertex, std::size_t drop_vertex) {
  Corridor corridor;
  corridor.lines = lines;
  corridor.pickup_vertex = pickup_vertex;
  corridor.drop_vertex = drop_vertex;
  for (std::size_t lane = 0; lane < geometry.lanes.size(); ++lane) {
    if (is_free[lane]) {
      ++corridor.free_lane_count;
    } else if (IsValidLane(geometry, lane)) {
      corridor.closed_lanes.push_back(lane);
    }
  }
  return corridor;
}

bool IsOnFreeLine(const Lines& lines, const Point& point) {
  return NearAny(point.y, lines.highway_ys, kOnCorridorTolM) ||
         std::abs(point.x - lines.pickup_column_x) < kOnCorridorTolM ||
         std::abs(point.x - lines.drop_column_x) < kOnCorridorTolM ||
         (NearAny(point.x, lines.spine_xs, kOnCorridorTolM) &&
          WithinSpineSpan(lines, point.y, kOnCorridorTolM));
}

}  // namespace

double Distance(const Point& a, const Point& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

std::optional<Corridor> FindCorridor(const Geometry& geometry,
                                     const Point& pickup, const Point& drop,
                                     const LayoutHighways& layout) {
  const std::optional<std::size_t> pickup_vertex =
      NearestVertex(geometry, pickup);
  const std::optional<std::size_t> drop_vertex = NearestVertex(geometry, drop);
  if (!pickup_vertex || !drop_vertex || *pickup_vertex == *drop_vertex) {
    return std::nullopt;
  }
  const Point& from = geometry.vertices[*pickup_vertex];
  const Point& to = geometry.vertices[*drop_vertex];

  Lines lines;
  lines.pickup_column_x = from.x;
  lines.drop_column_x = to.x;
  // Layout sheet first; infer from the graph only when not configured.
  lines.highway_ys = layout.x_highway_ys.empty()
      ? CollectHighwayYs(geometry)
      : ConfiguredLinesOnGraph(geometry, layout.x_highway_ys, true);
  if (layout.z_highway_xs.empty()) {
    lines.spine_xs = CollectSpineXs(geometry);
  } else {
    lines.spine_xs =
        ConfiguredLinesOnGraph(geometry, layout.z_highway_xs, false);
    // Sheet Z highways run only between the outermost X highways.
    if (!lines.highway_ys.empty()) {
      lines.limit_spines_to_highway_span = true;
      lines.spine_ymin = *std::min_element(lines.highway_ys.begin(),
                                           lines.highway_ys.end());
      lines.spine_ymax = *std::max_element(lines.highway_ys.begin(),
                                           lines.highway_ys.end());
    }
  }
  if (lines.highway_ys.empty()) {
    return std::nullopt;
  }

  const std::vector<bool> is_free = MarkFreeLanes(geometry, lines);
  if (!IsReachable(geometry, is_free, *pickup_vertex, *drop_vertex)) {
    return std::nullopt;
  }
  return BuildCorridor(geometry, lines, is_free, *pickup_vertex,
                       *drop_vertex);
}

bool IsPathOnCorridor(const Lines& lines, const Point& drop,
                      double end_tolerance_m, const std::vector<Point>& path) {
  if (path.size() < 2 || Distance(path.back(), drop) > end_tolerance_m) {
    return false;
  }
  return std::all_of(path.begin() + 1, path.end(),
                     [&lines](const Point& p) {
                       return IsOnFreeLine(lines, p);
                     });
}

bool HasStartedTravel(bool reports_moving, const Point& plan_pose,
                      const Point& current_pose, double travel_start_m) {
  return reports_moving || Distance(plan_pose, current_pose) >= travel_start_m;
}

}  // namespace execution::drop_corridor
