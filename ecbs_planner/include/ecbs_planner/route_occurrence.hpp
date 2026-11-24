#ifndef ECBS_PLANNER__ROUTE_OCCURRENCE_HPP_
#define ECBS_PLANNER__ROUTE_OCCURRENCE_HPP_

#include <cmath>
#include <cstdint>
#include <string>

namespace ecbs_planner
{

/// Position on a planned route. Graph vertex id ≠ route-array index.
struct RouteOccurrence
{
  uint32_t path_revision{0};
  uint32_t route_occurrence{0};
  double segment_offset_m{0.0};
  int64_t waypoint_index{-1};
  double x{0.0};
  double y{0.0};
  std::string level_name;

  bool same_slot(const RouteOccurrence & o) const
  {
    return path_revision == o.path_revision &&
           route_occurrence == o.route_occurrence &&
           std::fabs(segment_offset_m - o.segment_offset_m) < 1e-6;
  }
};

inline RouteOccurrence make_occurrence(
  uint32_t path_revision,
  uint32_t route_occurrence,
  double x,
  double y,
  const std::string & level = "",
  int64_t waypoint_index = -1,
  double segment_offset_m = 0.0)
{
  RouteOccurrence o;
  o.path_revision = path_revision;
  o.route_occurrence = route_occurrence;
  o.segment_offset_m = segment_offset_m;
  o.waypoint_index = waypoint_index;
  o.x = x;
  o.y = y;
  o.level_name = level;
  return o;
}

}  // namespace ecbs_planner

#endif  // ECBS_PLANNER__ROUTE_OCCURRENCE_HPP_
