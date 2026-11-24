#ifndef ECBS_PLANNER__NAV_GRAPH_HPP_
#define ECBS_PLANNER__NAV_GRAPH_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_set>
#include <utility>
#include <vector>

namespace ecbs_planner
{

/// Directed lanes of the fleet nav graph, indexed like the RMF graph the
/// fleet adapter builds (waypoint indices match TimedPoint::index).
class NavGraph
{
public:
  void add_lane(std::size_t entry, std::size_t exit);
  bool empty() const {return out_.empty();}
  std::size_t num_waypoints() const {return out_.size();}

  /// True if \p to can be reached from \p from while every lane touching a
  /// waypoint in \p closed is closed (the same rule full_control applies).
  bool reachable(
    std::size_t from, std::size_t to,
    const std::unordered_set<int64_t> & closed) const;

  /// Position of waypoint \p wp (needed for shortest_length).
  void set_position(std::size_t wp, double x, double y);

  /// XY of waypoint \p wp; nullopt if unknown.
  std::optional<std::pair<double, double>> position(std::size_t wp) const;

  /// Length (m) of the shortest route from \p from to \p to with every
  /// waypoint in \p closed shut; nullopt if there is none or positions are
  /// unknown.
  std::optional<double> shortest_length(
    std::size_t from, std::size_t to,
    const std::unordered_set<int64_t> & closed) const;

  /// Waypoints of the shortest route (from..to) with \p closed shut; empty if none.
  std::vector<std::size_t> shortest_route(
    std::size_t from, std::size_t to,
    const std::unordered_set<int64_t> & closed) const;

  /// Waypoint nearest (x, y); nullopt without positions.
  std::optional<std::size_t> nearest(double x, double y) const;

private:
  std::vector<std::vector<std::size_t>> out_;
  std::vector<std::pair<double, double>> pos_;
  std::vector<bool> has_pos_;
};

}  // namespace ecbs_planner

#endif  // ECBS_PLANNER__NAV_GRAPH_HPP_
