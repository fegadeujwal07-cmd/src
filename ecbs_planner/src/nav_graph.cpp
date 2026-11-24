#include "ecbs_planner/nav_graph.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <functional>
#include <queue>

namespace ecbs_planner
{

void NavGraph::add_lane(std::size_t entry, std::size_t exit)
{
  const std::size_t n = std::max(entry, exit) + 1;
  if (out_.size() < n) {
    out_.resize(n);
  }
  out_[entry].push_back(exit);
}

void NavGraph::set_position(std::size_t wp, double x, double y)
{
  if (pos_.size() <= wp) {
    pos_.resize(wp + 1);
    has_pos_.resize(wp + 1, false);
  }
  pos_[wp] = {x, y};
  has_pos_[wp] = true;
}

std::optional<std::pair<double, double>> NavGraph::position(std::size_t wp) const
{
  if (wp >= has_pos_.size() || !has_pos_[wp]) {
    return std::nullopt;
  }
  return pos_[wp];
}

std::vector<std::size_t> NavGraph::shortest_route(
  std::size_t from, std::size_t to,
  const std::unordered_set<int64_t> & closed) const
{
  const std::size_t n = out_.size();
  auto known = [&](std::size_t i) {return i < has_pos_.size() && has_pos_[i];};
  auto is_closed = [&](std::size_t i) {return closed.count(static_cast<int64_t>(i)) > 0;};
  if (from >= n || to >= n || !known(from) || is_closed(from) || is_closed(to)) {
    return {};
  }
  // Waypoints without a position (graphs built without coordinates, dock
  // waypoints the YAML loader inserts) must not act as walls: reachable()
  // ignores positions, so a detour it accepts was then "not found" here and
  // every replan attempt failed. Such hops cost 1 instead of their length.
  auto hop = [&](std::size_t a, std::size_t b) {
      if (!known(a) || !known(b)) {
        return 1.0;
      }
      return std::hypot(pos_[b].first - pos_[a].first, pos_[b].second - pos_[a].second);
    };
  std::vector<double> dist(n, std::numeric_limits<double>::infinity());
  std::vector<std::size_t> prev(n, n);
  using Item = std::pair<double, std::size_t>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
  dist[from] = 0.0;
  open.emplace(0.0, from);
  while (!open.empty()) {
    const auto [d, wp] = open.top();
    open.pop();
    if (d > dist[wp]) {
      continue;
    }
    if (wp == to) {
      std::vector<std::size_t> route{to};
      while (route.back() != from) {
        route.push_back(prev[route.back()]);
      }
      std::reverse(route.begin(), route.end());
      return route;
    }
    for (const std::size_t next : out_[wp]) {
      if (is_closed(next)) {
        continue;
      }
      const double nd = d + hop(wp, next);
      if (nd < dist[next]) {
        dist[next] = nd;
        prev[next] = wp;
        open.emplace(nd, next);
      }
    }
  }
  return {};
}

std::optional<double> NavGraph::shortest_length(
  std::size_t from, std::size_t to,
  const std::unordered_set<int64_t> & closed) const
{
  const auto route = shortest_route(from, to, closed);
  if (route.empty()) {
    return std::nullopt;
  }
  // A length in metres needs every waypoint's position; shortest_route() may
  // cross unpositioned ones (hop cost), so the length is then unknown.
  for (const auto wp : route) {
    if (wp >= has_pos_.size() || !has_pos_[wp]) {
      return std::nullopt;
    }
  }
  double len = 0.0;
  for (std::size_t i = 1; i < route.size(); ++i) {
    len += std::hypot(
      pos_[route[i]].first - pos_[route[i - 1]].first,
      pos_[route[i]].second - pos_[route[i - 1]].second);
  }
  return len;
}

std::optional<std::size_t> NavGraph::nearest(double x, double y) const
{
  std::optional<std::size_t> best;
  double best_d = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < pos_.size(); ++i) {
    const double d = has_pos_[i] ? std::hypot(pos_[i].first - x, pos_[i].second - y) : best_d;
    if (d < best_d) {
      best_d = d;
      best = i;
    }
  }
  return best;
}

bool NavGraph::reachable(
  std::size_t from, std::size_t to,
  const std::unordered_set<int64_t> & closed) const
{
  if (from >= out_.size() || to >= out_.size()) {
    return false;
  }
  if (from == to) {
    return true;
  }
  auto is_closed = [&](std::size_t wp) {
      return closed.count(static_cast<int64_t>(wp)) > 0;
    };
  if (is_closed(from) || is_closed(to)) {
    return false;
  }
  std::vector<bool> seen(out_.size(), false);
  std::queue<std::size_t> open;
  open.push(from);
  seen[from] = true;
  while (!open.empty()) {
    const std::size_t wp = open.front();
    open.pop();
    for (const std::size_t next : out_[wp]) {
      if (seen[next] || is_closed(next)) {
        continue;
      }
      if (next == to) {
        return true;
      }
      seen[next] = true;
      open.push(next);
    }
  }
  return false;
}

}  // namespace ecbs_planner
