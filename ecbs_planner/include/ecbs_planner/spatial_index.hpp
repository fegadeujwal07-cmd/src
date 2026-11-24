#ifndef ECBS_PLANNER__SPATIAL_INDEX_HPP_
#define ECBS_PLANNER__SPATIAL_INDEX_HPP_

#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "ecbs_planner/types.hpp"

namespace ecbs_planner
{

/// Grid hash for route-overlap candidate pruning (10–30 robot fleets).
class RouteSpatialIndex
{
public:
  explicit RouteSpatialIndex(double cell_m = 2.0)
  : cell_m_(cell_m > 1e-3 ? cell_m : 2.0)
  {
  }

  void clear()
  {
    cells_.clear();
    by_amr_.clear();
  }

  void upsert_path(const TimedPath & path)
  {
    erase_amr(path.amr_id);
    if (path.amr_id.empty()) {
      return;
    }
    std::unordered_set<int64_t> keys;
    for (const auto & w : path.waypoints) {
      keys.insert(key_of(w.x, w.y));
    }
    // Conservatively cover segment interiors so crossing pairs without shared
    // waypoint cells are still candidate-matched.
    for (std::size_t i = 1; i < path.waypoints.size(); ++i) {
      const auto & a = path.waypoints[i - 1];
      const auto & b = path.waypoints[i];
      const double len = std::hypot(b.x - a.x, b.y - a.y);
      const int steps = std::max(1, static_cast<int>(std::ceil(len / cell_m_)));
      for (int s = 0; s <= steps; ++s) {
        const double t = static_cast<double>(s) / static_cast<double>(steps);
        keys.insert(key_of(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)));
      }
    }
    for (const auto k : keys) {
      cells_[k].insert(path.amr_id);
      by_amr_[path.amr_id].insert(k);
    }
  }

  void erase_amr(const std::string & amr_id)
  {
    const auto it = by_amr_.find(amr_id);
    if (it == by_amr_.end()) {
      return;
    }
    for (const auto k : it->second) {
      auto cit = cells_.find(k);
      if (cit != cells_.end()) {
        cit->second.erase(amr_id);
        if (cit->second.empty()) {
          cells_.erase(cit);
        }
      }
    }
    by_amr_.erase(it);
  }

  /// Candidate peers whose cached cells touch \p path's cells.
  std::vector<std::string> candidates_for(const TimedPath & path) const
  {
    std::unordered_set<std::string> out;
    for (const auto & w : path.waypoints) {
      const int ix = static_cast<int>(std::floor(w.x / cell_m_));
      const int iy = static_cast<int>(std::floor(w.y / cell_m_));
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          const auto kit = cells_.find(pack(ix + dx, iy + dy));
          if (kit == cells_.end()) {
            continue;
          }
          for (const auto & id : kit->second) {
            if (id != path.amr_id) {
              out.insert(id);
            }
          }
        }
      }
    }
    return std::vector<std::string>(out.begin(), out.end());
  }

  std::size_t cell_count() const { return cells_.size(); }

private:
  int64_t key_of(double x, double y) const
  {
    return pack(
      static_cast<int>(std::floor(x / cell_m_)),
      static_cast<int>(std::floor(y / cell_m_)));
  }

  static int64_t pack(int ix, int iy)
  {
    return (static_cast<int64_t>(ix) << 32) ^
           (static_cast<uint32_t>(iy));
  }

  double cell_m_;
  std::unordered_map<int64_t, std::unordered_set<std::string>> cells_;
  std::unordered_map<std::string, std::unordered_set<int64_t>> by_amr_;
};

}  // namespace ecbs_planner

#endif  // ECBS_PLANNER__SPATIAL_INDEX_HPP_
