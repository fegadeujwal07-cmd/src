#ifndef ZONE_MANAGER__CONFLICT_DETECTOR_HPP_
#define ZONE_MANAGER__CONFLICT_DETECTOR_HPP_

#include <string>
#include <unordered_set>
#include <vector>

#include "zone_manager/conflict_event.hpp"
#include "zone_manager/world_model.hpp"

namespace zone_manager
{

/// Pure algorithm: no ROS. Reads WorldModel occupancy; emits edge events only.
class ConflictDetector
{
public:
  /// Evaluate only `affected` zones. count()>1 → conflict.
  /// Publishes Detected/Resolved only on state transitions (no duplicates).
  std::vector<ConflictEvent> evaluate(
    const WorldModel & world,
    const std::unordered_set<std::string> & affected);

  const std::unordered_set<std::string> & active_conflicts() const
  {
    return active_conflicts_;
  }

private:
  std::unordered_set<std::string> active_conflicts_;
};

}  // namespace zone_manager

#endif  // ZONE_MANAGER__CONFLICT_DETECTOR_HPP_
