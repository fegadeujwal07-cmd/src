#include "zone_manager/conflict_detector.hpp"

namespace zone_manager
{

std::vector<ConflictEvent> ConflictDetector::evaluate(
  const WorldModel & world,
  const std::unordered_set<std::string> & affected)
{
  std::vector<ConflictEvent> events;
  events.reserve(affected.size());

  for (const auto & zone_id : affected) {
    const std::size_t count = world.occupant_count(zone_id);
    const bool was_conflict = active_conflicts_.count(zone_id) > 0;
    const bool is_conflict = count > 1;

    if (!was_conflict && is_conflict) {
      active_conflicts_.insert(zone_id);
      ConflictEvent ev;
      ev.type = ConflictEventType::Detected;
      ev.zone_id = zone_id;
      ev.robot_ids = world.robots_in_zone(zone_id);
      events.push_back(std::move(ev));
    } else if (was_conflict && !is_conflict) {
      active_conflicts_.erase(zone_id);
      ConflictEvent ev;
      ev.type = ConflictEventType::Resolved;
      ev.zone_id = zone_id;
      ev.robot_ids = world.robots_in_zone(zone_id);
      events.push_back(std::move(ev));
    }
  }

  return events;
}

}  // namespace zone_manager
