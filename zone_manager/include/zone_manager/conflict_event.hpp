#ifndef ZONE_MANAGER__CONFLICT_EVENT_HPP_
#define ZONE_MANAGER__CONFLICT_EVENT_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace zone_manager
{

enum class ConflictEventType : uint8_t
{
  Detected = 0,
  Resolved = 1,
};

/// Internal conflict edge event (ROS-free). Published via Conflict.msg at the boundary.
struct ConflictEvent
{
  ConflictEventType type{ConflictEventType::Detected};
  std::string zone_id;
  std::vector<std::string> robot_ids;
};

}  // namespace zone_manager

#endif  // ZONE_MANAGER__CONFLICT_EVENT_HPP_
