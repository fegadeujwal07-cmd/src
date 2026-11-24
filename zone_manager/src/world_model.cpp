#include "zone_manager/world_model.hpp"

#include <algorithm>

namespace zone_manager
{

std::vector<std::string> RobotState::reserved_zones() const
{
  std::vector<std::string> out;
  out.reserve(kMaxReservedZonesPerRobot);
  auto push_unique = [&out](const std::string & z) {
      if (z.empty()) {
        return;
      }
      if (std::find(out.begin(), out.end(), z) == out.end()) {
        out.push_back(z);
      }
    };

  push_unique(current_zone);
  for (std::size_t i = 0; i < next_count && i < next_zones.size(); ++i) {
    push_unique(next_zones[i]);
    if (out.size() >= kMaxReservedZonesPerRobot) {
      break;
    }
  }
  return out;
}

std::vector<std::string> RobotState::occupied_zones() const
{
  std::vector<std::string> out = reserved_zones();
  for (const auto & zone : rear_zones) {
    if (zone.empty()) {
      continue;
    }
    if (std::find(out.begin(), out.end(), zone) == out.end()) {
      out.push_back(zone);
    }
  }
  return out;
}

void WorldModel::ensure_free_slots()
{
  if (free_slots_ready_) {
    return;
  }
  free_slots_.reserve(kMaxRobots);
  for (std::size_t i = 0; i < kMaxRobots; ++i) {
    free_slots_.push_back(static_cast<uint8_t>(i));
  }
  free_slots_ready_ = true;
}

std::optional<uint8_t> WorldModel::acquire_slot(const std::string & robot_id)
{
  const auto it = robot_to_slot_.find(robot_id);
  if (it != robot_to_slot_.end()) {
    return it->second;
  }

  ensure_free_slots();
  if (free_slots_.empty()) {
    return std::nullopt;
  }

  const uint8_t slot = free_slots_.back();
  free_slots_.pop_back();
  slots_[slot] = Slot{};
  slots_[slot].in_use = true;
  slots_[slot].state.robot_id = robot_id;
  robot_to_slot_[robot_id] = slot;
  return slot;
}

void WorldModel::release_slot_if_idle(uint8_t slot)
{
  if (slot >= kMaxRobots || !slots_[slot].in_use) {
    return;
  }
  const auto & st = slots_[slot].state;
  if (!st.current_zone.empty() || st.next_count > 0 || st.pose.valid) {
    return;
  }
  if (!slots_[slot].applied_zones.empty()) {
    return;
  }
  robot_to_slot_.erase(slots_[slot].state.robot_id);
  slots_[slot] = Slot{};
  free_slots_.push_back(slot);
}

void WorldModel::mark_dirty(const std::string & robot_id)
{
  dirty_robots_.insert(robot_id);
}

void WorldModel::register_zones(const std::vector<std::string> & zone_ids)
{
  for (const auto & id : zone_ids) {
    if (!id.empty()) {
      (void)zones_[id];
    }
  }
}

void WorldModel::update_pose(const std::string & robot_id, double x, double y)
{
  if (robot_id.empty()) {
    return;
  }
  const auto slot_opt = acquire_slot(robot_id);
  if (!slot_opt) {
    return;
  }
  auto & st = slots_[*slot_opt].state;
  st.pose.x = x;
  st.pose.y = y;
  st.pose.valid = true;
  st.stamp = std::chrono::steady_clock::now();
  // Pose alone does not change reservations — no dirty for occupancy.
}

void WorldModel::update_zones(
  const std::string & robot_id,
  const std::string & current_zone,
  const std::vector<std::string> & next_zones,
  const std::vector<std::string> & zone_sequence)
{
  if (robot_id.empty()) {
    return;
  }
  const auto slot_opt = acquire_slot(robot_id);
  if (!slot_opt) {
    return;
  }

  auto & st = slots_[*slot_opt].state;
  st.current_zone = current_zone;
  st.next_count = 0;
  for (const auto & z : next_zones) {
    if (z.empty() || st.next_count >= st.next_zones.size()) {
      continue;
    }
    st.next_zones[st.next_count++] = z;
  }
  st.zone_sequence.clear();
  st.zone_sequence.reserve(zone_sequence.size());
  for (const auto & z : zone_sequence) {
    if (!z.empty()) {
      st.zone_sequence.push_back(z);
    }
  }
  st.stamp = std::chrono::steady_clock::now();
  mark_dirty(robot_id);
}

void WorldModel::update_rear_zones(
  const std::string & robot_id,
  const std::vector<std::string> & rear_zones)
{
  if (robot_id.empty()) {
    return;
  }
  const auto slot_opt = acquire_slot(robot_id);
  if (!slot_opt) {
    return;
  }
  auto & st = slots_[*slot_opt].state;
  if (st.rear_zones == rear_zones) {
    return;  // nothing to re-apply
  }
  st.rear_zones = rear_zones;
  st.stamp = std::chrono::steady_clock::now();
  // Rear zones ARE part of occupied_zones(), so occupancy must be re-synced.
  mark_dirty(robot_id);
}

void WorldModel::clear_robot(const std::string & robot_id)
{
  const auto it = robot_to_slot_.find(robot_id);
  if (it == robot_to_slot_.end()) {
    return;
  }
  auto & st = slots_[it->second].state;
  st.current_zone.clear();
  st.next_count = 0;
  st.next_zones = {};
  st.zone_sequence.clear();
  st.pose = Pose2D{};
  st.stamp = std::chrono::steady_clock::now();
  mark_dirty(robot_id);
}

std::unordered_set<std::string> WorldModel::sync_dirty_occupancy()
{
  std::unordered_set<std::string> affected;
  if (dirty_robots_.empty()) {
    return affected;
  }

  std::vector<std::string> dirty(dirty_robots_.begin(), dirty_robots_.end());
  dirty_robots_.clear();

  for (const auto & robot_id : dirty) {
    const auto it = robot_to_slot_.find(robot_id);
    if (it == robot_to_slot_.end()) {
      continue;
    }
    const uint8_t slot = it->second;
    auto & entry = slots_[slot];

    // occupied_zones(), not reserved_zones(): rear zones must raise a conflict
    // when another robot drives into them, without joining the distance-ordered
    // list the pause rules rank by.
    const std::vector<std::string> new_zones = entry.state.occupied_zones();
    const std::vector<std::string> & old_zones = entry.applied_zones;

    std::unordered_set<std::string> old_set(old_zones.begin(), old_zones.end());
    std::unordered_set<std::string> new_set(new_zones.begin(), new_zones.end());

    for (const auto & zone_id : old_zones) {
      if (new_set.count(zone_id) > 0) {
        continue;
      }
      auto & bits = zones_[zone_id].occupants;
      bits.reset(slot);
      affected.insert(zone_id);
      if (bits.none()) {
        // Keep empty zone entry if it was pre-registered from catalog;
        // still fine to leave a zero bitset.
      }
    }

    for (const auto & zone_id : new_zones) {
      if (old_set.count(zone_id) > 0) {
        continue;
      }
      zones_[zone_id].occupants.set(slot);
      affected.insert(zone_id);
    }

    entry.applied_zones = new_zones;
    if (new_zones.empty()) {
      release_slot_if_idle(slot);
    }
  }

  return affected;
}

const RobotState * WorldModel::robot(const std::string & robot_id) const
{
  const auto it = robot_to_slot_.find(robot_id);
  if (it == robot_to_slot_.end()) {
    return nullptr;
  }
  return &slots_[it->second].state;
}

std::vector<std::string> WorldModel::robots_in_zone(const std::string & zone_id) const
{
  std::vector<std::string> robots;
  const auto it = zones_.find(zone_id);
  if (it == zones_.end()) {
    return robots;
  }
  const auto & bits = it->second.occupants;
  for (std::size_t i = 0; i < kMaxRobots; ++i) {
    if (!bits.test(i)) {
      continue;
    }
    if (slots_[i].in_use && !slots_[i].state.robot_id.empty()) {
      robots.push_back(slots_[i].state.robot_id);
    }
  }
  return robots;
}

std::size_t WorldModel::occupant_count(const std::string & zone_id) const
{
  const auto it = zones_.find(zone_id);
  if (it == zones_.end()) {
    return 0;
  }
  return it->second.occupants.count();
}

bool WorldModel::has_zone(const std::string & zone_id) const
{
  return zones_.count(zone_id) > 0;
}

}  // namespace zone_manager
