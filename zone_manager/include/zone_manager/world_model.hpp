#ifndef ZONE_MANAGER__WORLD_MODEL_HPP_
#define ZONE_MANAGER__WORLD_MODEL_HPP_

#include <array>
#include <bitset>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace zone_manager
{

constexpr std::size_t kMaxRobots = 128;
constexpr std::size_t kMaxReservedZonesPerRobot = 4;  // current + next 3

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  bool valid{false};
};

struct RobotState
{
  std::string robot_id;
  std::string current_zone;
  std::array<std::string, 3> next_zones{};
  std::size_t next_count{0};
  /// Remaining task zones from current onward (from RobotZoneStatus.zone_sequence).
  std::vector<std::string> zone_sequence;
  Pose2D pose;
  std::chrono::steady_clock::time_point stamp{};

  /// Zones lane-reachable from behind the robot (monitor computes these from
  /// pose + heading + nav graph). The space another AMR could drive into and
  /// rear-end this one.
  std::vector<std::string> rear_zones;

  /// All reserved zones for this robot: current + next≤3 (deduped, max 4).
  ///
  /// Deliberately EXCLUDES rear_zones. This list is ordered by distance ahead
  /// along the route, and both pause rules rank robots by a zone's index in it
  /// (see conflict_decision.cpp). A rear zone is behind, so it has no valid
  /// position on that scale: appended last it looks "farthest ahead" and would
  /// pause the LEADER instead of the follower; inserted first it shifts every
  /// other zone down and silently re-tunes the near/far threshold for the whole
  /// fleet. It belongs in occupied_zones() only.
  std::vector<std::string> reserved_zones() const;

  /// Everything this robot claims: reserved + rear. Used for occupancy, so a
  /// robot driving into another's rear zone raises a conflict, while the
  /// ranking above stays untouched.
  std::vector<std::string> occupied_zones() const;
};

/// Authoritative runtime state: robot states + zone occupancy bitsets + dirty set.
///
/// Subscribers only write robot/pose/zone fields and mark dirty.
/// Timer calls sync_dirty_occupancy() then ConflictDetector reads bitsets.
class WorldModel
{
public:
  void register_zones(const std::vector<std::string> & zone_ids);

  void update_pose(const std::string & robot_id, double x, double y);

  /// Set current + next≤3 (+ optional full zone_sequence). Marks dirty.
  void update_zones(
    const std::string & robot_id,
    const std::string & current_zone,
    const std::vector<std::string> & next_zones,
    const std::vector<std::string> & zone_sequence = {});

  /// Rear zones have their OWN setter on purpose.
  ///
  /// Only RobotZoneStatus carries them; MonitoredTaskDetails does not, and it
  /// also calls update_zones(). Had rear zones been a defaulted parameter there,
  /// every task refresh would have passed {} and silently wiped a robot's rear
  /// protection. Keeping them separate makes that impossible for any caller.
  void update_rear_zones(
    const std::string & robot_id,
    const std::vector<std::string> & rear_zones);

  void clear_robot(const std::string & robot_id);

  /// Apply occupancy bitset deltas for all dirty robots.
  /// Returns only zones that changed (symmetric leave/enter). O(≤4) per robot.
  std::unordered_set<std::string> sync_dirty_occupancy();

  const RobotState * robot(const std::string & robot_id) const;

  std::vector<std::string> robots_in_zone(const std::string & zone_id) const;

  std::size_t occupant_count(const std::string & zone_id) const;

  bool has_zone(const std::string & zone_id) const;

  std::size_t robot_count() const { return robot_to_slot_.size(); }

  std::size_t zone_count() const { return zones_.size(); }

  const std::unordered_set<std::string> & dirty_robots() const { return dirty_robots_; }

private:
  struct ZoneRuntime
  {
    std::bitset<kMaxRobots> occupants;
  };

  struct Slot
  {
    RobotState state;
    /// Last applied reserved zones in the occupancy bitsets.
    std::vector<std::string> applied_zones;
    bool in_use{false};
  };

  std::optional<uint8_t> acquire_slot(const std::string & robot_id);
  void release_slot_if_idle(uint8_t slot);
  void ensure_free_slots();
  void mark_dirty(const std::string & robot_id);

  std::unordered_map<std::string, uint8_t> robot_to_slot_;
  std::array<Slot, kMaxRobots> slots_{};
  std::unordered_map<std::string, ZoneRuntime> zones_;
  std::unordered_set<std::string> dirty_robots_;
  std::vector<uint8_t> free_slots_;
  bool free_slots_ready_{false};
};

}  // namespace zone_manager

#endif  // ZONE_MANAGER__WORLD_MODEL_HPP_
