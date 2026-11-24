#ifndef ZONE_MANAGER__CONFLICT_DECISION_HPP_
#define ZONE_MANAGER__CONFLICT_DECISION_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "zone_manager/replan_priority.hpp"

namespace zone_manager
{

enum class ConflictClass : uint8_t
{
  Unknown = 0,
  /// Straight corridor, same travel direction (back-to-back).
  StraightSameDirection = 1,
  /// Straight corridor, head-on.
  StraightOpposite = 2,
  /// Inverted-T / merge, same post-junction direction.
  JunctionSameDirection = 3,
  /// Inverted-T / merge, opposite post-junction direction.
  JunctionOpposite = 4,
  /// Through-traffic on main line vs branch joining against it.
  BranchVsThrough = 5,
};

enum class TravelDirection : uint8_t
{
  Unknown = 0,
  Same = 1,
  Opposite = 2,
};

struct ConflictRobotInput
{
  std::string robot_id;
  std::vector<std::string> reserved_zones;
  std::vector<std::string> zone_sequence;
  /// Zones behind this robot that another AMR could drive into and rear-end it.
  std::vector<std::string> rear_zones;
  /// From rmf_fleet_msgs/RobotMode.mode (legacy, kept for reference)
  uint32_t fleet_mode{0};
  bool mode_known{false};
  /// VDA5050 operating mode from /fleet_states. -1 = unknown, 0 = unhealthy, 1 = healthy.
  int16_t operating_mode{-1};
  RobotPriorityFacts priority;
};

struct ConflictDecision
{
  ConflictClass conflict_class{ConflictClass::Unknown};
  TravelDirection direction{TravelDirection::Unknown};
  /// Robots that should receive startPause.
  std::vector<std::string> pause_robot_ids;
  /// Set for opposite classes (C2/C4): robot to publish on /replan_candidate.
  std::string replan_candidate_id;
  std::string replan_reason;
  std::string debug_summary;
};

/// Unhealthy: MODE_EMERGENCY (5) or MODE_ADAPTER_ERROR (8). Unknown mode → healthy.
/// Legacy check kept for reference; opposite-conflict apex uses operating_mode instead.
bool is_robot_healthy(uint32_t fleet_mode, bool mode_known);

/// VDA5050 operating mode health check (apex for opposite-conflict replan).
/// Only operating_mode == 1 is healthy. 0 and -1 are unhealthy.
bool is_operating_mode_healthy(int16_t operating_mode);

TravelDirection classify_travel_direction(
  const std::vector<std::string> & seq_a,
  const std::vector<std::string> & seq_b,
  const std::string & conflict_zone);

/// True if approach paths before conflict share no zones (distinct branches/arms).
bool is_junction_conflict(
  const std::vector<std::string> & seq_a,
  const std::vector<std::string> & seq_b,
  const std::string & conflict_zone);

/// True when rem_self is a non-empty prefix of rem_other (self finishes on other's path).
bool remaining_blocks_other(
  const std::vector<std::string> & rem_self,
  const std::vector<std::string> & rem_other);

/// Decide pause set (+ optional replan candidate) for a detected zone conflict.
/// For N!=2 robots, falls back to behind-index pause (same as legacy).
ConflictDecision decide_conflict_action(
  const std::string & conflict_zone,
  const std::vector<ConflictRobotInput> & robots);

}  // namespace zone_manager

#endif  // ZONE_MANAGER__CONFLICT_DECISION_HPP_
