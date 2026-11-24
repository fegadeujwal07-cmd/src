#ifndef ECBS_PLANNER__PASSAGE_ARBITRATION_HPP_
#define ECBS_PLANNER__PASSAGE_ARBITRATION_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ecbs_planner/conflict_store.hpp"

namespace ecbs_planner
{

inline constexpr const char * kStatusFreeProposal = "FREE_PROPOSAL";
inline constexpr const char * kStatusWait = "WAIT";
inline constexpr const char * kStatusStaleRoute = "STALE_ROUTE";
inline constexpr const char * kStatusRecoveryRequired = "RECOVERY_REQUIRED";
inline constexpr const char * kStatusAuthorityConflict = "AUTHORITY_CONFLICT";
inline constexpr const char * kStatusUnresolvedOpposite = "unresolved_opposite";

inline constexpr const char * kGrantCommitted = "COMMITTED";
inline constexpr const char * kGrantRejected = "REJECTED";
inline constexpr const char * kGrantStale = "STALE";
inline constexpr const char * kGrantSuperseded = "SUPERSEDED";
/// Passage finished / grant no longer authoritative (distinct from clearance).
inline constexpr const char * kGrantCompleted = "COMPLETED";

struct RobotPriorityFacts
{
  std::string robot_id;
  std::string task_id;
  std::string task_priority;
  uint64_t priority_version{0};
  bool healthy{true};
  bool available{true};
  /// Progress on current route (occurrence). Used for B2B front/rear.
  uint32_t progress_occurrence{0};
  bool progress_valid{false};
  /// False when foreign base / occupancy / no exit blocks independent motion.
  bool physically_feasible{true};
  /// Observed pose for longitudinal B2B distance (metres along overlap).
  double progress_x{0.0};
  double progress_y{0.0};
  bool progress_xy_valid{false};
  /// Seconds waiting in cluster (bounded fairness among feasible candidates).
  double waiting_age_sec{0.0};
};

struct PassageDecision
{
  std::vector<std::string> go_robots;
  std::vector<std::string> wait_robots;
  /// Mandatory wait assignments to write back: conflict_id → waiter,blocker
  std::vector<std::pair<std::string, std::pair<std::string, std::string>>>
    mandatory_assignments;
  std::string reason;
  std::string preference_kind;
  bool unresolved_opposite{false};
  bool recovery_required{false};
  std::string debug_summary;
};

inline int task_priority_rank(const std::string & priority)
{
  if (priority.empty()) {
    return 0;
  }
  if (priority == "low" || priority == "0" || priority == "false") {
    return 1;
  }
  return 2;
}

inline int easiness_score(bool self_blocks_other, bool other_blocks_self)
{
  if (self_blocks_other == other_blocks_self) {
    return 0;
  }
  return other_blocks_self ? 1 : -1;
}

/// Fold per-overlap easiness flags for \p id within \p members (1 go / -1 yield / 0 tie).
int cluster_easiness_for(
  const std::vector<ConflictRecord> & records,
  const std::unordered_set<std::string> & members,
  const std::string & id);

/// Decision order: feasibility → opposite HardHold → B2B → easiness → priority.
/// Opposite HardHold applies even when reciprocal edges form a cycle.
/// \p current_task_by_robot: only matching task_id priority facts are used.
PassageDecision arbitrate_passage(
  const ConflictCluster & cluster,
  const std::vector<ConflictRecord> & records,
  const std::unordered_map<std::string, RobotPriorityFacts> & facts,
  const std::unordered_set<std::string> & already_granted_go,
  const std::unordered_map<std::string, std::string> & current_task_by_robot = {});

int b2b_front_rear_from_progress(
  uint32_t a_progress_occ,
  uint32_t b_progress_occ);

}  // namespace ecbs_planner

#endif  // ECBS_PLANNER__PASSAGE_ARBITRATION_HPP_
