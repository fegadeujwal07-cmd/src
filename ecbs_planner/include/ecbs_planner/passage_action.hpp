#ifndef ECBS_PLANNER__PASSAGE_ACTION_HPP_
#define ECBS_PLANNER__PASSAGE_ACTION_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "ecbs_planner/conflict_store.hpp"
#include "ecbs_planner/route_occurrence.hpp"
#include "ecbs_planner/types.hpp"

namespace ecbs_planner
{

enum class PassageActionState
{
  Eligible = 0,
  Proposed,
  PendingCommit,
  Committed,
  InProgress,
  Done,
  Held,
};

/// Movement interval on a fixed route — not a whole-robot winner.
struct PassageAction
{
  std::string action_id;
  std::string amr_id;
  std::string task_id;              // canonical
  std::string execution_order_id;
  uint32_t path_revision{0};
  RouteOccurrence from;
  RouteOccurrence to;
  int32_t from_index{0};
  int32_t to_index{0};
  /// Clearance obligations (not exclusivity proof).
  std::vector<std::string> reserved_conflict_ids;
  /// Downstream free space required past exit (metres).
  double downstream_space_m{0.0};
  std::vector<std::string> predecessor_action_ids;
  std::vector<std::string> predecessor_clearance_ids;
  std::string task_priority;
  double waiting_age_sec{0.0};
  int bypass_count{0};
  uint64_t proposal_id{0};
  uint64_t grant_id{0};
  PassageActionState exec_state{PassageActionState::Eligible};
  bool is_approach{false};
  bool safe_hold_verified{false};
  std::string hold_reason;
};

struct PassageActionBuildInput
{
  std::string amr_id;
  const TimedPath * path{nullptr};
  int32_t from_index{0};
  double desired_extension_m{20.0};
  std::string task_id;
  std::string execution_order_id;
  std::vector<ConflictRecord> relevant;
  /// Occupied exit / foreign bases that block a hold past an overlap.
  bool exit_blocked{false};
  double footprint_length_m{1.2};
};

/// Build one or more actions: approach-to-wait, or passage through overlaps to
/// a genuinely safe hold. Merges consecutive overlaps when no mid stop exists.
std::vector<PassageAction> build_passage_actions(const PassageActionBuildInput & in);

/// Verify full safe-hold: approach free, body can clear overlap, stop free,
/// stop does not block unresolved passage, interval reservable.
bool verify_safe_hold(
  const PassageAction & action,
  const TimedPath & path,
  const std::vector<ConflictRecord> & active,
  bool exit_space_free);

}  // namespace ecbs_planner

#endif  // ECBS_PLANNER__PASSAGE_ACTION_HPP_
